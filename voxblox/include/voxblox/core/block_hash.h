#ifndef VOXBLOX_BLOCK_HASH_H_
#define VOXBLOX_BLOCK_HASH_H_

#include <atomic>
#include <functional>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

// Simple concurrent map.
// Most operations are thread safe, however inserting an element will break all
// existing iterators.
// Elements inserted during a rehash may be lost, elements deleted during a
// rehash may appear back.
// Each value is returned in a data struct that can be used to lock and unlock
// the value to ensure thread safe modification. The user is under no obligation
// to actually use or respect the lock.
// Multiple threads can read existing elements, create new elements and delete
// elements simultaneously without issues.
// Under the hood all data is stored using shared pointers that are organized
// into linked lists in an array. Each list has one atomic boolean controlling
// access for insertion and deletion.
// The hash rather than the index is used to find if two values are the same,
// this is quick but can cause two elements to be erroneously seen as belonging
// together.
//
template <typename ValueType>
class ConcurrentHashMap {
 public:
  class Data {
    pubilc :

        Data(ValueType& value, std::atomic_flag& lock_flag)
        : value_(value), lock_flag_(lock_flag){};

    ValueType& value() { return value_; }

    void unlock() { lock_flag_.clear(); }

    bool tryToLock() { return !lock_flag_.test_and_set(); }

    // spins until locked
    void lock() {
      while (lock_flag_.test_and_set())
        ;
    }

   private:
    ValueType& value_;
    std::atomic_flag& lock_flag_;
  };

  // should be an stl iterator, but I am lazy.
  // thread safe... as long as a rehash does not happen.
  // if a rehash does occur (fellowed by some insert / deletes) things get crazy
  // ...seriously I drew it out and ++ could move to literally any other element
  class PseudoIterator {
   public:
    PseudoIterator(const std::shared_ptr<Node>& node_ptr,
                   const size_t bucket_idx,
                   const std::shared_ptr<std::vector<std::shared_ptr<Bucket>>>&
                       data_buckets)
        : node_ptr_(node_ptr),
          bucket_idx_(bucket_idx),
          data_buckets_(data_buckets) {}

    PseudoIterator& operator++() {
      node_ptr = node_ptr_->next_element while (
          (node_ptr == nullptr) && (bucket_idx_ < data_buckets_.size())) {
        node_ptr_ = data_buckets_[bucket_idx_++].nodePtr();
      }
    }

    Data& operator*() { return node_ptr_->value; }

    bool operator==(const PseudoIterator& other) {
      return (bucket_idx_ == other.bucket_idx_) &&
             (node_ptr_ == other.node_ptr_);
    }

   private:
    std::shared_ptr<Node> node_ptr_;
    size_t bucket_idx_;
    std::shared_ptr<std::vector<std::shared_ptr<Bucket>>> data_buckets_;
  };

  ConcurrentHashMap() : ConcurrentHashMap(1) {}

  ConcurrentHashMap(size_t num_inital_buckets) {
    // all logic is based on the assumption there is at least 1 bucket
    if (num_inital_buckets == 0) {
      ++num_inital_buckets;
    }

    data_buckets_ = std::make_shared<std::vector<std::shared_ptr<Bucket>>>(
        num_inital_buckets);
  }

  // thread safe
  bool tryFind(const AnyIndex& index, ValueType* value) const {
    size_t hash;
    std::shared_ptr<Bucket> bucket;
    std::shared_ptr<Node> node_ptr;

    if (getNodePtr(index, &hash, &bucket_ptr, &node_ptr)) {
      value = &(node_ptr->value_);
      return true;
    } else {
      return false;
    }
  }

  // thread safe
  bool tryFind(const AnyIndex& index, ValueType& value) const {
    return tryFind(index, &value);
  }

  // will create element if it does not exist, thread safe
  ValueType& findOrCreate(const AnyIndex& index) {
    std::shared_ptr<Node> node_ptr;

    getOrCreateDataPtr(index, &node_ptr);

    return node_ptr->value;
  }

  // thread safe
  bool erase(const AnyIndex& index) {
    size_t hash = hashFunction(index);
    std::shared_ptr<Bucket>& bucket_ptr =
        data_buckets_->at(hash % data_buckets_->size());

    bucket_ptr->lock();

    std::shared_ptr<Node> prev_node_ptr;
    std::shared_ptr<Node>& node_ptr = bucket_ptr->nodePtr();

    // iterate through the list in the bucket until at the correct hash or at
    // the end
    while (node_ptr != nullptr || node_ptr->hash_ != hash) {
      prev_node_ptr = node_ptr;
      node_ptr = node_ptr->next_node_;
    }

    if (node_ptr == nullptr) {
      // nothing to erase
      bucket_ptr->unlock();
      return false;
    }

    if (prev_data_ptr != nullptr) {
      prev_data_ptr->next_node_ = node_ptr->next_node_;
    }

    --num_elements_;

    bucket_ptr->unlock();

    return true;
  }

  size_t size() const { return num_elements_.load(); }

  PseudoIterator begin() {
    PseudoIterator it(nullptr, 0, data_buckets_);
    return ++it;
  }

  PseudoIterator end() {
    return PseudoIterator(nullptr, data_buckets_.size(), nullptr);
  }

 private:
  class Bucket {
   public:
    std::shared_ptr<Node>& nodePtr() { return node_ptr_; }

    void unlock() { write_lock_flag_.clear(); }

    bool tryToLock() { return !write_lock_flag_.test_and_set(); }

    // spins until locked
    void lock() {
      while (write_lock_flag_.test_and_set())
        ;
    }

   private:
    std::shared_ptr<Node> node_ptr;
    std::atomic_flag write_lock_flag_;
  }

  struct Node {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Node(const size_t hash) : hash_(hash){};

    size_t hash_;
    std::atomic_flag lock_flag_;
    ValueType value_;
    std::shared_ptr<Node> next_node_;
  };

  // thread safe, though the data can be outdated
  bool getNodePtr(const AnyIndex& index, size_t* hash,
                  std::shared_ptr<Bucket>* bucket_ptr,
                  std::shared_ptr<Node>* node_ptr) const {
    *hash = hashFunction(index);
    *bucket_ptr = data_buckets_->at(hash % data_buckets_->size());
    *node_ptr = bucket_ptr->nodePtr();

    // iterate through the list in the bucket until at the correct hash or at
    // the end
    while (*node_ptr != nullptr || (*node_ptr)->hash_ != *hash) {
      *node_ptr = (*node_ptr)->next_node_;
    }

    return *node_ptr != nullptr
  }

  // if the data does not exist it will be created, thread safe
  void getOrCreateNodePtr(const AnyIndex& index,
                          std::shared_ptr<Node>* node_ptr) {
    size_t hash;
    std::shared_ptr<Bucket> bucket_ptr;

    if (!getDataPtr(index, &hash, &bucket_ptr, node_ptr)) {
      // the data does not exist so we are going to create and add it

      // get exclusive write access to list
      bucket_ptr.lock();

      *node_ptr = std::make_shared<Node>(hash);

      ++num_elements_;

      bucket_ptr.unlock();

      rehash();
    }
  }

  // rehashes if the average elements per bucket is over 0.5
  // the rehash brings this down to 0.25
  // at least a 50% chance of a memory leak if you create data while its running
  void rehash(const size_t min_size = 1) {
    static constexpr float max_load_factor =
        0.5f;  // a bit on the low side but blocks are cheap
    static constexpr float target_load_factor = 0.5f * max_load_factor;
    static constexpr float inv_target_load_factor = 1.0f / target_load_factor;

    const float load_fator = static_cast<float>(num_elements_) /
                             static_cast<float>(data_buckets_->size());

    // check if rehash needed
    if (load_fator > max_load_factor) {
      size_t new_size = std::max(
          min_size, static_cast<size_t>(inv_target_load_factor *
                                        static_cast<float>(num_elements_)));
      std::shared_ptr<std::vector<std::shared_ptr<Node>>> new_buckets =
          std::make_shared<std::vector<std::shared_ptr<Node>>>(new_size);

      Index index = begin();
      while (index++ != end()) {
        new_buckets->at(index.data_ptr_->hash % new_buckets->size())
            .store(index_.data_ptr, std::memory_order_relaxed);
      }

      data_buckets_.swap(new_buckets);
    }
  }

  const size_t static inline hashFunction(const BlockIndex& index) {
    static constexpr size_t prime1 = 73856093;
    static constexpr size_t prime2 = 19349663;
    static constexpr size_t prime3 = 83492791;

    return (static_cast<size_t>(index.x()) * prime1 ^ index.y() * prime2 ^
            index.z() * prime3);
  }

  std::atomic<size_t> num_elements_;
  std::shared_ptr<std::vector<std::shared_ptr<Bucket>>> data_buckets_;
};

struct BlockIndexHash {
  static constexpr size_t prime1 = 73856093;
  static constexpr size_t prime2 = 19349663;
  static constexpr size_t prime3 = 83492791;

  std::size_t operator()(const BlockIndex& index) const {
    return (static_cast<unsigned int>(index.x()) * prime1 ^ index.y() * prime2 ^
            index.z() * prime3);
  }
};

/*template <typename ValueType>
struct BlockHashMapType {
  typedef std::unordered_map<
      BlockIndex, ValueType, BlockIndexHash, std::equal_to<BlockIndex>,
      Eigen::aligned_allocator<std::pair<const BlockIndex, ValueType>>>
      type;
};*/

template <typename ValueType>
struct BlockHashMapType {
  typedef ConcurrentHashMap<ValueType> type;
};

typedef std::unordered_set<AnyIndex, BlockIndexHash, std::equal_to<AnyIndex>,
                           Eigen::aligned_allocator<AnyIndex>>
    IndexSet;

typedef typename BlockHashMapType<IndexVector>::type HierarchicalIndexMap;

// typedef typename HierarchicalIndexMap::value_type HierarchicalIndex;

}  // namespace voxblox

#endif  // VOXBLOX_BLOCK_HASH_H_
