#ifndef VOXBLOX_BLOCK_HASH_H_
#define VOXBLOX_BLOCK_HASH_H_

#include <atomic>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

// Simple concurrent map.
// Rehashing is not thread safe, and must be done manually in concurrent use.
// Check the comments on each function for how thread safe it is.
// Multiple threads can read existing elements and create new elements
// simultaneously without issues.
// Under the hood all data is stored using atomic pointers that are organized
// into linked lists in an array. Each list has one atomic boolean controlling
// access for insertion and deletion.
// The hash rather than the index is used to find if two values are the same,
// this is quick but can cause two elements to be erroneously seen as belonging
// together.
//
template <typename ValueType>
class BaseHashMap {
 private:
  struct Node {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Node(const size_t hash) : hash_(hash), next_node_(nullptr){};

    Node(const Node& other)
        : hash_(other.hash_),
          value_(other.value_),
          next_node_(other.next_node_.load()) {}

    Node& operator=(const Node& other) {
      hash_ = other.hash_;
      value_ = other.value_;
      next_node_.store(other.next_node_.load());
      return *this;
    }

    size_t hash_;
    ValueType value_;
    std::atomic<Node*> next_node_;
  };

  class Bucket {
   public:
    Bucket() : atomic_ptr_(nullptr) { write_lock_flag_.clear(); }

    Bucket(const Bucket& other) : atomic_ptr_(other.atomic_ptr_.load()) {
      write_lock_flag_.clear();
    }

    Bucket& operator=(const Bucket& other) {
      atomic_ptr_.store(other.atomic_ptr_.load());
      // it is impossible to copy an atomic flags state
      write_lock_flag_.clear();

      return *this;
    }

    std::atomic<Node*>& atomicPtr() { return atomic_ptr_; }

    void unlock() { write_lock_flag_.clear(); }

    bool tryToLock() { return !write_lock_flag_.test_and_set(); }

    // spins until locked
    void lock() {
      while (write_lock_flag_.test_and_set())
        ;
    }

   private:
    std::atomic<Node*> atomic_ptr_;
    std::atomic_flag write_lock_flag_;
  };

 public:
  // should be an stl iterator, but I am lazy.
  // not thread safe
  class PseudoIterator {
   public:
    PseudoIterator(BaseHashMap<ValueType>::Node* node_ptr,
                   const size_t bucket_idx,
                   const std::shared_ptr<std::vector<Bucket>>& data_buckets)
        : node_ptr_(node_ptr),
          bucket_idx_(bucket_idx),
          data_buckets_(data_buckets) {}

    PseudoIterator& operator++() {
      node_ptr_ = node_ptr_->next_node_.load();
      while ((node_ptr_ == nullptr) &&
             ((++bucket_idx_) < data_buckets_->size())) {
        node_ptr_ = data_buckets_->at(bucket_idx_).atomicPtr().load();
      }
    }

    PseudoIterator(const PseudoIterator& other)
        : node_ptr_(other.node_ptr_),
          bucket_idx_(other.bucket_idx_),
          data_buckets_(other.data_buckets_) {}

    ValueType& operator*() const { return node_ptr_->value_; }

    bool operator==(const PseudoIterator& other) {
      return (bucket_idx_ == other.bucket_idx_) &&
             (node_ptr_ == other.node_ptr_);
    }

    bool operator!=(const PseudoIterator& other) { return !(*this == other); }

    PseudoIterator& operator=(const PseudoIterator& other) {
      node_ptr_ = other.node_ptr_;
      bucket_idx_ = other.bucket_idx_;
      data_buckets_ = other.data_buckets_;
      return *this;
    }

   private:
    BaseHashMap<ValueType>::Node* node_ptr_;
    size_t bucket_idx_;
    std::shared_ptr<std::vector<Bucket>> data_buckets_;
  };

  BaseHashMap() : BaseHashMap(1) {}

  BaseHashMap(size_t num_inital_buckets) : num_elements_(0) {
    // all logic is based on the assumption there is at least 1 bucket
    if (num_inital_buckets == 0) {
      ++num_inital_buckets;
    }

    data_buckets_ = std::make_shared<std::vector<Bucket>>(num_inital_buckets);
  }

  ~BaseHashMap() { clear(); }

  // thread safe
  // if nothing is found, value is unmodified
  bool tryFind(const AnyIndex& index, ValueType* value) const {
    size_t hash;
    Bucket* bucket_ptr;
    Node* node_ptr;
    std::atomic<Node*>* atomic_ptr_ptr;

    if (getAtomicPtrPtr(index, &hash, &bucket_ptr, &node_ptr,
                        &atomic_ptr_ptr)) {
      *value = node_ptr->value_;
      return true;
    } else {
      return false;
    }
  }

  // thread safe
  bool elementExists(const AnyIndex& index) const {
    ValueType value;
    return tryFind(index, &value);
  }

  // will create element if it does not exist, thread safe
  ValueType& findOrCreate(const AnyIndex& index, bool* was_created) {
    Node* node_ptr;

    *was_created = getOrCreateNode(index, &node_ptr);

    return node_ptr->value_;
  }

  // will create element if it does not exist, thread safe
  ValueType& findOrCreate(const AnyIndex& index) {
    Node* node_ptr;
    getOrCreateNode(index, &node_ptr);

    return node_ptr->value_;
  }

  // Only thread safe if no read operation occurs on the same index, or in other
  // words not even slightly thread safe.
  bool erase(const AnyIndex& index) {
    size_t hash = hashFunction(index);

    Bucket& bucket = data_buckets_->at(hash % data_buckets_->size());

    bucket.lock();

    Node* prev_node_ptr = nullptr;

    // iterate through the list in the bucket until at the correct hash or at
    // the end
    Node* node_ptr = bucket.atomicPtr().load();
    while (node_ptr != nullptr || node_ptr->hash_ != hash) {
      prev_node_ptr = node_ptr;
      node_ptr = node_ptr->next_node_.load();
    }

    if (node_ptr == nullptr) {
      // nothing to erase
      bucket.unlock();
      return false;
    }

    if (prev_node_ptr != nullptr) {
      prev_node_ptr->next_node_.store(node_ptr->next_node_.load());
    }

    delete node_ptr;
    --num_elements_;

    bucket.unlock();

    return true;
  }

  size_t size() const { return num_elements_.load(); }

  bool empty() const { return num_elements_.load() == 0; }

  // not thread safe
  void clear() {
    for (Bucket& bucket : *data_buckets_) {
      // leave locked as will soon be deleted
      bucket.lock();

      Node* node_ptr = bucket.atomicPtr().load();
      while (node_ptr != nullptr) {
        Node* next_node_ptr = node_ptr->next_node_.load();
        delete node_ptr;

        node_ptr = next_node_ptr;
      }
    }

    data_buckets_->clear();
    num_elements_ = 0;
    // always need at least one bucket
    data_buckets_->resize(1);
  }

  PseudoIterator begin() const {
    size_t bucket_idx = 0;
    Node* node_ptr = data_buckets_->at(bucket_idx).atomicPtr().load();

    while ((node_ptr == nullptr) && ((++bucket_idx) < data_buckets_->size())) {
      node_ptr = data_buckets_->at(bucket_idx).atomicPtr().load();
    }
    return PseudoIterator(node_ptr, bucket_idx, data_buckets_);
  }

  PseudoIterator end() const {
    return PseudoIterator(nullptr, data_buckets_->size(), nullptr);
  }

  // rehashes if the average elements per bucket is over 0.5
  // the rehash brings this down to 0.25
  // super not thread safe, will almost certainly cause memory leaks and
  // segfaults if any other operation happens while it is running
  void rehash(const size_t min_size = 1) {
    static constexpr size_t max_inv_load_factor = 2;
    static constexpr size_t inv_target_load_factor = 4;

    // check if rehash needed
    if ((max_inv_load_factor * num_elements_) > data_buckets_->size()) {
      std::shared_ptr<std::vector<Bucket>> new_buckets =
          std::make_shared<std::vector<Bucket>>(
              std::max(min_size, inv_target_load_factor * num_elements_));

      std::cerr << "new_size " << new_buckets->size() << std::endl;

      // loop through current buckets
      for (Bucket& bucket : *data_buckets_) {
        Node* node_ptr = bucket.atomicPtr().load();

        // loop through list in each bucket
        while (node_ptr != nullptr) {
          // make a copy of the next_node pointer then destroy it
          Node* next_node_ptr = node_ptr->next_node_.load();
          node_ptr->next_node_.store(nullptr);

          // find matching bucket in new bucket vector
          std::atomic<Node*>* new_atomic_ptr_ptr =
              &(new_buckets->at(node_ptr->hash_ % new_buckets->size())
                    .atomicPtr());
          Node* new_node_ptr = new_atomic_ptr_ptr->load();

          // move to end of bucket
          while (new_node_ptr != nullptr) {
            new_atomic_ptr_ptr = &(new_node_ptr->next_node_);
            new_node_ptr = new_atomic_ptr_ptr->load();
          }

          // insert data
          new_atomic_ptr_ptr->store(node_ptr);

          node_ptr = next_node_ptr;
        }
      }

      data_buckets_ = new_buckets;
    }
  }

 private:
  // thread safe, though it might not return an element that was inserted
  // after it was called
  // and yeah the pointers go a little nuts here
  bool getAtomicPtrPtr(const AnyIndex& index, size_t* hash,
                       Bucket** bucket_ptr_ptr, Node** node_ptr_ptr,
                       std::atomic<Node*>** atomic_ptr_ptr_ptr) const {
    *hash = hashFunction(index);
    *bucket_ptr_ptr = &(data_buckets_->at(*hash % data_buckets_->size()));
    *atomic_ptr_ptr_ptr = &((*bucket_ptr_ptr)->atomicPtr());
    // iterate through the list in the bucket until at the correct hash or at
    // the end
    *node_ptr_ptr = (*atomic_ptr_ptr_ptr)->load();
    while ((*node_ptr_ptr != nullptr) && ((*node_ptr_ptr)->hash_ != *hash)) {
      // the pointeryist line I have ever written
      *atomic_ptr_ptr_ptr = &((*node_ptr_ptr)->next_node_);
      *node_ptr_ptr = (*atomic_ptr_ptr_ptr)->load();
    }

    return *node_ptr_ptr != nullptr;
  }

  // if the data does not exist it will be created
  // if data is created returns true, otherwise false
  // thread safe with one caveat, see autoRehash()
  bool getOrCreateNode(const AnyIndex& index, Node** node_ptr_ptr) {
    size_t hash;
    Bucket* bucket_ptr;
    std::atomic<Node*>* atomic_ptr_ptr;
    if (getAtomicPtrPtr(index, &hash, &bucket_ptr, node_ptr_ptr,
                        &atomic_ptr_ptr)) {
      return false;
    } else {
      // the data does not exist so we are going to create and add it

      // get exclusive write access to list
      bucket_ptr->lock();

      // reload atomic to make sure its still unallocated
      *node_ptr_ptr = atomic_ptr_ptr->load();
      if (*node_ptr_ptr != nullptr) {
        // someone else created it
        bucket_ptr->unlock();
        return false;
      }

      atomic_ptr_ptr->store(new Node(hash));
      *node_ptr_ptr = atomic_ptr_ptr->load();
      ++num_elements_;

      // this destroys all thread safety (without adding expensive locks), and
      // so is a no-op on the concurrent version
      autoRehash();

      bucket_ptr->unlock();

      return true;
    }
  }

  void autoRehash(){/*rehash();*/};

  const size_t static inline hashFunction(const BlockIndex& index) {
    static constexpr size_t prime1 = 73856093;
    static constexpr size_t prime2 = 19349663;
    static constexpr size_t prime3 = 83492791;

    return (static_cast<size_t>(index.x()) * prime1 ^ index.y() * prime2 ^
            index.z() * prime3);
  }

  std::atomic<size_t> num_elements_;
  std::shared_ptr<std::vector<Bucket>> data_buckets_;
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

template <typename ValueType>
struct OldBlockHashMapType {
  typedef std::unordered_map<
      BlockIndex, ValueType, BlockIndexHash, std::equal_to<BlockIndex>,
      Eigen::aligned_allocator<std::pair<const BlockIndex, ValueType>>>
      type;
};

template <typename ValueType>
struct BlockHashMapType {
  typedef BaseHashMap<ValueType> type;
};

typedef std::unordered_set<AnyIndex, BlockIndexHash, std::equal_to<AnyIndex>,
                           Eigen::aligned_allocator<AnyIndex>>
    IndexSet;

typedef typename OldBlockHashMapType<IndexVector>::type HierarchicalIndexMap;

// typedef typename HierarchicalIndexMap::value_type HierarchicalIndex;

}  // namespace voxblox

#endif  // VOXBLOX_BLOCK_HASH_H_
