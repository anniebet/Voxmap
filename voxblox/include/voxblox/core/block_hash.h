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
// Erasing elements is NOT thread safe.
// Automatic rehashing, NULLIFIES all thread safety guarantees.
// Manual rehashing can be called with rehash(), if the table does not need to
// expand it is a cheap operation but is NOT thread safe.
// Modifying an existing element is only thread safe if ValueType is atomic.
// Multiple threads can read existing elements and create new elements
// simultaneously without issues.
// Under the hood all data is stored using atomic pointers that are organized
// into linked lists in an array.
// The hash rather than the index is used to find if two values are the same,
// this is quick but can cause two elements to be erroneously seen as belonging
// together.
// Reading a value is guaranteed to get it if it exists, However if its value
// has recently been changed by another thread, the read may return the old
// value. This can happen even if the writing thread has made the change and
// exited the function before the read begins.
// the size() function may be slightly off under some thread conditions.
//
// The map is at its best when you never need to erase or modify values. An
// example of this is storing pointers to blocks for a TSDF integrator.
template <typename ValueType>
class ConcurrentHashMap {
 public:
  // should be an stl iterator, but I am lazy
  class Index {
   public:
    Index& operator++() {
      data_ptr_ = data_ptr_->next_element.load(std::memory_order_relaxed);
      while ((data_ptr_ == nullptr) && (data_buckets_.size() > bucket_idx_)) {
        data_ptr_ =
            data_buckets_[bucket_idx_++].load(std::memory_order_relaxed);
      }
    }

    ValueType& operator*() { return data_ptr_->value; }

    bool operator==(const Index& other) {
      return (bucket_idx_ == other.bucket_idx_) &&
             (data_ptr_ == other.data_ptr_);
    }
  }

  private : Data* data_ptr_;
  size_t bucket_idx_;
  std::shared_ptr<std::vector<std::atomic<Data*>>> data_buckets_;
}

ConcurrentHashMap()
    : ConcurrentHashMap(1) {
}

ConcurrentHashMap(size_t num_inital_buckets) {
  // all logic is based on the assumption there is at least 1 bucket
  if (num_inital_buckets == 0) {
    ++num_inital_buckets;
  }

  data_buckets_ =
      std::make_shared<std::vector<std::atomic<Data*>>>(num_inital_buckets);
}

~ConcurrentHashMap() { clear(); }

// thread safe, value may be old
bool tryFind(const AnyIndex& index, ValueType* value) const {
  size_t hash;
  std::atomic<Data*>* atomic_ptr;
  Data* data_ptr;

  if (getDataPtr(index, &hash, atomic_ptr, &data_ptr)) {
    value = &(data_ptr->value);
    return true;
  } else {
    return false;
  }
}

// thread safe, value may be old
bool tryFind(const AnyIndex& index, ValueType& value) const {
  return tryFind(index, &value);
}

// will create element if it does not exist, only thread safe is ValueType is
// atomic and automatic rehashing is disabled
void updateOrInsert(const AnyIndex& index, const ValueType& value) {
  Data* data_ptr;

  getOrCreateDataPtr(index, &data_ptr);

  data_ptr->value = value;
}

// will create element if it does not exist, thread safe, value may be old
ValueType& findOrCreate(const AnyIndex& index) {
  Data* data_ptr;

  getOrCreateDataPtr(index, &data_ptr);

  return data_ptr->value;
}

// thread safe
bool elementExists(const AnyIndex& index) const {
  size_t hash;
  std::atomic<Data*>* atomic_ptr;
  Data* data_ptr;

  return getDataPtr(index, &hash, atomic_ptr, &data_ptr);
}

// currently not thread safe
// could be made to be if there is a need
bool erase(const AnyIndex& index) {
  const size_t hash = hashFunction(index);

  Data* prev_data_ptr;
  Data* data_ptr = data_buckets_->at(hash % data_buckets_->size())
                       .load(std::memory_order_relaxed);

  while (data_ptr != nullptr || data_ptr->hash != hash) {
    prev_data_ptr = data_ptr;
    data_ptr = data_ptr->next_element.load(std::memory_order_relaxed);
  }

  if (data_ptr == nullptr) {
    // nothing to erase
    return false;
  }

  if (prev_data_ptr != nullptr) {
    prev_data_ptr->next_element.store(data_ptr->next_element,
                                      std::memory_order_relaxed);
  }

  delete data_ptr;
  --num_elements_;

  return true;
}

// count may lag actual
size_t size() const { return num_elements_; }

// not thread safe
void clear() {
  Index index = begin();
  while (index != end()) {
    Data* data_ptr = index.data_ptr_;
    ++index;
    delete data_ptr;
  }

  num_elements_ = 0;
  data_buckets_->clear();
  // we always need at least 1 bucket
  data_buckets_->resize(1);
}

Index begin() {
  Index index;
  index.data_buckets_ = data_buckets_;

  while (index.data_ptr_ == nullptr) {
    data_ptr_ =
        data_buckets_[index.bucket_idx_++].load(std::memory_order_relaxed);
  }

  return index;
}

Index end() {
  Index index;
  index.bucket_idx_ = data_buckets_.size();

  // forward only iterator means end index does not need to know where the data
  // is

  return index;
}

private:
struct Data {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  size_t hash;
  ValueType value;
  std::atomic<Data*> next_element;
};

// thread safe, though the data can be outdated
bool getDataPtr(const AnyIndex& index, size_t* hash,
                std::atomic<Data*>* atomic_ptr, Data** data_ptr) const {
  *hash = hashFunction(index);

  // get atomic pointer to correct bucket
  atomic_ptr = &(data_buckets_->at(*hash % data_buckets_->size()));
  *data_ptr = atomic_ptr->load(std::memory_order_relaxed);

  // iterate through the list in the bucket until at the correct hash or at
  // the end
  while (*data_ptr != nullptr || (*data_ptr)->hash != *hash) {
    atomic_ptr = &((*data_ptr)->next_element);
    *data_ptr = atomic_ptr->load(std::memory_order_relaxed);
  }

  // return true if read succeeded
  return *data_ptr != nullptr;
}

// if the data does not exist it will be created, thread safe
void getOrCreateDataPtr(const AnyIndex& index, Data** data_ptr) {
  size_t hash;
  std::atomic<Data*>* atomic_ptr;

  if (!getDataPtr(index, &hash, atomic_ptr, data_ptr)) {
    // the data does not exist so we are going to create and add it
    *data_ptr = new Data;
    (*data_ptr)->hash = hash;

    Data* expected_ptr = nullptr;

    // we are going to repeatedly shove the data into the list until it sticks
    while (!atomic_ptr->compare_exchange_strong(expected_ptr, *data_ptr)) {
      // we were beaten to the insert, check if is the data we wanted that
      // was inserted
      if (expected_ptr->hash == hash) {
        // some other thread created the data we needed, use that one instead
        delete *data_ptr;
        *data_ptr = expected_ptr;
        break;
      }

      // try the insert again
      atomic_ptr = &(expected_ptr->next_element);
      expected_ptr = nullptr;
    }

    // if expected stayed null it was our thread that added the data
    if (expected_ptr == nullptr) {
      ++num_elements_;

      rehash();
    }
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
    std::shared_ptr<std::vector<std::atomic<Data*>>> new_buckets =
        std::make_shared<std::vector<std::atomic<Data*>>>(new_size);

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
std::shared_ptr<std::vector<std::atomic<Data*>>> data_buckets_;
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
