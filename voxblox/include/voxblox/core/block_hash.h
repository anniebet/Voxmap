#ifndef VOXBLOX_BLOCK_HASH_H_
#define VOXBLOX_BLOCK_HASH_H_

#include <atomic>
#include <functional>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

template <typename ValueType>
class ConcurrentHashMap {
 public:

  ConcurrentHashMap()
      : ConcurrentHashMap(1) {
  }

  ConcurrentHashMap(size_t num_inital_buckets) {
    // all logic is based on the assumption there is at least 1 bucket
    if (num_inital_buckets == 0) {
      ++num_inital_buckets;
    }

    data_buckets_ =
        std::make_shared<std::vector<typename Data::Ptr>>(num_inital_buckets);
  }

  ~ConcurrentHashMap() { clear(); }

  bool tryFind(const AnyIndex& index, ValueType* value) const {
    typename Data::Ptr& data_ptr = getDataPtr(index);

    if (data_ptr == nullptr) {
      return false;
    } else {
      value = &(data_ptr->value);
      return true;
    }
  }

  bool tryFind(const AnyIndex& index, ValueType& value) const {
    typename Data::Ptr& data_ptr = getDataPtr(index);

    if (data_ptr == nullptr) {
      return false;
    } else {
      value = data_ptr->value;
      return true;
    }
  }

  // will create element if it does not exist
  ValueType& findOrCreate(const AnyIndex& index) {
    typename Data::Ptr& data_ptr = getDataPtr(index);

    if (data_ptr == nullptr) {
      data_ptr = new Data;
      ++num_elements_;

      rehash();
    }

    return data_ptr->value;
  }

  bool elementExists(const AnyIndex& index) const {
    typename Data::Ptr& data_ptr = getDataPtr(index);
    return data_ptr != nullptr;
  }

  bool erase(const AnyIndex& index) {
    const size_t hash = hashFunction(index);

    typename Data::Ptr prev_data_ptr;
    typename Data::Ptr& data_ptr =
        data_buckets_->at(hash % data_buckets_->size());

    while (data_ptr != nullptr || data_ptr->hash != hash) {
      prev_data_ptr = data_ptr;
      data_ptr = data_ptr->next_element;
    }

    if (data_ptr == nullptr) {
      // nothing to erase
      return false;
    }

    if (prev_data_ptr != nullptr) {
      prev_data_ptr->next_element = data_ptr->next_element;
    }

    delete data_ptr;
    data_ptr = nullptr;
    --num_elements_;

    return true;
  }

  size_t size() const { return num_elements_; }

  void clear() {
    for (typename Data::Ptr data_ptr : *data_buckets_) {
      while (data_ptr != nullptr) {
        typename Data::Ptr next_data_ptr = data_ptr->next_element;
        delete data_ptr;
        data_ptr = next_data_ptr;
      }
    }

    num_elements_ = 0;
    data_buckets_->clear();
    // we always need at least 1 bucket
    data_buckets_->resize(1);
  }

  void getAllElements(std::vector<ValueType>* value_vector){

    value_vector->clear();
    value_vector->reserve(num_elements_);

    for(typename Data::Ptr data_ptr : data_buckets_ ){
      while (data_ptr != nullptr) {
        value_vector->push_back(data_ptr_->value);
        data_ptr = data_ptr->next_element;
      }
    }
  }

 private:
  struct Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename std::atomic<Data*> Ptr;

    std::size_t hash;
    ValueType value;
    Ptr next_element;
  };

  // will return nullptr if it is not in the list
  typename Data::Ptr& getDataPtr(const AnyIndex& index) const {
    const size_t hash = hashFunction(index);
    typename Data::Ptr& data_ptr =
        data_buckets_->at(hash % data_buckets_->size());

    while (data_ptr != nullptr || data_ptr->hash != hash) {
      data_ptr = data_ptr->next_element;
    }

    return data_ptr;
  }

  void rehash() {
    static constexpr float max_load_factor =
        0.5f;  // a bit on the low side but blocks are cheap
    static constexpr float target_load_factor = 0.5f * max_load_factor;
    static constexpr float inv_target_load_factor = 1.0f / target_load_factor;

    const float load_fator = static_cast<float>(num_elements_) /
                             static_cast<float>(data_buckets_->size());

    // check if rehash needed
    if (load_fator > max_load_factor) {
      std::shared_ptr<std::vector<typename Data::Ptr>> new_buckets =
          std::make_shared<std::vector<typename Data::Ptr>>(static_cast<size_t>(
              inv_target_load_factor * static_cast<float>(num_elements_)));

      for (typename Data::Ptr data_ptr : *data_buckets_) {
        while (data_ptr != nullptr) {
          new_buckets->at(data_ptr->hash % new_buckets->size()) = data_ptr;
          typename Data::Ptr data_ptr = data_ptr->next_element;
        }
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
  std::shared_ptr<std::vector<typename Data::Ptr>> data_buckets_;
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
