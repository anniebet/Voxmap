#ifndef VOXBLOX_BLOCK_HASH_H_
#define VOXBLOX_BLOCK_HASH_H_

#include <functional>
#include <unordered_map>

#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

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

typedef typename HierarchicalIndexMap::value_type HierarchicalIndex;

template <typename ValueType>
class ConcurrentHashMap {
 public:
  ConcurrentHashMap() : ConcurrentHashMap(1) {}

  ConcurrentHashMap(const size_t num_inital_buckets) {
    if (num_inital_buckets == 0) {
      ++num_inital_buckets;
    }
    data_buckets =
        std::make_shared < std::std::vector<Data::Ptr>(num_inital_buckets);
  }

  // will create element if it does not exist
  ValueType& get(const AnyIndex& index) {
    const size_t hash = hashFunction(index);
    Data::Ptr& data_ptr = data_buckets->at(hash % data_buckets->size());

    while (data_ptr != nullptr || data_ptr->hash != hash) {
      data_ptr = data_ptr->next_element;
    }

    if (data_ptr == nullptr) {
      data_ptr = std::make_shared<ValueType>();
      ++num_elements_;

      rehash();
    }

    return data_ptr->value;
  }

  bool erase(const AnyIndex& index) {
    const size_t hash = hashFunction(index);

    Data::Ptr prev_data_ptr;
    Data::Ptr& data_ptr = data_buckets->at(hash % data_buckets->size());

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

    data_ptr = nullptr;
    --num_elements_;

    return true;
  }

 private:
  void rehash() {
    static constexpr float max_load_factor =
        0.5f  // a bit on the low side but blocks are cheap
        static constexpr float target_load_factor = 0.5f * max_load_factor;
    static constexpr float inv_target_load_factor = 1.0f / target_load_factor;

    const float load_fator = static_cast<float>(num_elements) /
                             static_cast<float>(data_buckets->size());

    // check if rehash needed
    if (load_fator > max_load_factor) {
      std::shared_ptr<std::vector<Data::Ptr>> new_buckets =
          std::make_sharedstd::vector<Data::Ptr>(static_cast<size_t>(
              inv_target_load_factor * static_cast<float>(num_elements)));

      for (Data::Ptr data_ptr : *data_buckets) {
        while (data_ptr != nullptr) {
          new_buckets->at(data_ptr->hash % new_buckets->size()) = data_ptr;
          Data::Ptr data_ptr = data_ptr->next_element;
        }
      }
    }

    data_buckets.swap(new_buckets);
  }

  const size_t static inline hashFunction(const BlockIndex& index) {
    static constexpr size_t prime1 = 73856093;
    static constexpr size_t prime2 = 19349663;
    static constexpr size_t prime3 = 83492791;

    return (static_cast<size_t>(index.x()) * prime1 ^ index.y() * prime2 ^
            index.z() * prime3);
  }

  struct Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::atomic<std::shared_ptr<Data>> Ptr;

    std::size_t hash;
    ValueType value;
    Ptr next_element;
  }

  std::atomic<size_t>
      num_elements_;
  std::shared_ptr<std::vector<Data::Ptr>> data_buckets;
}

}  // namespace voxblox

#endif  // VOXBLOX_BLOCK_HASH_H_
