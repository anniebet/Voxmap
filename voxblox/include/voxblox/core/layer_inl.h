#ifndef VOXBLOX_CORE_LAYER_INL_H_
#define VOXBLOX_CORE_LAYER_INL_H_

#include <fstream>  // NOLINT
#include <string>
#include <utility>

#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/message_lite.h>

#include "./Block.pb.h"
#include "./Layer.pb.h"
#include "voxblox/core/block.h"
#include "voxblox/core/voxel.h"
#include "voxblox/utils/protobuf_utils.h"

namespace voxblox {

template <typename VoxelType>
Layer<VoxelType>::Layer(const LayerProto& proto)
    : voxel_size_(proto.voxel_size()),
      voxels_per_side_(proto.voxels_per_side()) {
  CHECK_EQ(getType().compare(proto.type()), 0)
      << "Incorrect voxel type, proto type: " << proto.type()
      << " layer type: " << getType();

  // Derived config parameter.
  CHECK_GT(proto.voxel_size(), 0.0);
  block_size_ = voxel_size_ * voxels_per_side_;
  CHECK_GT(block_size_, 0.0);
  block_size_inv_ = 1.0 / block_size_;
  CHECK_GT(proto.voxels_per_side(), 0u);
  voxels_per_side_inv_ = 1.0f / static_cast<FloatingPoint>(voxels_per_side_);
}

template <typename VoxelType>
void Layer<VoxelType>::getProto(LayerProto* proto) const {
  CHECK_NOTNULL(proto);

  CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  proto->set_voxel_size(voxel_size_);
  proto->set_voxels_per_side(voxels_per_side_);
  proto->set_type(getType());
}

template <typename VoxelType>
Layer<VoxelType>::Layer(const Layer& other) {
  voxel_size_ = other.voxel_size_;
  voxels_per_side_ = other.voxels_per_side_;
  block_size_ = other.block_size_;
  block_size_inv_ = other.block_size_inv_;

  for (const typename BlockType::Ptr& block_ptr : other.block_map_) {
    typename BlockType::Ptr new_block =
        allocateBlockPtrByIndex(block_ptr->block_index());

    for (size_t linear_idx = 0u; linear_idx < block_ptr->num_voxels();
         ++linear_idx) {
      const VoxelType& voxel = block_ptr->getVoxelByLinearIndex(linear_idx);
      VoxelType& new_voxel = new_block->getVoxelByLinearIndex(linear_idx);
      new_voxel = voxel;
    }
  }
}

template <typename VoxelType>
bool Layer<VoxelType>::saveToFile(const std::string& file_path) const {
  //  constexpr bool kIncludeAllBlocks = true;
  //  return saveSubsetToFile(file_path, BlockIndexList(), kIncludeAllBlocks);
}

template <typename VoxelType>
bool Layer<VoxelType>::saveSubsetToFile(const std::string& file_path,
                                        BlockIndexList blocks_to_include,
                                        bool include_all_blocks) const {
  /*CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  CHECK(!file_path.empty());
  std::fstream outfile;
  outfile.open(file_path, std::fstream::out | std::fstream::binary);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }

  // Only serialize the blocks if there are any.
  // Count the number of blocks that need to be serialized.
  size_t num_blocks_to_write = 0u;
  if ((include_all_blocks && !block_map_.empty()) ||
      !blocks_to_include.empty()) {
    for (const BlockMapPair& pair : block_map_) {
      bool write_block_to_file = include_all_blocks;

      if (!write_block_to_file) {
        BlockIndexList::const_iterator it = std::find(
            blocks_to_include.begin(), blocks_to_include.end(), pair.first);
        if (it != blocks_to_include.end()) {
          ++num_blocks_to_write;
        }
      } else {
        ++num_blocks_to_write;
      }
    }
  }
  if (include_all_blocks) {
    CHECK_EQ(num_blocks_to_write, block_map_.size());
  } else {
    CHECK_LE(num_blocks_to_write, block_map_.size());
    CHECK_LE(num_blocks_to_write, blocks_to_include.size());
  }

  // Write the total number of messages to the beginning of this file.
  // One layer header and then all the block maps
  const uint32_t num_messages = 1u + num_blocks_to_write;
  if (!utils::writeProtoMsgCountToStream(num_messages, &outfile)) {
    LOG(ERROR) << "Could not write message number to file.";
    outfile.close();
    return false;
  }

  // Write out the layer header.
  LayerProto proto_layer;
  getProto(&proto_layer);
  if (!utils::writeProtoMsgToStream(proto_layer, &outfile)) {
    LOG(ERROR) << "Could not write layer header message.";
    outfile.close();
    return false;
  }

  // Serialize blocks.
  for (const BlockMapPair& pair : block_map_) {
    bool write_block_to_file = include_all_blocks;
    if (!write_block_to_file) {
      BlockIndexList::const_iterator it = std::find(
          blocks_to_include.begin(), blocks_to_include.end(), pair.first);
      if (it != blocks_to_include.end()) {
        write_block_to_file = true;
      }
    }
    if (write_block_to_file) {
      BlockProto block_proto;
      pair.second->getProto(&block_proto);

      if (!utils::writeProtoMsgToStream(block_proto, &outfile)) {
        LOG(ERROR) << "Could not write block message.";
        outfile.close();
        return false;
      }
    }
  }
  outfile.close();*/
  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::addBlockFromProto(const BlockProto& block_proto,
                                         BlockMergingStrategy strategy) {
  CHECK_NE(getType().compare(voxel_types::kNotSerializable), 0)
      << "The voxel type of this layer is not serializable!";

  if (isCompatible(block_proto)) {
    typename BlockType::Ptr block_ptr(new BlockType(block_proto));
    const BlockIndex block_index =
        getGridIndexFromOriginPoint(block_ptr->origin(), block_size_inv_);

    switch (strategy) {
      case BlockMergingStrategy::kProhibit: {
        bool already_existed;
        block_map_.findOrCreate(block_index, &already_existed) = block_ptr;

        CHECK_EQ(already_existed, false) << "Block collision at index: "
                                         << block_index;
        break;
      }
      case BlockMergingStrategy::kReplace: {
        block_map_.findOrCreate(block_index) = block_ptr;
        break;
      }
      case BlockMergingStrategy::kDiscard: {
        bool already_existed;
        typename BlockType::Ptr& map_block_ptr =
            block_map_.findOrCreate(block_index, &already_existed);
        if (!already_existed) {
          map_block_ptr = block_ptr;
        }

        break;
      }
      case BlockMergingStrategy::kMerge: {
        bool already_existed;
        typename BlockType::Ptr& map_block_ptr =
            block_map_.findOrCreate(block_index, &already_existed);
        if (already_existed) {
          map_block_ptr->mergeBlock(*block_ptr);
        } else {
          map_block_ptr = block_ptr;
        }

        break;
      }
      default:
        LOG(FATAL) << "Unknown BlockMergingStrategy: "
                   << static_cast<int>(strategy);
        return false;
    }
    // Mark that this block has been updated.
    block_ptr->updated() = true;
  } else {
    LOG(ERROR)
        << "The blocks from this protobuf are not compatible with this layer!";
    return false;
  }
  return true;
}

template <typename VoxelType>
bool Layer<VoxelType>::isCompatible(const LayerProto& layer_proto) const {
  bool compatible = true;
  compatible &= (layer_proto.voxel_size() == voxel_size_);
  compatible &= (layer_proto.voxels_per_side() == voxels_per_side_);
  compatible &= (getType().compare(layer_proto.type()) == 0);
  return compatible;
}

template <typename VoxelType>
bool Layer<VoxelType>::isCompatible(const BlockProto& block_proto) const {
  bool compatible = true;
  compatible &= (block_proto.voxel_size() == voxel_size_);
  compatible &=
      (block_proto.voxels_per_side() == static_cast<int>(voxels_per_side_));
  return compatible;
}

template <typename VoxelType>
size_t Layer<VoxelType>::getMemorySize() const {
  size_t size = 0u;

  // Calculate size of members
  size += sizeof(voxel_size_);
  size += sizeof(voxels_per_side_);
  size += sizeof(block_size_);
  size += sizeof(block_size_inv_);

  // Calculate size of blocks
  size_t num_blocks = getNumberOfAllocatedBlocks();
  if (num_blocks > 0u) {
    typename Block<VoxelType>::Ptr block = *(block_map_.begin());
    size += num_blocks * block->getMemorySize();
  }
  return size;
}

template <typename VoxelType>
std::string Layer<VoxelType>::getType() const {
  return getVoxelType<VoxelType>();
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_LAYER_INL_H_
