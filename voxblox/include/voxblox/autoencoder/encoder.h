#ifndef VOXBLOX_AUTOENCODER_ENCODER_H_
#define VOXBLOX_AUTOENCODER_ENCODER_H_

#include <fstream>

namespace voxblox {

class Encoder {
 private:
  size_t block_elements_;
  size_t hidden_units_;

  Eigen::MatrixXf W_, W_prime_, b_, b_prime_;

  void loader(const std::string& folder_path, const std::string& file_name,
              size_t height, size_t width, Eigen::MatrixXf* mat) {
    std::string path = folder_path + '/' + file_name;
    std::ifstream file(path, std::ios::in | std::ios::binary);

    mat->resize(height,width);

    if (file.is_open()) {
      for (size_t i = 0; i < height; ++i) {
        for (size_t j = 0; j < width; ++j) {
          float value;
          file.read((char*)&value, sizeof(float));
          (*mat)(i, j) = value;
        }
      }
      file.close();
    }
  }

  void block_values_to_vector(const Block<TsdfVoxel>& block,
                              Eigen::VectorXf* vec) {
    for (size_t i = 0; i < block_elements_ / 2; ++i) {
      TsdfVoxel voxel = block.getVoxelByLinearIndex(i);
      (*vec)(2 * i) = voxel.distance;
      (*vec)(2 * i + 1) = voxel.weight;
    }
  }

  void vector_to_block_values(const Eigen::VectorXf& vec,
                              Block<TsdfVoxel>::Ptr block) {
    for (size_t i = 0; i < block_elements_ / 2; ++i) {
      block->getVoxelByLinearIndex(i).distance = vec(2 * i);
      block->getVoxelByLinearIndex(i).weight = vec(2 * i + 1);
    }
  }

  void full_to_hidden(const Eigen::VectorXf& full, Eigen::VectorXf* hidden) {
    *hidden = W_ * full + b_;
  }

  void hidden_to_full(const Eigen::VectorXf& hidden, Eigen::VectorXf* full) {
    *full = W_prime_ * hidden + b_prime_;
  }

 public:
  Encoder(const std::string& folder_path, size_t voxels_per_side,
          size_t hidden_units)
      : block_elements_(2 * voxels_per_side * voxels_per_side *
                        voxels_per_side),
        hidden_units_(hidden_units) {
    loader(folder_path, "W", hidden_units_, block_elements_, &W_);
    loader(folder_path, "b", hidden_units_, 1, &b_);
    loader(folder_path, "W_prime", block_elements_, hidden_units_, &W_prime_);
    loader(folder_path, "b_prime", block_elements_, 1, &b_prime_);
  }

  void denoise_block(Block<TsdfVoxel>::Ptr block) {
    Eigen::VectorXf full_vec(block_elements_);
    Eigen::VectorXf hidden_vec(hidden_units_);

    block_values_to_vector(*block, &full_vec);
    full_to_hidden(full_vec, &hidden_vec);
    hidden_to_full(hidden_vec, &full_vec);
    vector_to_block_values(full_vec, block);
  }

  void denoise_layer(Layer<TsdfVoxel>* layer) {
    BlockIndexList blocks;
    layer->getAllAllocatedBlocks(&blocks);

    for (AnyIndex block_idx : blocks) {
      denoise_block(layer->getBlockPtrByIndex(block_idx));
    }
  }
};
}

#endif