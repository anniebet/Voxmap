#ifndef VOXBLOX_AUTOENCODER_ENCODER_H_
#define VOXBLOX_AUTOENCODER_ENCODER_H_

#include <fstream>

namespace voxblox {

class Encoder {
 private:
  size_t block_elements_;
  std::vector<size_t> hidden_units_;

  double max_weight_;
  double truncation_distance_;

  std::vector<Eigen::MatrixXf> W_, W_prime_, b_, b_prime_;

  void infoLoader(const std::string& folder_path,
                  const std::string& file_name) {
    std::string path = folder_path + '/' + file_name;
    std::ifstream file(path, std::ios::in | std::ios::binary);

    bool first_el = true;
    std::cerr << file_name << std::endl;
    if (file.is_open()) {
      while (true) {
        int32_t value;
        file.read((char*)&value, sizeof(uint32_t));
        if (file.eof()) {
          break;
        }
        if (first_el) {
          first_el = false;
          block_elements_ = value;
        } else {
          hidden_units_.push_back(value);
        }
        std::cerr << " " << value << std::endl;
      }
      file.close();
    } else {
      ROS_ERROR_STREAM("Could not load " << path);
    }
  }

  void loader(const std::string& folder_path, const std::string& file_name,
              size_t height, size_t width, Eigen::MatrixXf* mat) {
    std::string path = folder_path + '/' + file_name;
    std::ifstream file(path, std::ios::in | std::ios::binary);

    mat->resize(height, width);

    std::cerr << file_name << std::endl;
    if (file.is_open()) {
      for (size_t i = 0; i < height; ++i) {
        for (size_t j = 0; j < width; ++j) {
          float value;
          file.read((char*)&value, sizeof(float));
          (*mat)(i, j) = value;
          // std::cerr << i << " " << j << " " << value << std::endl;
        }
      }
      file.close();
    } else {
      ROS_ERROR_STREAM("Could not load " << path);
    }
  }

  void blockValuesToVector(const Block<TsdfVoxel>& block,
                           Eigen::VectorXf* vec) {
    for (size_t i = 0; i < block_elements_; ++i) {
      TsdfVoxel voxel = block.getVoxelByLinearIndex(i);
      (*vec)(i) = voxel.distance / ((2 * truncation_distance_)) + 0.5;
      //(*vec)(2 * i + 1) = voxel.weight / max_weight_;
    }
  }

  void vectorToBlockValues(const Eigen::VectorXf& vec,
                           Block<TsdfVoxel>::Ptr block) {
    for (size_t i = 0; i < block_elements_; ++i) {
      // std::cerr << block->getVoxelByLinearIndex(i).distance << " " <<
      // 2*truncation_distance_*(vec(2 * i) - 0.5) << " " <<
      // block->getVoxelByLinearIndex(i).weight << " " << max_weight_*vec(2 * i
      // + 1) << std::endl;

      float distance = 2 * truncation_distance_ * (vec(i) - 0.5);
      if (distance > truncation_distance_) {
        distance = truncation_distance_;
      } else if (distance < -truncation_distance_) {
        distance = -truncation_distance_;
      }

      block->getVoxelByLinearIndex(i).distance = distance;
      // block->getVoxelByLinearIndex(i).weight = max_weight_ * vec(2 * i + 1);
    }
  }

  static float sigmoid(float x) { return 1 / (1 + std::exp(-x)); }
  // static float sigmoid(float x) { return x / (1 + std::fabs(x)); }

  void fullToHidden(const Eigen::VectorXf& full, Eigen::VectorXf* hidden) {
    // for(size_t i = 0; i < full.size(); ++i)
    //  std::cerr << (full)[i] << std::endl;
    Eigen::MatrixXf temp = full.transpose();
    for (size_t i = 0; i < hidden_units_.size(); ++i) {
      temp = temp * W_[i] + b_[i];
      temp = temp.unaryExpr(&Encoder::sigmoid);
    }

    *hidden = temp.transpose();
  }

  void hiddenToFull(const Eigen::VectorXf& hidden, Eigen::VectorXf* full) {
    Eigen::MatrixXf temp = hidden.transpose();
    for (int i = hidden_units_.size()-1; i >= 0; --i) {
      temp = temp * W_prime_[i] + b_prime_[i];
      temp = temp.unaryExpr(&Encoder::sigmoid);
    }

    *full = temp.transpose();
  }

 public:
  Encoder(const std::string& folder_path, double max_weight,
          double truncation_distance)
      : max_weight_(max_weight), truncation_distance_(truncation_distance) {
    infoLoader(folder_path, "info");

    W_.resize(hidden_units_.size());
    b_.resize(hidden_units_.size());
    W_prime_.resize(hidden_units_.size());
    b_prime_.resize(hidden_units_.size());

    for (size_t i = 0; i < hidden_units_.size(); ++i) {
      size_t elements;
      if (i == 0) {
        elements = block_elements_;
      } else {
        elements = hidden_units_[i - 1];
      }

      std::cerr << elements << " " << hidden_units_[i] << std::endl;

      loader(folder_path, "W_" + std::to_string(i), elements, hidden_units_[i],
             &(W_[i]));
      loader(folder_path, "b_" + std::to_string(i), 1, hidden_units_[i],
             &(b_[i]));

      loader(folder_path, "W_prime_" + std::to_string(i), hidden_units_[i],
             elements, &(W_prime_[i]));
      loader(folder_path, "b_prime_" + std::to_string(i), 1, elements,
             &(b_prime_[i]));
    }
  }

  void denoiseBlock(Block<TsdfVoxel>::Ptr block) {
    Eigen::VectorXf full_vec(block_elements_);
    Eigen::VectorXf full_vec2(block_elements_);
    Eigen::VectorXf hidden_vec(hidden_units_.back());

    blockValuesToVector(*block, &full_vec);
    fullToHidden(full_vec, &hidden_vec);
    hiddenToFull(hidden_vec, &full_vec2);
    vectorToBlockValues(full_vec2, block);

    /*for (size_t i = 0; i < 512; ++i) {
      std::cerr << full_vec[i] << " " << full_vec2[i] << std::endl;
    }

    std::cerr << "hidden" << std::endl;

    for (size_t i = 0; i < 16; ++i) {
      std::cerr << hidden_vec[i] << std::endl;
    }*/
  }

  void denoiseLayer(Layer<TsdfVoxel>* layer) {
    BlockIndexList blocks;
    layer->getAllAllocatedBlocks(&blocks);

    for (AnyIndex block_idx : blocks) {
      denoiseBlock(layer->getBlockPtrByIndex(block_idx));
    }
  }

  void writeLayer(const Layer<TsdfVoxel>& layer, const std::string& file_path) {
    std::ofstream file(file_path, std::ios::out | std::ios::binary);
    if (file.is_open()) {
      BlockIndexList blocks;
      layer.getAllAllocatedBlocks(&blocks);

      unsigned int num_blocks = blocks.size();
      unsigned int block_info = 2 * layer.voxels_per_side() *
                                layer.voxels_per_side() *
                                layer.voxels_per_side();

      file.write((char*)&num_blocks, sizeof(unsigned int));
      file.write((char*)&block_info, sizeof(unsigned int));

      for (AnyIndex block_idx : blocks) {
        for (size_t i = 0; i < (block_info / 2); ++i) {
          float value = layer.getBlockByIndex(block_idx)
                            .getVoxelByLinearIndex(i)
                            .distance;
          file.write((char*)&value, sizeof(float));
          value =
              layer.getBlockByIndex(block_idx).getVoxelByLinearIndex(i).weight;
          file.write((char*)&value, sizeof(float));
        }
      }
      file.close();
    }
  }
};
}

#endif