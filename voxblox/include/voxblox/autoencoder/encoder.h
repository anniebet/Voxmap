#ifndef VOXBLOX_AUTOENCODER_ENCODER_H_
#define VOXBLOX_AUTOENCODER_ENCODER_H_

namespace voxblox {

class Encoder {
 private:
  size_t block_elements_;
  std::vector<size_t> hidden_units_;

  double max_weight_;
  double truncation_distance_;

  std::vector<Eigen::MatrixXf> W_distance_, W_prime_distance_, b_distance_, b_prime_distance_;
  std::vector<Eigen::MatrixXf> W_weight_, W_prime_weight_, b_weight_, b_prime_weight_;
  std::vector<Eigen::MatrixXf> W_color_, W_prime_color_, b_color_, b_prime_color_;

  void infoLoader(const std::string& folder_path,
                  const std::string& file_name) {
    std::string path = folder_path + '/' + file_name;
    std::ifstream file(path, std::ios::in | std::ios::binary);

    hidden_units_.resize(0);

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
          //std::cerr << i << " " << j << " " << value << std::endl;
        }
      }
      file.close();
    } else {
      ROS_ERROR_STREAM("Could not load " << path);
    }
  }

  void blockValuesToVector(const Block<TsdfVoxel>& block,
                           Eigen::VectorXf* vec_distance, Eigen::VectorXf* vec_weight) {
    size_t num_voxels = block.voxels_per_side() * block.voxels_per_side() *
                 block.voxels_per_side();

    for (size_t i = 0; i < num_voxels; ++i) {
      TsdfVoxel voxel = block.getVoxelByLinearIndex(i);
      (*vec_distance)(i) = voxel.distance / ((2 * truncation_distance_)) + 0.5;
      (*vec_weight)(i) = voxel.weight / max_weight_;
    }
  }

  void vectorToBlockValues(const Eigen::VectorXf& vec_distance, const Eigen::VectorXf& vec_weight,
                           Block<TsdfVoxel>::Ptr block) {
    size_t num_voxels = block->voxels_per_side() * block->voxels_per_side() *
                 block->voxels_per_side();

    Eigen::VectorXf vec_distance_norm = vec_distance.cwiseMin(1.0).cwiseMax(-1.0);
    Eigen::VectorXf vec_weight_norm = vec_weight.cwiseMin(1.0).cwiseMax(-1.0);

    for (size_t i = 0; i < num_voxels; ++i) {
      float distance = 2 * truncation_distance_ * (vec_distance_norm[i] - 0.5);
      float weight = max_weight_ * vec_weight_norm[i];

      block->getVoxelByLinearIndex(i).distance = distance;
      block->getVoxelByLinearIndex(i).weight = weight;
    }
  }

  static float sigmoid(float x) { return 1 / (1 + std::exp(-x)); }
  // static float sigmoid(float x) { return x / (1 + std::fabs(x)); }

  void fullToHidden(const Eigen::VectorXf& full, Eigen::VectorXf* hidden, std::vector<Eigen::MatrixXf>& W, std::vector<Eigen::MatrixXf>& b) {
    // for(size_t i = 0; i < full.size(); ++i)
    //  std::cerr << (full)[i] << std::endl;
    Eigen::MatrixXf temp = full.transpose();
    for (size_t i = 0; i < hidden_units_.size(); ++i) {
      ROS_ERROR_STREAM(" " << temp.size() << " " << W[i].size() << " " << b[i].size());
      temp = temp * W[i] + b[i];
      ROS_ERROR_STREAM(" " << i);
      temp = temp.unaryExpr(&Encoder::sigmoid);
    }

    *hidden = temp.transpose();
  }

  void hiddenToFull(const Eigen::VectorXf& hidden, Eigen::VectorXf* full, std::vector<Eigen::MatrixXf>& W_prime, std::vector<Eigen::MatrixXf>& b_prime) {
    Eigen::MatrixXf temp = hidden.transpose();
    for (int i = hidden_units_.size() - 1; i >= 0; --i) {
      temp = temp * W_prime[i] + b_prime[i];
      temp = temp.unaryExpr(&Encoder::sigmoid);
    }

    *full = temp.transpose();
  }

  void setup_weights(const std::string& folder_path, std::vector<Eigen::MatrixXf>* W, std::vector<Eigen::MatrixXf>* W_prime, std::vector<Eigen::MatrixXf>* b, std::vector<Eigen::MatrixXf>* b_prime){
    infoLoader(folder_path, "info");

    W->resize(hidden_units_.size());
    b->resize(hidden_units_.size());
    W_prime->resize(hidden_units_.size());
    b_prime->resize(hidden_units_.size());

    for (size_t i = 0; i < hidden_units_.size(); ++i) {
      size_t elements;
      if (i == 0) {
        elements = block_elements_;
      } else {
        elements = hidden_units_[i - 1];
      }

      loader(folder_path, "W_" + std::to_string(i), elements, hidden_units_[i],
             &(W->at(i)));
      loader(folder_path, "b_" + std::to_string(i), 1, hidden_units_[i],
             &(b->at(i)));

      loader(folder_path, "W_prime_" + std::to_string(i), hidden_units_[i],
             elements, &(W_prime->at(i)));
      loader(folder_path, "b_prime_" + std::to_string(i), 1, elements,
             &(b_prime->at(i)));
    }
  }

 public:
  void setup(const std::string& folder_path, const TsdfIntegrator::Config& config) {

    max_weight_ = config.max_weight;
    truncation_distance_ = config.default_truncation_distance;

    setup_weights(folder_path + "/weight", &W_weight_, &W_prime_weight_, &b_weight_, &b_prime_weight_);
    setup_weights(folder_path + "/distance", &W_distance_, &W_prime_distance_, &b_distance_, &b_prime_distance_);
    //setup_weights(folder_path + "/color", &W_color_, &W_prime_color_, &b_color_, &b_prime_color_);
  }

  void denoiseBlock(Block<TsdfVoxel>::Ptr block) {
    Eigen::VectorXf full_weight_vec(block_elements_);
    Eigen::VectorXf full_distance_vec(block_elements_);
    
    Eigen::VectorXf full_weight_vec2(block_elements_);
    Eigen::VectorXf full_distance_vec2(block_elements_);

    Eigen::VectorXf hidden_weight_vec(hidden_units_.back());
    Eigen::VectorXf hidden_distance_vec(hidden_units_.back());

    blockValuesToVector(*block, &full_distance_vec, &full_weight_vec);

    fullToHidden(full_distance_vec, &hidden_distance_vec, W_distance_, b_distance_);
    hiddenToFull(hidden_distance_vec, &full_distance_vec2, W_prime_distance_, b_prime_distance_);

    fullToHidden(full_weight_vec, &hidden_weight_vec, W_weight_, b_weight_);
    hiddenToFull(hidden_weight_vec, &full_weight_vec2, W_prime_weight_, b_prime_weight_);
    vectorToBlockValues(full_distance_vec2, full_weight_vec2, block);
    
    for(size_t i = 0; i < full_distance_vec.size(); ++i){
      if(full_distance_vec[i] > 0.1){
        std::cerr << full_distance_vec[i] << " " << full_distance_vec2[i] << std::endl;
      }
    }
  }

  void denoiseLayer(Layer<TsdfVoxel>* layer) {
    BlockIndexList blocks;
    layer->getAllAllocatedBlocks(&blocks);

    for (AnyIndex block_idx : blocks) {
      denoiseBlock(layer->getBlockPtrByIndex(block_idx));
    }
  }
};
}

#endif