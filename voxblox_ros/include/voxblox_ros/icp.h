/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef VOXBLOX_ROS_ICP_H_
#define VOXBLOX_ROS_ICP_H_

#include <algorithm>

#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh.h>
#include <voxblox/mesh/mesh_layer.h>

namespace voxblox {

void getTransformFromCorrelation(const PointsMatrix &src_demean,
                                 const Point &src_center,
                                 const PointsMatrix &tgt_demean,
                                 const Point &tgt_center,
                                 Transformation *T_src_tgt) {
  CHECK_NOTNULL(T_src_tgt);
  CHECK(src_demean.cols() == tgt_demean.cols());

  // Assemble the correlation matrix H = source * target'
  Matrix3 H = src_demean * tgt_demean.transpose();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Matrix3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3 u = svd.matrixU();
  Matrix3 v = svd.matrixV();

  // Compute R = V * U'
  if (u.determinant() * v.determinant() < 0) {
    for (int x = 0; x < 3; ++x) v(x, 2) *= -1;
  }

  // Form transformation
  Matrix3 rotation_matrix = v * u.transpose();
  Point translation = tgt_center - (rotation_matrix * src_center);

  *T_src_tgt = Transformation(Rotation(rotation_matrix), translation);
}

void getTransformationFromMatchedPoints(const PointsMatrix &src,
                                        const PointsMatrix &tgt,
                                        const PointsMatrix &weights,
                                        Transformation *T_src_tgt) {
  CHECK_NOTNULL(T_src_tgt);
  CHECK(src.cols() == tgt.cols());
  CHECK(src.cols() == weights.cols());

  // normalize weights
  PointsMatrix weights_norm =
      weights.array().colwise() / weights.array().rowwise().sum();

  // find and remove mean
  Point src_center = (src.array() * weights_norm.array()).rowwise().sum();
  Point tgt_center = (tgt.array() * weights_norm.array()).rowwise().sum();

  PointsMatrix src_demean_weighted =
      (src.colwise() - src_center).array() * weights_norm.array();
  PointsMatrix tgt_demean_weighted =
      (tgt.colwise() - tgt_center).array() * weights_norm.array();

  // align
  getTransformFromCorrelation(src_demean_weighted, src_center,
                              tgt_demean_weighted, tgt_center, T_src_tgt);
}

void getTransformationFromMatchedPoints(const PointsMatrix &src,
                                        const PointsMatrix &tgt,
                                        Transformation *T_src_tgt) {
  PointsMatrix weights = PointsMatrix::Ones(3, src.cols());
  getTransformationFromMatchedPoints(src, tgt, weights, T_src_tgt);
}

void stepICP(Layer<TsdfVoxel> *tsdf_layer, const Pointcloud &points_C,
             const Transformation &T_G_C_in, Transformation *T_G_C_out) {
  CHECK_NOTNULL(tsdf_layer);
  CHECK_NOTNULL(T_G_C_out);

  Interpolator interpolator(tsdf_layer);

  int npts = points_C.size();
  PointsMatrix src(3, npts);
  PointsMatrix tgt(3, npts);
  PointsMatrix weights(3, npts);

  // build alignment matrices
  size_t idx = 0;
  for (IndexElement i = 0; i < 1000; ++i) {

    IndexElement rand_el = std::rand() % points_C.size();
    Point p_C = points_C[rand_el];

    Point p_G = T_G_C_in * p_C;

    FloatingPoint distance = 0.0;
    Point grad(0,0,0);
    if (interpolator.getDistance(p_G, &distance, true) && interpolator.getGradient(p_G, &grad, true)) {
      src.col(idx) = p_C;
      tgt.col(idx) = p_G - distance * grad;

      FloatingPoint weight = tsdf_layer->getBlockPtrByCoordinates(p_G)
                                 ->getVoxelByCoordinates(p_G)
                                 .weight;

      //std::cout << weight << std::endl;

      weights.col(idx) = Point(weight,weight,weight);
      ++idx;
    }
  }

  if(idx < 1){
    *T_G_C_out = T_G_C_in;
    return;
  }

  // resize
  src.conservativeResize(Eigen::NoChange, idx);
  tgt.conservativeResize(Eigen::NoChange, idx);
  weights.conservativeResize(Eigen::NoChange, idx);

  getTransformationFromMatchedPoints(src, tgt, weights, T_G_C_out);
}

void stepICP(Layer<TsdfVoxel> *tsdf_layer_A, Layer<TsdfVoxel> *tsdf_layer_B,
             const Transformation &T_A_B_in, Transformation *T_A_B_out) {
  CHECK_NOTNULL(tsdf_layer_A);
  CHECK_NOTNULL(tsdf_layer_B);
  CHECK_NOTNULL(T_A_B_out);

  Interpolator interpolator_A(tsdf_layer_A);
  Interpolator interpolator_B(tsdf_layer_B);

  int npts = tsdf_layer_A->getNumberOfAllocatedBlocks() *
             tsdf_layer_A->voxels_per_side() * tsdf_layer_A->voxels_per_side() *
             tsdf_layer_A->voxels_per_side();

  PointsMatrix src(3, npts);
  PointsMatrix tgt(3, npts);
  PointsMatrix weights(3, npts);

  BlockIndexList blocks_index_A;
  tsdf_layer_A->getAllAllocatedBlocks(&blocks_index_A);

  // build alignment matrices
  size_t idx = 0;
  for (BlockIndex index_A : blocks_index_A) {
    Block<TsdfVoxel>::Ptr block_A = tsdf_layer_A->getBlockPtrByIndex(index_A);

    for (size_t i = 0; i < block_A->num_voxels(); ++i) {
      Point p_A = block_A->computeCoordinatesFromLinearIndex(i);
      Point p_B = T_A_B_in * p_A;

      FloatingPoint distance_A, distance_B;
      Point grad_A, grad_B;
      if (interpolator_A.getDistance(p_A, &distance_A, true) &&
          interpolator_A.getGradient(p_A, &grad_A, true) &&
          interpolator_B.getDistance(p_B, &distance_B, true) &&
          interpolator_B.getGradient(p_B, &grad_B, true)) {
        src.col(idx) = p_A - distance_A * grad_A;
        tgt.col(idx) = p_B - distance_B * grad_B;

        FloatingPoint weight = tsdf_layer_A->getBlockPtrByCoordinates(p_A)
                                   ->getVoxelByCoordinates(p_A)
                                   .weight +
                               tsdf_layer_B->getBlockPtrByCoordinates(p_B)
                                   ->getVoxelByCoordinates(p_B)
                                   .weight;
        weights.col(idx) = Point(weight, weight, weight);
        ++idx;
      }
    }
  }

  // resize
  src.conservativeResize(Eigen::NoChange, idx);
  tgt.conservativeResize(Eigen::NoChange, idx);
  weights.conservativeResize(Eigen::NoChange, idx);

  getTransformationFromMatchedPoints(src, tgt, weights, T_A_B_out);
}
};

#endif