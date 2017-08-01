#ifndef VOXBLOX_CORE_MESH_LAYER_H_
#define VOXBLOX_CORE_MESH_LAYER_H_

#include <glog/logging.h>
#include <utility>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/mesh/mesh.h"

namespace voxblox {

// A special type of layer just for containing the mesh. Same general interface
// as a layer of blocks, but only contains a single thing, not a set of voxels.
class MeshLayer {
 public:
  typedef std::shared_ptr<MeshLayer> Ptr;
  typedef std::shared_ptr<const MeshLayer> ConstPtr;
  typedef typename BlockHashMapType<Mesh::Ptr>::type MeshMap;

  explicit MeshLayer(FloatingPoint block_size) : block_size_(block_size) {}
  virtual ~MeshLayer() {}

  // By index.
  inline const Mesh& getMeshByIndex(const BlockIndex& index) const {
    Mesh::Ptr mesh_ptr;
    if (mesh_map_.tryFind(index, &mesh_ptr)) {
      return *mesh_ptr;
    } else {
      LOG(FATAL) << "Accessed unallocated mesh at " << index.transpose();
    }
  }

  inline Mesh& getMeshByIndex(const BlockIndex& index) {
    Mesh::Ptr mesh_ptr;
    if (mesh_map_.tryFind(index, &mesh_ptr)) {
      return *mesh_ptr;
    } else {
      LOG(FATAL) << "Accessed unallocated mesh at " << index.transpose();
    }
  }

  inline typename Mesh::ConstPtr getMeshPtrByIndex(
      const BlockIndex& index) const {
    Mesh::Ptr mesh_ptr;
    if (mesh_map_.tryFind(index, &mesh_ptr)) {
      return mesh_ptr;
    } else {
      LOG(WARNING) << "Returning null ptr to mesh!";
      return typename Mesh::ConstPtr();
    }
  }

  inline typename Mesh::Ptr getMeshPtrByIndex(const BlockIndex& index) {
    Mesh::Ptr mesh_ptr;
    if (mesh_map_.tryFind(index, &mesh_ptr)) {
      return mesh_ptr;
    } else {
      LOG(WARNING) << "Returning null ptr to mesh!";
      return typename Mesh::Ptr();
    }
  }

  // Gets a mesh by the mesh index it if already exists,
  // otherwise allocates a new one.
  inline typename Mesh::Ptr allocateMeshPtrByIndex(const BlockIndex& index) {
    return mesh_map_.findOrCreate(index);

    bool was_created;
    typename Mesh::Ptr& mesh_ptr = mesh_map_.findOrCreate(index, &was_created);

    if (was_created) {
      mesh_ptr = std::make_shared<Mesh>(
          block_size_, index.cast<FloatingPoint>() * block_size_);
    }

    return mesh_ptr;
  }

  inline typename Mesh::ConstPtr getMeshPtrByCoordinates(
      const Point& coords) const {
    return getMeshPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  inline typename Mesh::Ptr getMeshPtrByCoordinates(const Point& coords) {
    return getMeshPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Gets a mesh by the coordinates it if already exists,
  // otherwise allocates a new one.
  inline typename Mesh::Ptr allocateMeshPtrByCoordinates(const Point& coords) {
    return allocateMeshPtrByIndex(computeBlockIndexFromCoordinates(coords));
  }

  // Coord to mesh index.
  inline BlockIndex computeBlockIndexFromCoordinates(
      const Point& coords) const {
    return getGridIndexFromPoint(coords, block_size_inv_);
  }

  void removeMesh(const BlockIndex& index) { mesh_map_.erase(index); }

  void removeMeshByCoordinates(const Point& coords) {
    mesh_map_.erase(computeBlockIndexFromCoordinates(coords));
  }

  const MeshMap& getMeshMap() const { return mesh_map_; }

  void combineMesh(Mesh::Ptr combined_mesh) const {
    // Used to prevent double ups in vertices
    BlockHashMapType<IndexElement>::type uniques;

    // Some triangles will have zero area we store them here first then filter
    // them
    VertexIndexList temp_indices;

    // If two vertices closer than (voxel_size / key_multiplication_factor) then
    // the second vertice will be discarded and the first one used in its place
    constexpr FloatingPoint key_multiplication_factor = 100;

    // Combine everything in the layer into one giant combined mesh.
    int v = 0;
    for (Mesh::Ptr& mesh : mesh_map_) {
      for (size_t i = 0; i < mesh->vertices.size(); ++i) {
        // convert from 3D point to key
        BlockIndex vert_key =
            (key_multiplication_factor * mesh->vertices[i] / block_size())
                .cast<IndexElement>();

        bool is_new_key;
        int& stored_v = uniques.findOrCreate(vert_key, &is_new_key);
        if (is_new_key) {
          stored_v = v;
          combined_mesh->vertices.push_back(mesh->vertices[i]);

          if (mesh->hasColors()) {
            combined_mesh->colors.push_back(mesh->colors[i]);
          }
          if (mesh->hasNormals()) {
            combined_mesh->normals.push_back(mesh->normals[i]);
          }

          temp_indices.push_back(v);
          v++;
        } else {
          temp_indices.push_back(stored_v);
        }
      }
    }

    // extract indices of triangles with non-zero area
    for (size_t i = 0; i < temp_indices.size(); i += 3) {
      // check that corners of triangles have not been merged
      if ((temp_indices[i] != temp_indices[i + 1]) &&
          (temp_indices[i] != temp_indices[i + 2]) &&
          (temp_indices[i + 1] != temp_indices[i + 2])) {
        combined_mesh->indices.push_back(temp_indices[i]);
        combined_mesh->indices.push_back(temp_indices[i + 1]);
        combined_mesh->indices.push_back(temp_indices[i + 2]);
      }
    }
  }

  size_t getNumberOfAllocatedMeshes() const { return mesh_map_.size(); }
  // Deletes ALL parts of the mesh.
  void clear() { mesh_map_.clear(); }

  FloatingPoint block_size() const { return block_size_; }

 private:
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint block_size_inv_;

  MeshMap mesh_map_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_MESH_LAYER_H_
