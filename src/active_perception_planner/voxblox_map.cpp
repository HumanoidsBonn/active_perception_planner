#include "active_perception_planner/voxblox_map.h"

using namespace active_perception_planner;

bool VoxbloxMap::isObserved(const Eigen::Vector3d& point) {
  return esdf_server_->getEsdfMapPtr()->isObserved(point);
}

// get occupancy
int VoxbloxMap::getVoxelState(const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance < c_voxel_size_) {
      return VoxelState::OCCUPIED;
    } else {
      return VoxelState::FREE;
    }
  } else {
    return VoxelState::UNKNOWN;
  }
}

// get voxel size
double VoxbloxMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool VoxbloxMap::getVoxelCenter(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = esdf_server_->getEsdfMapPtr()
                                     ->getEsdfLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

// get the stored TSDF distance
double VoxbloxMap::getVoxelDistance(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      esdf_server_->getTsdfMapPtr()
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->distance;
    }
  }
  return 0.0;
}

// get the stored weight
double VoxbloxMap::getVoxelWeight(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      esdf_server_->getTsdfMapPtr()
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->weight;
    }
  }
  return 0.0;
}

// get the maximum allowed weight (return 0 if using uncapped weights)
double VoxbloxMap::getMaximumWeight() { return c_maximum_weight_; }
