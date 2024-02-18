#include "active_perception_planner/voxblox_esdf_map.h"
#include <voxblox_ros/ros_params.h>

using namespace active_perception_planner;

VoxbloxEsdfMap::VoxbloxEsdfMap(ros::NodeHandle& nhp):VoxbloxTsdfMap(nhp, false)
{
  esdf_server_.reset(new voxblox::EsdfServer(nh_, nhp_));

  // cache constants
  c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();
}
voxblox::EsdfServer& VoxbloxEsdfMap::getESDFServer() { return *esdf_server_; }


bool VoxbloxEsdfMap::isObserved(const Eigen::Vector3d& point) {
  return esdf_server_->getEsdfMapPtr()->isObserved(point);
}

// get occupancy
int VoxbloxEsdfMap::getVoxelState(const Eigen::Vector3d& point) {
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
double VoxbloxEsdfMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool VoxbloxEsdfMap::getVoxelCenter(Eigen::Vector3d* center,
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

// get the stored ESDF distance
double VoxbloxEsdfMap::getVoxelDistance(const Eigen::Vector3f& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::EsdfVoxel>::Ptr block =
      esdf_server_->getEsdfMapPtr()
          ->getEsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::EsdfVoxel* esdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (esdf_voxel) {
      return esdf_voxel->distance;
    }
  }
  return 0.0;
}

// get the stored weight
double VoxbloxEsdfMap::getVoxelWeight(const Eigen::Vector3f& point) {
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

