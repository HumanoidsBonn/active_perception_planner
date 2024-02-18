#ifndef ACTIVE_PERCEPTION_PLANNER_VOXBLOX_ESDF_MAP_H
#define ACTIVE_PERCEPTION_PLANNER_VOXBLOX_ESDF_MAP_H
#include <voxblox_ros/esdf_server.h>
#include "active_perception_planner/voxblox_tsdf_map.h"

#include <memory>

namespace active_perception_planner {

enum VoxelState
{
  OCCUPIED = 0,
  FREE = 1,
  UNKNOWN = -1
};

class VoxbloxEsdfMap: public VoxbloxTsdfMap
{

public:

  VoxbloxEsdfMap(ros::NodeHandle& nhp);

// check whether point is part of the map
  bool isObserved(const Eigen::Vector3d& point);

  // get occupancy
  int getVoxelState(const Eigen::Vector3d& point);

  // get voxel size
  double getVoxelSize();

  // get the center of a voxel from input point
  bool getVoxelCenter(Eigen::Vector3d* center,
                      const Eigen::Vector3d& point);

  // get the stored distance
  double getVoxelDistance(const Eigen::Vector3f& point) override;

  double getVoxelWeight(const Eigen::Vector3f& point) override;

  // accessor to the server for specialized planners
  voxblox::EsdfServer& getESDFServer();

protected:

  // esdf server that contains the map, subscribe to external ESDF/ESDF updates
  std::unique_ptr<voxblox::EsdfServer> esdf_server_;

  double c_voxel_size_;
  double c_block_size_;

  bool tsdf_layer_present_ = false;


};
}
#endif // ACTIVE_PERCEPTION_PLANNER_VOXBLOX_MAP_H
