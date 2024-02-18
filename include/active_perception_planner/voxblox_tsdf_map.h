#ifndef ACTIVE_PERCEPTION_PLANNER_VOXBLOX_TSDF_MAP_H
#define ACTIVE_PERCEPTION_PLANNER_VOXBLOX_TSDF_MAP_H
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_msgs/Layer.h>
#include <vpp_msgs/GetTsdfMap.h>


namespace active_perception_planner {

class VoxbloxTsdfMap
{
  public:

  VoxbloxTsdfMap(ros::NodeHandle& nhp, bool is_service_based = true);

  voxblox::TsdfVoxel getVoxelFromMap(const voxblox::Point& point);

    // get the stored weight
  double getVoxelWeight(const Eigen::Vector3f& point);

  double getVoxelDistance(const Eigen::Vector3f& point);

  // get the maximum allowed weight (return 0 if using uncapped weights)
  double getMaximumWeight();

  protected:
  ros::NodeHandle nh_, nhp_;
  ros::ServiceClient get_tsdf_map_client_;
  ros::Timer timer_tsdf_client_;

  bool is_service_based_ = true;
  bool tsdf_layer_present_ = false;
  bool tsdf_layer_updated_ = false;
  bool enable_cont_update_ = false;


  // cache constants
  double c_maximum_weight_;

  voxblox::Layer<voxblox::TsdfVoxel>* getTsdfLayerPtr();

  void updateTsdfMap();

  void updateTSDFMapCallback(const ros::TimerEvent&);

  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_map_layer_;
  std::mutex layer_mutex_;
};

}

#endif 

