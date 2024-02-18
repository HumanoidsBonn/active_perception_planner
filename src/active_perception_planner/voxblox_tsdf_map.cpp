#include "active_perception_planner/voxblox_tsdf_map.h"
#include <voxblox_ros/ros_params.h>

using namespace active_perception_planner;

VoxbloxTsdfMap::VoxbloxTsdfMap(ros::NodeHandle& nhp, bool is_service_based):nh_(""), nhp_(nhp), is_service_based_(is_service_based)
{
  c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nhp_)
                          .max_weight;  // direct access is not exposed

  if(is_service_based_)
  {
    get_tsdf_map_client_ = nh_.serviceClient<vpp_msgs::GetTsdfMap>("/gsm_node/get_tsdf_map");
    timer_tsdf_client_.start(); 
  }
  else
  {
    timer_tsdf_client_.stop();
  }
}

void VoxbloxTsdfMap::updateTsdfMap()
{
  vpp_msgs::GetTsdfMap tsdf_map_srv;
  bool success = get_tsdf_map_client_.call(tsdf_map_srv);
  if (!success)
  {
    tsdf_layer_updated_ = false;
    ROS_ERROR("Failed to call get tsdf vpp map service");
    return;
  }
  {
    std::scoped_lock lk(layer_mutex_);
    bool success = voxblox::deserializeMsgToLayer(tsdf_map_srv.response.voxblox_map, tsdf_map_layer_);
    if(success)
    {
    tsdf_layer_present_ = true;
    tsdf_layer_updated_ = true;
    }
  }
}

void VoxbloxTsdfMap::updateTSDFMapCallback(const ros::TimerEvent& event)
{
  if(is_service_based_ == false)
    return;

  updateTsdfMap();
}

voxblox::Layer<voxblox::TsdfVoxel> * VoxbloxTsdfMap::getTsdfLayerPtr()
{
  voxblox::Layer<voxblox::TsdfVoxel> * tsdf_layer;
  {
    std::scoped_lock lk(layer_mutex_);
    tsdf_layer = tsdf_map_layer_;
  }
  return tsdf_layer;
}

voxblox::TsdfVoxel VoxbloxTsdfMap::getVoxelFromMap(const voxblox::Point& point)
{
  voxblox::TsdfVoxel voxel;
  auto block = getTsdfLayerPtr()->getBlockPtrByCoordinates(point);
  if(block == nullptr)
  {
      ROS_INFO_STREAM("No Block in Map!");
      voxel.distance = 0;
      voxel.weight = 0;
  }
  else
  {
      voxel = block->getVoxelByCoordinates(point);
  }
  return voxel;
}

double VoxbloxTsdfMap::getVoxelWeight(const Eigen::Vector3f& point)
{
  return getVoxelFromMap(point).weight;
}

double VoxbloxTsdfMap::getVoxelDistance(const Eigen::Vector3f& point)
{
  return getVoxelFromMap(point).distance;
}

double VoxbloxTsdfMap::getMaximumWeight()
{
  return c_maximum_weight_;
}