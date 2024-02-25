#ifndef ACTIVE_PERCEPTION_PLANNER_ONLINE_VIEWPOINT_PLANNER_H
#define ACTIVE_PERCEPTION_PLANNER_ONLINE_VIEWPOINT_PLANNER_H

#include "active_perception_planner/common.h"
#include <oc2_interface_msgs/Crop.h>
#include<vector>
#include<memory>
#include <iostream>

#include <ros/ros.h>
#include "pointcloud_roi/detected_roi_filter.h"
#include "active_perception_planner/dawn_ik_controller.h"
#include "active_perception_planner/voxblox_tsdf_map.h"

namespace active_perception_planner
{
class OnlineViewpointPlanner
{
  public:
  OnlineViewpointPlanner(ros::NodeHandle& nhp, std::shared_ptr<pointcloud_roi::DetectedRoiFilter>& roi_detector);

  void setCrop(const Crop& crop, oc2_interface_msgs::Crop& crop_msg);

  private:

  ros::NodeHandle nhp_;
  ros::Timer timer_fruit_follower_;

  std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector_;
  std::shared_ptr<active_perception_planner::DawnIKController> ik_controller_;
  std::shared_ptr<active_perception_planner::VoxbloxTsdfMap> voxblox_map_;

  Crop crop_;
  oc2_interface_msgs::Crop crop_msg_;

  active_perception_planner::DetectedFruitWithPeduncle interested_fruit_;

  



};
}


#endif