#include "active_perception_planner/online_viewpoint_planner.h"

using namespace active_perception_planner;

OnlineViewpointPlanner::OnlineViewpointPlanner(ros::NodeHandle& nhp, std::shared_ptr<pointcloud_roi::DetectedRoiFilter>& roi_detector):nhp_(nhp), roi_detector_(roi_detector)
{

}

void OnlineViewpointPlanner::setCrop(const Crop& crop, oc2_interface_msgs::Crop& crop_msg)
{
  crop_ = crop;
  crop_msg_ = crop_msg;
}