#ifndef ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCEPTION_INTERFACE_H
#define ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCEPTION_INTERFACE_H

#include "ros/ros.h"
#include "oc2_interface_msgs/DetectFruits.h"
#include "oc2_interface_msgs/StartFollowFruit.h"
#include "oc2_interface_msgs/StopFollowFruit.h"
#include "oc2_interface_msgs/SaveJointState.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include<active_perception_planner/active_perciever.h>

namespace active_perception_planner
{

class ActivePercieverInterface
{
public:

  ActivePercieverInterface();

  bool processDetectFruitsServiceCallback(oc2_interface_msgs::DetectFruits::Request& req, oc2_interface_msgs::DetectFruits::Response& res);

  bool processStartFollowFruitsServiceCallback(oc2_interface_msgs::StartFollowFruit::Request& req, oc2_interface_msgs::StartFollowFruit::Response& res);

  bool processStopFollowFruitsServiceCallback(oc2_interface_msgs::StopFollowFruit::Request& req, oc2_interface_msgs::StopFollowFruit::Response& res);

  bool processSaveJointStatesCallback(oc2_interface_msgs::SaveJointState::Request& req, oc2_interface_msgs::SaveJointState::Response& res);


protected:

  ros::NodeHandle nh_, nhp_;

  ros::ServiceServer srv_detect_fruits_, srv_start_follow_fruits_, srv_stop_follow_fruits_, srv_save_js_;

  std::shared_ptr<ActivePerciever> active_perciever_;

  ros::Publisher pub_superellipsoids_;

  oc2_interface_msgs::DetectFruits::Response getFruitMapAsSrvResponse();

  void publishVisualizations();

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  std::string config_package_path_;

  void loadJointStatesasRememberedJointValues();

};

}

#endif // ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCEPTION_INTERFACE_H
