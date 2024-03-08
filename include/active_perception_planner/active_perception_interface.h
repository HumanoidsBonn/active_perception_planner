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

#include <actionlib/server/simple_action_server.h>
#include <oc2_interface_msgs/DetectFruitsAction.h>
#include <oc2_interface_msgs/StartFollowPeduncle.h>

#include <yolact_ros_msgs/SetConfidence.h>
#include <yolact_ros_msgs/SetRegionOfInterest.h>

namespace active_perception_planner
{

class ActivePercieverInterface
{
public:

  ActivePercieverInterface();

  bool processDetectFruitsServiceCallback(oc2_interface_msgs::DetectFruits::Request& req, oc2_interface_msgs::DetectFruits::Response& res);

  bool processStartFollowFruitsServiceCallback(oc2_interface_msgs::StartFollowFruit::Request& req, oc2_interface_msgs::StartFollowFruit::Response& res);

  bool processStartFollowPeduncleServiceCallback(oc2_interface_msgs::StartFollowPeduncle::Request &req, oc2_interface_msgs::StartFollowPeduncle::Response &res);


  bool processStopFollowFruitsServiceCallback(oc2_interface_msgs::StopFollowFruit::Request& req, oc2_interface_msgs::StopFollowFruit::Response& res);

  bool processSaveJointStatesCallback(oc2_interface_msgs::SaveJointState::Request& req, oc2_interface_msgs::SaveJointState::Response& res);

  void executeDetectionServerGoalCallback();

  void preemptDetectionServerGoalCallback();


protected:

  ros::NodeHandle nh_, nhp_;

  ros::ServiceServer srv_detect_fruits_, srv_start_follow_fruits_, srv_start_follow_peduncle_, srv_stop_follow_fruits_, srv_save_js_;

  ros::ServiceClient client_start_depth_seg_, client_stop_depth_seg_, client_start_obj_det_, client_stop_obj_det_;

  ros::ServiceClient client_start_depth_seg_d405_, client_stop_depth_seg_d405_, client_start_obj_det_d405_, client_stop_obj_det_d405_;

  ros::ServiceClient client_set_confidence_iou_, client_set_roi_;

  using DetectionAction = oc2_interface_msgs::DetectFruitsAction;
	using DetectionServer = actionlib::SimpleActionServer<DetectionAction>;
	std::shared_ptr<DetectionServer> act_detection_server_;
  oc2_interface_msgs::DetectFruitsResult detection_result_;

  bool disable_motion_ = false;

  bool disable_d405_processing_ = false;

  double wait_time_;

  std::shared_ptr<ActivePerciever> active_perciever_;

  ros::Publisher pub_superellipsoids_;
  ros::Publisher pub_crop_;

  oc2_interface_msgs::DetectFruits::Response getFruitMapAsSrvResponse();

  //oc2_interface_msgs::DetectFruitsResult getFruitMapAsActionResult();

  oc2_interface_msgs::DetectFruitsResult getCropAsActionResult();



  void publishVisualizations();

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  std::string config_package_path_;

  void loadJointStatesasRememberedJointValues();

  bool callEmptyService(ros::ServiceClient& client);


};

}

#endif // ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCEPTION_INTERFACE_H
