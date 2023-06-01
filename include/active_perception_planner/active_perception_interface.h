#ifndef ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCEPTION_INTERFACE_H
#define ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCEPTION_INTERFACE_H

#include "ros/ros.h"
#include "oc2_interface_msgs/DetectFruits.h"
#include "oc2_interface_msgs/StartFollowFruit.h"
#include "oc2_interface_msgs/StopFollowFruit.h"


#include<active_perception_planner/active_perciever.h>

namespace active_perception_planner
{

class ActivePercieverInterface
{
public:

  ActivePercieverInterface(ros::NodeHandle& nhp);

  bool processDetectFruitsServiceCallback(const oc2_interface_msgs::DetectFruits::Request& req, oc2_interface_msgs::DetectFruits::Response& res);

  bool processStartFollowFruitsServiceCallback(const oc2_interface_msgs::StartFollowFruit::Request& req, oc2_interface_msgs::StartFollowFruit::Response& res);

  bool processStopFollowFruitsServiceCallback(const oc2_interface_msgs::StopFollowFruit::Request& req, oc2_interface_msgs::StopFollowFruit::Response& res);


protected:

  ros::NodeHandle nh_, nhp_;

  ros::ServiceServer srv_detect_fruits_, srv_start_follow_fruits_, srv_stop_follow_fruits_;

  std::shared_ptr<ActivePerciever> active_perciever_;
};

}

#endif // ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCEPTION_INTERFACE_H
