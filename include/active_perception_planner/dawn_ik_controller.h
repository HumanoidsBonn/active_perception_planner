#ifndef ACTIVE_PERCEPTION_PLANNER_DAWN_IK_CONTROLLER_H
#define ACTIVE_PERCEPTION_PLANNER_DAWN_IK_CONTROLLER_H

#include "active_perception_planner/common.h"
#include "/home/rohit/workspace/ros1/oc2_ws/devel/include/dawn_ik/IKGoal.h"
#include<vector>
#include<memory>
#include <iostream>

namespace active_perception_planner
{

enum DawnIKControllerMode
{
  ROBOT_IDLE = dawn_ik::IKGoal::MODE_0,
  ENDPOINT_POSE = dawn_ik::IKGoal::MODE_1,
  ENDPOINT_POSE_ORIENTATION = dawn_ik::IKGoal::MODE_1 | dawn_ik::IKGoal::MODE_2,
  ENDPOINT_POSE_LOOKATGOAL = dawn_ik::IKGoal::MODE_1 | dawn_ik::IKGoal::MODE_3
};

struct DawnIKGoals
{
  geometry_msgs::Pose pose;
  geometry_msgs::Pose target;
};


class DawnIKController
{
  public:

  DawnIKController(ros::NodeHandle& nhp);

  bool updateGoal(DawnIKControllerMode mode, const DawnIKGoals& goals);

  bool lookAtTarget(const Eigen::Vector3d& position, const Eigen::Vector3d& target);

  bool startDawnIK();

  bool stopDawnIK();

  bool fullPoseTarget(const geometry_msgs::Pose& desired_pose);

  inline bool isIKControllerEnabled()
  {
    return is_enabled_;
  }


  private:
  ros::NodeHandle nhp_;
  ros::Publisher pub_goal_;

  ros::ServiceClient client_start_dawn_ik_;
  ros::ServiceClient client_stop_dawn_ik_;

  bool is_enabled_ = false; 

  private: 

  bool callEmptyService(ros::ServiceClient& client);

};

}

#endif