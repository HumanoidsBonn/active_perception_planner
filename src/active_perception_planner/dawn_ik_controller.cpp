#include "active_perception_planner/dawn_ik_controller.h"
#include <std_srvs/Empty.h>

using namespace active_perception_planner;

DawnIKController::DawnIKController(ros::NodeHandle& nhp):nhp_(nhp)
{
  pub_goal_ = nhp_.advertise<dawn_ik::IKGoal>("/dawn_ik_solver/ik_goal", 1);
  client_start_dawn_ik_  = nhp_.serviceClient<std_srvs::Empty>("/dawn_ik_solver/start");
  client_stop_dawn_ik_   = nhp_.serviceClient<std_srvs::Empty>("/dawn_ik_solver/stop");
}

bool DawnIKController::startDawnIK()
{
  if(callEmptyService(client_start_dawn_ik_))
  {
    is_enabled_ = true;
    return true;
  }
  return false;

}
bool DawnIKController::stopDawnIK()
{
  if(callEmptyService(client_stop_dawn_ik_))
  {
    is_enabled_ = false;
    return true;
  }
  return false;
}

bool DawnIKController::updateGoal(DawnIKControllerMode mode, const DawnIKGoals& goals)
{
  dawn_ik::IKGoal goal;
  goal.mode = mode;
  if (mode == DawnIKControllerMode::ENDPOINT_POSE)
  {
    goal.m1_x = goals.pose.position.x;
    goal.m1_y = goals.pose.position.y;
    goal.m1_z = goals.pose.position.z;
    goal.m1_limit_dist = 0.1;
    goal.m1_weight = 4.0;
  }
  else if (mode == DawnIKControllerMode::ENDPOINT_POSE_ORIENTATION)
  {
    goal.m1_x = goals.pose.position.x;
    goal.m1_y = goals.pose.position.y;
    goal.m1_z = goals.pose.position.z;
    goal.m1_limit_dist = 0.1;
    goal.m1_weight = 4.0;
    goal.m2_w = goals.pose.orientation.w;
    goal.m2_x = goals.pose.orientation.x;
    goal.m2_y = goals.pose.orientation.y;
    goal.m2_z = goals.pose.orientation.z;
    goal.m2_weight = 1.0;
  }
  else if (mode == DawnIKControllerMode::ENDPOINT_POSE_LOOKATGOAL)
  {
    goal.m1_x = goals.pose.position.x;
    goal.m1_y = goals.pose.position.y;
    goal.m1_z = goals.pose.position.z;
    goal.m1_weight = 1.0;
    goal.m3_x = goals.target.position.x;
    goal.m3_y = goals.target.position.y;
    goal.m3_z = goals.target.position.z;
    goal.m3_weight = 1.0;
  }
  else
  {
    ROS_ERROR("DawnIKController::updateGoal: Unknown mode: %d", mode);
    return false;
  }
  pub_goal_.publish(goal);
  return true;

}

bool DawnIKController::lookAtTarget(const Eigen::Vector3d& position, const Eigen::Vector3d& target)
{
  DawnIKControllerMode mode = DawnIKControllerMode::ENDPOINT_POSE_LOOKATGOAL;
  DawnIKGoals goal;
  goal.pose.position.x = position.x();
  goal.pose.position.y = position.y();
  goal.pose.position.z = position.z();
  goal.target.position.x = target.x();
  goal.target.position.y = target.y();
  goal.target.position.z = target.z();
  return updateGoal(mode, goal);
}

bool DawnIKController::fullPoseTarget(const geometry_msgs::Pose& desired_pose)
{
  DawnIKControllerMode mode = DawnIKControllerMode::ENDPOINT_POSE_ORIENTATION;
  DawnIKGoals goal;
  goal.pose = desired_pose;
  return updateGoal(mode, goal);
}

bool DawnIKController::callEmptyService(ros::ServiceClient& client)
{
  std_srvs::Empty empty_srv;
  if (!client.call(empty_srv))
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
  return true;
}