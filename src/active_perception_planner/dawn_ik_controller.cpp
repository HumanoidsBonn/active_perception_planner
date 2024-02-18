#include "active_perception_planner/dawn_ik_controller.h"

using namespace active_perception_planner;

DawnIKController::DawnIKController(ros::NodeHandle& nhp):nhp_(nhp)
{
  pub_goal_ = nhp_.advertise<dawn_ik::IKGoal>("/dawn_ik_solver/ik_goal", 1);
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
  DawnIKGoals goals;
  goals.pose.position.x = position.x();
  goals.pose.position.y = position.y();
  goals.pose.position.z = position.z();
  goals.target.position.x = target.x();
  goals.target.position.y = target.y();
  goals.target.position.z = target.z();
  return updateGoal(mode, goals);
}

