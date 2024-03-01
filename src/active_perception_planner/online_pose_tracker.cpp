#include "active_perception_planner/online_pose_tracker.h"
#include <std_srvs/Empty.h>
#include <controller_manager_msgs/SwitchController.h>


using namespace active_perception_planner;

OnlinePoseTracker::OnlinePoseTracker(ros::NodeHandle& nhp):nhp_(nhp)//, status_monitor_(nhp_, "status")
{
  pub_goal_ = nhp_.advertise<dawn_ik::IKGoal>("/dawn_ik_solver/ik_goal", 1);
  client_start_dawn_ik_  = nhp_.serviceClient<std_srvs::Empty>("/dawn_ik_solver/start");
  client_stop_dawn_ik_   = nhp_.serviceClient<std_srvs::Empty>("/dawn_ik_solver/stop");

  pub_otg_goal_ = nhp_.advertise<oc2_interface_msgs::ControllerCmd>("/head/controller/goal_pose", 1);
  sub_otg_status_ = nhp_.subscribe("/head/controller/status", 1, &OnlinePoseTracker::registerOTGStatusCallback, this);

  timer_check_status_ = nhp_.createTimer(ros::Duration(0.1), &OnlinePoseTracker::checkArmStatusTimerCallback, this);

  client_xarm_state_ = nhp_.serviceClient<xarm_msgs::SetInt16>("/head/set_state");
  client_xarm_mode_  = nhp_.serviceClient<xarm_msgs::SetInt16>("/head/set_mode");

  sub_xarm_robot_msg_ = nhp_.subscribe("/head/xarm_states", 1, &OnlinePoseTracker::registerXarmStateCallback, this);

  //sub_xarm_js_ = nhp_.subscribe("/head/joint_states", &OnlinePoseTracker::registerJointStateCallback)


  //client_switch_controllers_ = nhp_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

  // Make a publisher for sending pose commands
  //pub_target_pose_ = nhp_.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

  // timer_move_to_pose_ = nhp_.createTimer(ros::Duration(0.1), &DawnIKController::moveToPoseTimerCallback, this);
  // timer_move_to_pose_.stop();


  // planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  // if (!planning_scene_monitor_->getPlanningScene())
  // {
  //   ROS_ERROR_STREAM_NAMED("DawnIKController", "Error in setting up the PlanningSceneMonitor.");
  //   exit(EXIT_FAILURE);
  // }

  // planning_scene_monitor_->startSceneMonitor();
  // planning_scene_monitor_->startWorldGeometryMonitor(
  // planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
  // planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
  //     false /* skip octomap monitor */);
  // planning_scene_monitor_->startStateMonitor();

  // tracker_ = std::make_shared<moveit_servo::PoseTracking>(nhp_, planning_scene_monitor_);

}

bool OnlinePoseTracker::startDawnIK()
{
  if(callEmptyService(client_start_dawn_ik_))
  {
    is_enabled_ = true;
    return true;
  }
  return false;

}
bool OnlinePoseTracker::stopDawnIK()
{
  if(callEmptyService(client_stop_dawn_ik_))
  {
    is_enabled_ = false;
    return true;
  }
  return false;
}

bool OnlinePoseTracker::updateGoal(DawnIKControllerMode mode, const DawnIKGoals& goals)
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

bool OnlinePoseTracker::lookAtTarget(const Eigen::Vector3d& position, const Eigen::Vector3d& target)
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

bool OnlinePoseTracker::fullPoseTarget(const geometry_msgs::Pose& desired_pose)
{
  DawnIKControllerMode mode = DawnIKControllerMode::ENDPOINT_POSE_ORIENTATION;
  DawnIKGoals goal;
  goal.pose = desired_pose;
  return updateGoal(mode, goal);
}

bool OnlinePoseTracker::callEmptyService(ros::ServiceClient& client)
{
  std_srvs::Empty empty_srv;
  if (!client.call(empty_srv))
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
  return true;
}

geometry_msgs::PoseStamped OnlinePoseTracker::getTrackerCurrentPose()
{

  geometry_msgs::TransformStamped current_ee_tf;
  tracker_->getCommandFrameTransform(current_ee_tf);
  // Convert it to a Pose
  geometry_msgs::PoseStamped current_pose;
  current_pose.header.frame_id = current_ee_tf.header.frame_id;
  current_pose.pose.position.x = current_ee_tf.transform.translation.x;
  current_pose.pose.position.y = current_ee_tf.transform.translation.y;
  current_pose.pose.position.z = current_ee_tf.transform.translation.z;
  current_pose.pose.orientation = current_ee_tf.transform.rotation;

  return current_pose;
}

void OnlinePoseTracker::setTrackerTargetPose(geometry_msgs::PoseStamped& target_pose)
{
  tracker_->resetTargetPose();
  target_pose.header.stamp = ros::Time::now();
  pub_target_pose_.publish(target_pose);
  timer_move_to_pose_.start();
}

void OnlinePoseTracker::stopTrackerMotion()
{
  tracker_->stopMotion();
  timer_move_to_pose_.stop();
}

void OnlinePoseTracker::moveToPoseTimerCallback(const ros::TimerEvent& e)
{
  tracker_->moveToPose(lin_tol_, rot_tol_, 0.15);
}

bool OnlinePoseTracker::switchController(std::string start_controller, std::string stop_controller)
{
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers.push_back(start_controller);
  srv.request.stop_controllers.push_back(stop_controller);
  srv.request.strictness = 2;

  if(client_switch_controllers_.call(srv))
  {
    return srv.response.ok;
  }
  else
  {
    return false;
  }
}

bool OnlinePoseTracker::updateOTGGoalPose(const geometry_msgs::PoseStamped& pose_stamped, const std::string& end_effector_frame)
{
  if(otg_status_.status  == oc2_interface_msgs::ControllerStatus::COLLISION)
  {
    ROS_WARN("Cannot accept new goals. Controller in collision");
    return false;
  }
  oc2_interface_msgs::ControllerCmd cmd;
  cmd.goal_pose = pose_stamped.pose;
  cmd.target_frame = end_effector_frame;
  cmd.header = pose_stamped.header;
  cmd.stop = false;
  cmd.header.stamp = ros::Time::now();
  publishOTGControllerCmd(cmd);
  return true;
}

void OnlinePoseTracker::stopOTGMotion()
{
  oc2_interface_msgs::ControllerCmd cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.stop = true;
  cmd.target_frame = "zed2i_camera_center";
  cmd.header.frame_id = "trolley_local_link";

  return publishOTGControllerCmd(cmd);
}

void OnlinePoseTracker::publishOTGControllerCmd(const oc2_interface_msgs::ControllerCmd& cmd)
{
  pub_otg_goal_.publish(cmd);
}

oc2_interface_msgs::ControllerStatus OnlinePoseTracker::getOTGStatus()
{
  return otg_status_;
}

bool OnlinePoseTracker::isOTGRunning()
{
  return otg_status_.status != 3 || otg_status_.status != oc2_interface_msgs::ControllerStatus::WAITING;
}

bool OnlinePoseTracker::isOTGReadyForNewTarget()
{
  return (otg_status_.status == oc2_interface_msgs::ControllerStatus::WAITING);
}

void OnlinePoseTracker::registerOTGStatusCallback(const oc2_interface_msgs::ControllerStatus& msg)
{
  otg_status_  = msg; 
}

void OnlinePoseTracker::registerXarmStateCallback(const xarm_msgs::RobotMsg& msg)
{
    xarm_robot_msg_ = msg;
    xarm_state_recd_ = true;
}

void OnlinePoseTracker::registerJointStateCallback(const sensor_msgs::JointState& msg)
{
    joint_state_msg_ = msg;
    joint_state_recd_ = true;
}

void OnlinePoseTracker::checkArmStatusTimerCallback(const ros::TimerEvent& e)
{
  if(!xarm_state_recd_)
  {
    ROS_WARN_THROTTLE(2.0,"waiting for robot states...");
    is_ready_ = false;
    return;
  }

  double state_delay = (ros::Time::now() - xarm_robot_msg_.header.stamp).toSec();
  if(state_delay > 0.5)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "last robot state is too old"<<state_delay);
    is_ready_ = false;
    return;
  }

  if(xarm_robot_msg_.mode != desired_mode_)
  {
    ROS_INFO_STREAM_THROTTLE(2.0,"wrong robot mode "<<xarm_robot_msg_.mode);
    ROS_WARN_STREAM("Switching to mode "<<desired_mode_<<"...");
    xarm_msgs::SetInt16 srv;
    srv.request.data = desired_mode_;
    if(!client_xarm_mode_.call(srv) || srv.response.ret != 0)
        ROS_ERROR_STREAM("could not switch mode "<<srv.response.message);
    is_ready_ = false;
    is_changing_config_ = true;
    return;
  }

  if(xarm_robot_msg_.state > 2 && switch_on_robot_ == true)
  {
    ROS_INFO_STREAM_THROTTLE(2.0,"wrong robot mode "<<xarm_robot_msg_.state);
    ROS_WARN_STREAM(" Switching to state 0...");
    xarm_msgs::SetInt16 srv;
    srv.request.data = 0;
    if(!client_xarm_state_.call(srv) || srv.response.ret != 0)
      ROS_ERROR_STREAM("could not switch mode "<<srv.response.message);
    is_ready_ = false;
    is_changing_config_ = true;
    return;
  }

  if(xarm_state_recd_ && state_delay < 0.5 && xarm_robot_msg_.mode == desired_mode_ && xarm_robot_msg_.state < 3)
  {
    is_ready_ = true;
    switch_on_robot_ = false;
    return;
  }

}

void OnlinePoseTracker::switchXarmMode(int mode)
{
  if(mode != xarm_robot_msg_.mode )
  {
    desired_mode_ = mode;
  }
}


