#ifndef ACTIVE_PERCEPTION_PLANNER_ONLINE_POSE_TRACKER_H
#define ACTIVE_PERCEPTION_PLANNER_ONLINE_POSE_TRACKER_H

#include "active_perception_planner/common.h"
#include<vector>
#include<memory>
#include <iostream>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>
#include <oc2_interface_msgs/ControllerCmd.h>
#include <oc2_interface_msgs/ControllerStatus.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <xarm_msgs/RobotMsg.h>
#include <xarm_msgs/SetInt16.h>
namespace active_perception_planner
{

class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      //ROS_INFO_STREAM_NAMED_THROTTLE(10.0, "Servo", "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};

// enum DawnIKControllerMode
// {
//   ROBOT_IDLE = dawn_ik::IKGoal::MODE_0,
//   ENDPOINT_POSE = dawn_ik::IKGoal::MODE_1,
//   ENDPOINT_POSE_ORIENTATION = dawn_ik::IKGoal::MODE_1 | dawn_ik::IKGoal::MODE_2,
//   ENDPOINT_POSE_LOOKATGOAL = dawn_ik::IKGoal::MODE_1 | dawn_ik::IKGoal::MODE_3
// };

// struct DawnIKGoals
// {
//   geometry_msgs::Pose pose;
//   geometry_msgs::Pose target;
// };


class OnlinePoseTracker
{
  public:

  OnlinePoseTracker(ros::NodeHandle& nhp);

  // bool updateGoal(DawnIKControllerMode mode, const DawnIKGoals& goals);

  // bool lookAtTarget(const Eigen::Vector3d& position, const Eigen::Vector3d& target);

  // bool startDawnIK();

  // bool stopDawnIK();

  bool fullPoseTarget(const geometry_msgs::Pose& desired_pose);

  inline bool isIKControllerEnabled()
  {
    return is_enabled_;
  }

  oc2_interface_msgs::ControllerStatus getOTGStatus();

  inline bool isHeadStationary()
  {
    return is_head_stationary_.load();
  }

  bool isOTGRunning();

  bool isOTGReadyForNewTarget();

  void stopOTGMotion();

  bool updateOTGGoalPose(const geometry_msgs::PoseStamped& pose_stamped, const std::string& end_effector_frame);


  geometry_msgs::PoseStamped getTrackerCurrentPose();

  void setTrackerTargetPose(geometry_msgs::PoseStamped& target_pose);

  void stopTrackerMotion();

  bool switchController(std::string start_controller, std::string stop_controller);

  inline void switchXarmToOTG()
  {
    switch_on_robot_ = true;
    switchXarmMode(7);
  }

  inline void switchXarmToTraj()
  {
    //ROS_WARN_THROTTLE
    switch_on_robot_ = true;
    switchXarmMode(1);
  }

  void switchXarmMode(int mode = 1);

  inline bool isOTGModeReady(){return is_ready_ && xarm_robot_msg_.mode == 7;}

  inline bool isArmReady(){return is_ready_;}

  inline bool isArmChangingConfig(){return is_changing_config_;}

  private:
  ros::NodeHandle nhp_;
  ros::Publisher pub_goal_;

  ros::Publisher pub_otg_goal_;

  ros::Subscriber sub_otg_status_;

  oc2_interface_msgs::ControllerStatus otg_status_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  std::shared_ptr<moveit_servo::PoseTracking> tracker_;

  ros::Publisher pub_target_pose_;

  bool switch_on_robot_ = false;

  int desired_mode_ = 1;
  bool is_ready_ = false;
  bool is_changing_config_ = false;

  std::atomic_bool is_head_stationary_ = true;


  //StatusMonitor status_monitor_;

  Eigen::Vector3d lin_tol_{ 0.01, 0.01, 0.01 };

  double rot_tol_ = 0.1;

  ros::ServiceClient client_start_dawn_ik_;
  ros::ServiceClient client_stop_dawn_ik_;

  ros::ServiceClient client_xarm_state_;
  ros::ServiceClient client_xarm_mode_;

  ros::Subscriber sub_xarm_robot_msg_;
  xarm_msgs::RobotMsg xarm_robot_msg_;
  bool xarm_state_recd_ = false;

  ros::Subscriber sub_xarm_js_;
  sensor_msgs::JointState joint_state_msg_;
	bool joint_state_recd_ = false;

  ros::Timer timer_check_status_;

  ros::ServiceClient client_switch_controllers_;

  bool is_enabled_ = false; 

  ros::Timer timer_move_to_pose_;

  private: 

  bool callEmptyService(ros::ServiceClient& client);

  void moveToPoseTimerCallback(const ros::TimerEvent& e);

  void checkArmStatusTimerCallback(const ros::TimerEvent& e);

  void publishOTGControllerCmd(const oc2_interface_msgs::ControllerCmd& cmd);

  void registerOTGStatusCallback(const oc2_interface_msgs::ControllerStatus& msg);


  void registerXarmStateCallback(const xarm_msgs::RobotMsg& msg);

  void registerJointStateCallback(const sensor_msgs::JointState& msg);


};

}

#endif