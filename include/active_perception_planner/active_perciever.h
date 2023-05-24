#ifndef ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H
#define ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include<moveit_motion_planner/motion_planner.h>
#include<capsicum_superellipsoid_detector/se_detector.h>
#include<oc2_interface_msgs/DetectedFruits.h>
#include<vpp_msgs/GetScenePointcloud.h>
#include<vpp_msgs/GetListInstancePointclouds.h>
#include<pcl_conversions/pcl_conversions.h>
namespace active_perception_planner {

enum ActivePercieverStatus
{
  IDLE = 0,
  LOCAL_MAPPING_MOTION_IN_PROGRESS = 1,
  LOCAL_MAPPING_MOTION_SUCCESS = 2,
  FRUIT_MAP_GENERATION_IN_PROGRESS = 3,

  FRUIT_MAP_GENERATION_SUCCESS = 4,
  FRUIT_FOLLOWING_IN_PROGRESS = 5,
  PEDUNCLE_DETECTION_PROGRESS = 6,

  ERROR_MANIP_ARMS_NOT_STOWED = -1,
  ERROR_MOVE_TO_INIT_MAPPING_POSE = -2,
  ERROR_LOCAL_MAPPING_MOTION = -3,
  ERROR_MAP_GENERATION = -4,
  ERROR_NO_FRUITS_IN_MAP = -5,
  ERROR_SHAPE_COMPLETION = -6,
  ERROR_FRUIT_FOLLOWING = -7
};

typedef std::map<uint, superellipsoid::DetectedFruit> FruitMap;
typedef std::map<uint, superellipsoid::DetectedFruit>::iterator FruitMapIterator;


class ActivePerciever{

public:

  ActivePerciever(ros::NodeHandle& nhp, const std::string& planning_group_name, const std::string& base_frame, const std::string& tip_frame);

  ActivePercieverStatus getActivePercieverStatus();

  bool clearSceneMap();

  ActivePercieverStatus performLocalMappingMotion();

  bool createSceneMapWithCompletedFruits();

  bool moveToInterestedFruitObservationPose(const uint& fruit_id);

  ActivePercieverStatus followFruit(const uint& fruit_id, const geometry_msgs::Point& fruit_centre);


protected:
  ActivePercieverStatus status_;
  ros::NodeHandle nh_, nhp_;

  std::shared_ptr<motion_planning::MotionPlanner> motion_planner_;
  std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector_;

  std::string planning_group_name_;
  std::string base_frame_;
  std::string tip_frame_;

  std::string moveit_planner_type_;
  std::string motion_type_;

  int num_planning_attempts_;
  float allowed_planning_time_;

  bool attempt_with_ptp_on_failure_ = true;

  ros::ServiceClient reset_map_client_;
  ros::ServiceClient get_instanceclouds_client_;
  ros::ServiceClient get_scenecloud_client_;

  FruitMap fruit_map_;
  superellipsoid::DetectedFruit interested_fruit_;

  //sensor_msgs::JointState

  std::vector<std::string> mapping_poses_list_; // list of named poses for exploring the current local poses
  std::string mapping_motion_type_;  // LIN for linear motion between 2 poses, and PTP for joint motion between 2 poses

  bool enable_check_manipulation_arms_stowed_ = false;
  bool checkManipulationArmsStowed();


  bool moveToInitialMappingPose();
  bool moveToNextMappingPose(const std::string &next_pose);

  void setActivePercieverStatus(const ActivePercieverStatus& status);

  bool retrieveInterestedFruit(const uint &fruit_id);

};

} // namespace active_perception_planner

#endif // ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H
