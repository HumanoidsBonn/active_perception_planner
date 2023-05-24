#include "active_perception_planner/active_perciever.h"


using namespace active_perception_planner;

ActivePerciever::ActivePerciever(ros::NodeHandle& nhp, const std::string& planning_group_name, const std::string& base_frame, const std::string& tip_frame):status_(ActivePercieverStatus::IDLE), nhp_(nhp), planning_group_name_(planning_group_name), base_frame_(base_frame), tip_frame_(tip_frame), motion_type_("PTP")
{
  nhp_.param<std::string>("moveit_planner_type", moveit_planner_type_, "pilz::CommandPlanner");
  nhp_.param<int>("num_planning_attempts", num_planning_attempts_, 2);
  nhp_.param<float>("allowed_planning_time", allowed_planning_time_, 5.0);

  motion_planner_ = std::make_shared<motion_planning::MotionPlanner>(planning_group_name_, nh_);
  motion_planner_->initialisePlanner(moveit_planner_type_, motion_type_, planning_group_name_);
  motion_planner_->initialiseMotionPlanRequest(motion_type_, planning_group_name_, num_planning_attempts_, allowed_planning_time_);
  reset_map_client_ = nh_.serviceClient<std_srvs::Empty>("/reset_map");
  get_instanceclouds_client_ = nh_.serviceClient<vpp_msgs::GetListInstancePointclouds>("get_list_instance_pointclouds");
  get_scenecloud_client_ = nh_.serviceClient<vpp_msgs::GetScenePointcloud>("get_scene_pointcloud");
}

ActivePercieverStatus ActivePerciever::getActivePercieverStatus()
{
  return status_;
}

bool ActivePerciever::clearSceneMap()
{
  fruit_map_.clear();
  std_srvs::Empty empty_srv;
  if (!reset_map_client_.call(empty_srv))
  {
      ROS_ERROR("Failed to call reset vpp map service");
  }
}

ActivePercieverStatus ActivePerciever::performLocalMappingMotion()
{
  if(checkManipulationArmsStowed() == false)
  {
    setActivePercieverStatus(ERROR_MANIP_ARMS_NOT_STOWED);
    return getActivePercieverStatus();
  }

  if(moveToInitialMappingPose() == false)
  {
    setActivePercieverStatus(ERROR_MOVE_TO_INIT_MAPPING_POSE);
    return getActivePercieverStatus();
  }

  for(const auto& next_pose: mapping_poses_list_)
  {
    if(moveToNextMappingPose(next_pose)== false)
    {
      setActivePercieverStatus(ERROR_LOCAL_MAPPING_MOTION);
      return getActivePercieverStatus();
    }
  }
  return getActivePercieverStatus();
}

bool ActivePerciever::createSceneMapWithCompletedFruits()
{
  setActivePercieverStatus(FRUIT_MAP_GENERATION_IN_PROGRESS);
  fruit_map_.clear();
  vpp_msgs::GetScenePointcloud get_scene_srv;
  if(!get_scenecloud_client_.call(get_scene_srv))
  {
    ROS_ERROR("Failed to call GetScenePointcloud vpp map service");
    setActivePercieverStatus(ERROR_MAP_GENERATION);
    return false;
  }

  vpp_msgs::GetListInstancePointclouds instcloud_srv;
  instcloud_srv.request.get_specific_instances = false;
  if(!get_instanceclouds_client_.call(instcloud_srv))
  {
    ROS_ERROR("Failed to call GetListInstancePointclouds vpp map service");
    setActivePercieverStatus(ERROR_MAP_GENERATION);
    return false;
  }
  if(instcloud_srv.response.instance_clouds_with_centroid.empty())
  {
    ROS_ERROR("Failed to detect fruits in the 3D map");
    setActivePercieverStatus(ERROR_NO_FRUITS_IN_MAP);
    return false;
  }
  bool atleast_one_completed_fruit = false;
  for(const auto& inst_element:instcloud_srv.response.instance_clouds_with_centroid)
  {
    superellipsoid::DetectedFruit fruit;
    pcl::fromROSMsg(inst_element.pointcloud, *(fruit.observed_fruit_shape));
    fruit.fruit_id = inst_element.instance_id;
    bool success = se_detector_->completeFruitShape(fruit);
    if(success)
      atleast_one_completed_fruit = true;
    fruit_map_.insert(std::pair<uint, superellipsoid::DetectedFruit>(fruit.fruit_id, fruit));
  }
  if(atleast_one_completed_fruit == false)
  {
    setActivePercieverStatus(ERROR_SHAPE_COMPLETION);
    return false;
  }
  setActivePercieverStatus(FRUIT_MAP_GENERATION_SUCCESS);
  return true;
}

bool ActivePerciever::checkManipulationArmsStowed()
{
  if(enable_check_manipulation_arms_stowed_ == false)
    return true;

  return false;
}

bool ActivePerciever::moveToInitialMappingPose()
{
  motion_planner_->clearMotionPlanRequestConstraints();
  return motion_planner_->moveNamedTarget("initial_mapping_pose", motion_type_);

}

bool ActivePerciever::moveToNextMappingPose(const std::string& next_pose)
{
  motion_planner_->clearMotionPlanRequestConstraints();
  bool success = motion_planner_->moveNamedTarget(next_pose, mapping_motion_type_);
  if(success == false && mapping_motion_type_ == "LIN" && attempt_with_ptp_on_failure_ == true)
  {
    return motion_planner_->moveNamedTarget(next_pose, motion_type_);
  }
  return success;
}

bool ActivePerciever::retrieveInterestedFruit(const uint &fruit_id)
{
  FruitMapIterator it = fruit_map_.find(fruit_id);
  if(it == fruit_map_.end())
  {
    return false;
  }
  interested_fruit_ = it->second;
  return true;
}
bool ActivePerciever::moveToInterestedFruitObservationPose(const uint &fruit_id)
{


}



