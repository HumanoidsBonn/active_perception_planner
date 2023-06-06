#include <active_perception_planner/active_perciever.h>
#include <active_perception_planner/utils.h>
#include <deque>
#include <std_msgs/Int8.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace active_perception_planner;

ActivePerciever::ActivePerciever(ros::NodeHandle& nhp, const std::string& planning_group_name, const std::string& base_frame, const std::string& tip_frame):status_(ActivePercieverStatus::IDLE), nhp_(nhp), planning_group_name_(planning_group_name), base_frame_(base_frame), tip_frame_(tip_frame), motion_type_("PTP"), plane_normal_(0, 0, 1), max_num_tries_(10),
  tf_buffer_(ros::Duration(30)),
  tf_listener_(tf_buffer_)
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

  timer_fruit_follower_ = nh_.createTimer(ros::Duration(0.2), &ActivePerciever::followFruitCallback, this);
  timer_status_publisher_ = nh_.createTimer(ros::Duration(0.2), &ActivePerciever::publishStatusTimerCallback, this);

  pub_status_advertiser_ = nhp_.advertise<std_msgs::Int8>("active_perciever_status", 1, true);

}

ActivePercieverStatus ActivePerciever::getActivePercieverStatus()
{
  return status_;
}

bool ActivePerciever::clearSceneMap()
{
  fruit_map_.clear();
  fruit_map_available_ = false;
  std_srvs::Empty empty_srv;
  if (!reset_map_client_.call(empty_srv))
  {
      ROS_ERROR("Failed to call reset vpp map service");
      return false;
  }
  return true;
}

bool ActivePerciever::performLocalMappingMotion()
{
  if(checkManipulationArmsStowed() == false)
  {
    setActivePercieverStatus(ERROR_MANIP_ARMS_NOT_STOWED);
    return false; //getActivePercieverStatus();
  }

  if(moveToInitialMappingPose() == false)
  {
    setActivePercieverStatus(ERROR_MOVE_TO_INIT_MAPPING_POSE);
    return false; //getActivePercieverStatus();
  }

  for(const auto& next_pose: mapping_poses_list_)
  {
    if(moveToNextMappingPose(next_pose)== false)
    {
      setActivePercieverStatus(ERROR_LOCAL_MAPPING_MOTION);
      return false; //getActivePercieverStatus();
    }
  }
  return true; //getActivePercieverStatus();
}

bool ActivePerciever::createSceneMapWithCompletedFruits(vpp_msgs::GetListInstancePointclouds& instcloud_srv)
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
  fruit_map_available_ = true;
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

bool ActivePerciever::retrieveInterestedFruit(const uint &desired_fruit_id, const geometry_msgs::Point& desired_fruit_centre)
{
  FruitMapIterator it = fruit_map_.find(desired_fruit_id);
  bool success = false;
  if(it != fruit_map_.end())
  {
   interested_fruit_ = it->second;
   success = isFruitCentreSimilar(desired_fruit_centre, interested_fruit_.fruit_centre);
  }
  return success;
}

bool ActivePerciever::retrieveInterestedFruit(const uint &desired_fruit_id)
{
  FruitMapIterator it = fruit_map_.find(desired_fruit_id);
  bool success = false;
  if(it != fruit_map_.end())
  {
   interested_fruit_ = it->second;
   return true;
  }
  return false;
}

bool ActivePerciever::findOptimumFruitObservationPoses(std::deque<geometry_msgs::Pose>& optimum_poses)
{
  Eigen::Vector3d ray_origin, ray_pt;
  ray_origin(0) = interested_fruit_.fruit_centre.x;
  ray_origin(1) = interested_fruit_.fruit_centre.y;
  ray_origin(2) = interested_fruit_.fruit_centre.z;

  ray_pt(0) = interested_fruit_.observed_shape_centroid.x;
  ray_pt(1) = interested_fruit_.observed_shape_centroid.y;
  ray_pt(2) = interested_fruit_.observed_shape_centroid.z;

  bool success = false;
  optimum_poses.clear();
  for(int num_tries = 0; num_tries < max_num_tries_; num_tries++)
  {
    Eigen::Vector3d inter_pt;
    plane_point_.setZero();
    plane_point_(2) = obs_z_plane_;
    if(calcLinePlaneIntersection(inter_pt, ray_origin, ray_pt, plane_normal_, plane_point_))
    {
      if(map_->getVoxelState(inter_pt) == VoxelState::FREE)
      {
        success = true;
        auto view_pose = fromEigen2GeometryMsg(inter_pt, ray_pt-ray_origin, view_dir_, cam_quat_);
        optimum_poses.push_back(view_pose);
      }
    }
    plane_point_(2) = obs_z_plane_ + voxel_factor_*map_->getVoxelSize();
  }
  return success;
}

bool ActivePerciever::moveToInterestedFruitObservationPose(const uint &fruit_id, const geometry_msgs::Point& fruit_centre)
{
  if(retrieveInterestedFruit(fruit_id, fruit_centre) == false)
  {
    ROS_WARN_STREAM("Could not retrive fruit with fruit id "<<int(fruit_id));
    return false;
  }
  //std::vector<geometry_msgs::Pose> view_poses;
  std::deque<geometry_msgs::Pose> view_poses;
  if(findOptimumFruitObservationPoses(view_poses) == false)
  {
    ROS_WARN_STREAM("Could not find even a single optimum fruit observation pose");
    return false;
  }

  bool move_success = false;
  while(move_success == false && !view_poses.empty())
  {
    geometry_msgs::Pose view_pose = view_poses.front();
    move_success = motion_planner_->movePoseTarget(view_pose, "PTP");
    view_poses.pop_front();
  }
  return move_success;
}

bool ActivePerciever::startFollowFruit(const uint &fruit_id)
{
  if(retrieveInterestedFruit(fruit_id))
  return true;
}

bool ActivePerciever::stopFollowFruit()
{
  timer_fruit_follower_.stop();
  while(timer_fruit_follower_.hasStarted() == false)
  {
    sleep(0.5);
  }
  return true;
}

void ActivePerciever::publishStatusTimerCallback(const ros::TimerEvent&)
{
  std_msgs::Int8 status_msg;
  status_msg.data = status_atomic_.load();
  pub_status_advertiser_.publish(status_msg);
}


void ActivePerciever::followFruitCallback(const ros::TimerEvent&)
{

}


bool ActivePerciever::isDetectedFruitCurrentInterestedOne(const superellipsoid::DetectedFruit& prev_det, superellipsoid::DetectedFruit& potential_candidate, double center_threshold)
{
  return isFruitCentreSimilar(prev_det.fruit_centre, potential_candidate.fruit_centre, center_threshold);
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ActivePerciever::getClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl)
{
  // Filter NaN Values
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl, *pc_pcl, *indices);

  if (pc_pcl->size() < min_cluster_size_)
  {
    ROS_ERROR("pc_pcl->size() < p_min_cluster_size");
    clusters.clear();
    return clusters;
  }
  pcl_ros::transformPointCloud(world_frame_, *pc_pcl, *pc_pcl, tf_listener_);
  std_msgs::Header pc_pcl_tf_ros_header = pcl_conversions::fromPCL(pc_pcl->header);

  // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  clusters = clustering::euclideanClusterExtraction(pc_pcl, cluster_indices, cluster_tolerance_, min_cluster_size_, max_cluster_size_);

  return clusters;
}

bool ActivePerciever::updateInterestedFruit(const sensor_msgs::PointCloud2 &surface_cloud)
{
  // Convert to PCL Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(surface_cloud, *pc_pcl);

  auto clusters = getClusters(pc_pcl);

  if(clusters.empty())
    return false;

  for(auto& cluster: clusters)
  {
    superellipsoid::DetectedFruit possible_same_fruit_;
    if(se_detector_->completeFruitShape(cluster, possible_same_fruit_) == false)
      continue;

    if(isDetectedFruitCurrentInterestedOne(prev_interested_fruit_detection_, possible_same_fruit_))
    {
      interested_fruit_detections_buffer_.push_back(possible_same_fruit_);
      interested_fruit_detections_buffer_.pop_back();
      prev_interested_fruit_detection_ = possible_same_fruit_;
      return true;
    }
  }

}

