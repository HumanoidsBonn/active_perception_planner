#include <active_perception_planner/active_perciever.h>
#include <active_perception_planner/utils.h>
#include <deque>
#include <std_msgs/Int8.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "yaml-cpp/yaml.h"


using namespace active_perception_planner;
using namespace utils;

ActivePerciever::ActivePerciever(ros::NodeHandle& nhp, const std::string& planning_group_name, const std::string& base_frame, const std::string& tip_frame):status_(ActivePercieverStatus::IDLE), nhp_(nhp), planning_group_name_(planning_group_name), base_frame_(base_frame), tip_frame_(tip_frame), planner_id_("PTP"), plane_normal_(0, 0, 1), max_num_tries_(10),
  tf_buffer_(ros::Duration(30)),
  tf_listener_(tf_buffer_)
{
  nhp_.param<bool>("move_group_interface_enabled", is_move_group_used_, true);
  nhp_.param<std::string>("moveit_planner_type", moveit_planner_type_, "pilz_industrial_motion_planner");
  nhp_.param<std::string>("planner_id", planner_id_, "PTP");
  nhp_.param<int>("num_planning_attempts", num_planning_attempts_, 2);
  nhp_.param<float>("allowed_planning_time", allowed_planning_time_, 5.0);
  nhp_.param<std::string>("planning_group_name", planning_group_name_, "head");
  nhp_.param<std::string>("base_frame", base_frame_, "trolley_local_link");
  nhp_.param<std::string>("tip_frame", tip_frame_, "camera_color_optical_frame");
  nhp_.param<std::string>("surface_cloud_topic", surface_cloud_topic_, "/voxblox_node/surface_cloud");
  nhp_.param<float>("obs_z_plane", obs_z_plane_, 1.0);
  nhp_.param<std::vector<std::string>>("pose_list", mapping_poses_list_, {"init_pose",  "obs_pose_202", "obs_pose_303", "obs_pose_404"});

  mapping_motion_type_ = "PTP";
  if(is_move_group_used_)
  {
    ROS_INFO("MOVE_GROUP PLANNER");
    auto base_ptr = std::make_shared<motion_planning::MoveGroupPlanner>(planning_group_name, nh_);
    motion_planner_ = std::static_pointer_cast<motion_planning::MotionPlanner>(base_ptr);

    std::vector<moveit_msgs::PlannerInterfaceDescription> inter_desc_vec;
    bool check = motion_planner_->getMoveGroupHandle()->getInterfaceDescriptions(inter_desc_vec);
    for (const auto& inter_desc: inter_desc_vec)
    {
      ROS_INFO_STREAM("Planner Interfaces: "<<inter_desc); 
    }
    motion_planner_->getMoveGroupHandle()->setPlanningPipelineId("pilz_industrial_motion_planner");
    motion_planner_->getMoveGroupHandle()->setPlannerId(planner_id_);

  }
  else
  {
    ROS_INFO("Pilz Planner");
    motion_planner_ = std::make_shared<motion_planning::MotionPlanner>(planning_group_name_, nh_);
    motion_planner_->initialisePlanner(moveit_planner_type_, planner_id_, planning_group_name_);
    motion_planner_->initialiseMotionPlanRequest(planner_id_, planning_group_name_, num_planning_attempts_, allowed_planning_time_);
  }

  se_detector_ = std::make_shared<superellipsoid::SuperellipsoidDetector>();
  
  reset_map_client_ = nh_.serviceClient<std_srvs::Empty>("/gsm_node/reset_map");
  get_instanceclouds_client_ = nh_.serviceClient<vpp_msgs::GetListInstancePointclouds>("/gsm_node/get_list_instance_pointclouds");
  get_scenecloud_client_ = nh_.serviceClient<vpp_msgs::GetScenePointcloud>("/gsm_node/get_scene_pointcloud");

  timer_fruit_follower_ = nh_.createTimer(ros::Duration(0.2), &ActivePerciever::followFruitCallback, this);
  timer_status_publisher_ = nh_.createTimer(ros::Duration(0.2), &ActivePerciever::publishStatusTimerCallback, this);

  sub_surface_cloud_ =  nh_.subscribe(surface_cloud_topic_, 1, &ActivePerciever::receiveSurfacePointCloudCallback, this);

  pub_status_advertiser_ = nhp_.advertise<std_msgs::Int8>("status", 1, true);

  pub_interested_fruit_ = nhp_.advertise<oc2_interface_msgs::DetectedFruit>("interested_fruit", 1, true);

  listener_ = std::make_unique<tf::TransformListener>();

  timer_status_publisher_.start();
  ROS_INFO("ActivePerciever");
}

ActivePercieverStatus ActivePerciever::getActivePercieverStatus()
{
  return status_;
}

bool ActivePerciever::clearSceneMap()
{
  std_srvs::Empty empty_srv;
  if(fruit_map_available_)
  {
    // if (!reset_map_client_.call(empty_srv))
    // {
    //     ROS_ERROR("Failed to call reset vpp map service");
    //     //return false;
    // }
  }
  fruit_map_.clear();
  fruit_map_available_ = false;
  return true;
}

bool ActivePerciever::performLocalMappingMotion()
{
//  auto config_package_path_ = ros::package::getPath("active_perception_planner");
//  std::string config_filepath = config_package_path_ + "/config/" + "planner.yaml";

//  YAML::Node config_yaml = YAML::LoadFile(config_filepath);

//  bool poses_list_found = false;
//  int i = 0;
//  for (YAML::const_iterator it = config_yaml.begin(); it != config_yaml.end(); it++)
//  {
//    ROS_INFO_STREAM("YAML Node No. "<<i<<"\t Name: "<<it->first.as<std::string>());
//    i++;
//    YAML::Node configs = it->second;
//    if(auto it1->first.as<std::string>() == "pose_list")
//    {
//      poses_list_found = true;
//      mapping_poses_list_ = it->second.as<std::vector<std::string>>();
//      ROS_INFO_STREAM("Mapping poses list: "<<mapping_poses_list_[0]);
//    }
//  }
//  if(!poses_list_found)
//  {
//    ROS_ERROR("Poses list not found in planner.yaml");
//  }

  if(checkManipulationArmsStowed() == false)
  {
    setActivePercieverStatus(ERROR_MANIP_ARMS_NOT_STOWED);
    return false; //getActivePercieverStatus();
  }

  // if(moveToInitialMappingPose() == false)
  // {
  //   setActivePercieverStatus(ERROR_MOVE_TO_INIT_MAPPING_POSE);
  //   return false; //getActivePercieverStatus();
  // }

  for(const auto& next_pose: mapping_poses_list_)
  {
    if(moveToNextMappingPose(next_pose)== false)
    {
      setActivePercieverStatus(ERROR_LOCAL_MAPPING_MOTION);
      return false; //getActivePercieverStatus();
    }
    sleep(3.0);
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
  ROS_WARN("GetScenePointcloud success ");
  instcloud_srv.request.get_specific_instances = false;
  if(!get_instanceclouds_client_.call(instcloud_srv))
  {
    ROS_ERROR("Failed to call GetListInstancePointclouds vpp map service");
    setActivePercieverStatus(ERROR_MAP_GENERATION);
    return false;
  }
  ROS_WARN("GetListInstancePointclouds success ");
  if(instcloud_srv.response.instance_clouds_with_centroid.empty())
  {
    ROS_ERROR("Failed to detect fruits in the 3D map");
    setActivePercieverStatus(ERROR_NO_FRUITS_IN_MAP);
    return false;
  }
  bool atleast_one_completed_fruit = false;
  int num_fruits = 0;
  for(const auto& inst_element:instcloud_srv.response.instance_clouds_with_centroid)
  {
    superellipsoid::DetectedFruit fruit;
    ROS_DEBUG("0");
    pcl::fromROSMsg(inst_element.pointcloud, *(fruit.observed_fruit_shape));
    fruit.fruit_id = inst_element.instance_id;
    ROS_DEBUG("1");
    bool success = se_detector_->completeFruitShape(fruit);
    if(success)
    {
      num_fruits++;
      ROS_INFO_STREAM("Fruit with successful shape completion: "<<fruit.fruit_id);
      atleast_one_completed_fruit = true;
    }
    else
    {
      ROS_WARN_STREAM("Fruit with failed shape completion: "<<fruit.fruit_id);
    }
    fruit_map_.insert(std::pair<uint, superellipsoid::DetectedFruit>(fruit.fruit_id, fruit));
  }
  ROS_DEBUG("3");
  if(atleast_one_completed_fruit == false)
  {
    setActivePercieverStatus(ERROR_SHAPE_COMPLETION);
    return false;
  }
  ROS_DEBUG("4");
  fruit_map_available_ = true;
  setActivePercieverStatus(FRUIT_MAP_GENERATION_SUCCESS);
  ROS_WARN_STREAM("Fruit Generation success with "<<num_fruits<<" completed");
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
  if(is_move_group_used_ == false)
    motion_planner_->clearMotionPlanRequestConstraints();
  return motion_planner_->moveNamedTarget("transport_pose", planner_id_);

}

bool ActivePerciever::moveToNextMappingPose(const std::string& next_pose)
{
  if(is_move_group_used_ == false)
    motion_planner_->clearMotionPlanRequestConstraints();
  
  ROS_WARN_STREAM("mapping_motion_type_"<<mapping_motion_type_);
  bool success = motion_planner_->moveNamedTarget(next_pose, mapping_motion_type_);
  if(success == false && mapping_motion_type_ == "LIN" && attempt_with_ptp_on_failure_ == true)
  {
    ROS_WARN_STREAM("mapping_motion_type_"<<planner_id_);
    return motion_planner_->moveNamedTarget(next_pose, planner_id_);
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
   success = isFruitCentreSimilar(desired_fruit_centre, interested_fruit_.fruit_pose.position);
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
  ray_origin(0) = interested_fruit_.fruit_pose.position.x;
  ray_origin(1) = interested_fruit_.fruit_pose.position.y;
  ray_origin(2) = interested_fruit_.fruit_pose.position.z;

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

bool ActivePerciever::moveToInterestedFruitObservationPose(const uint &fruit_id)
{
  if(retrieveInterestedFruit(fruit_id) == false)
  {
    ROS_WARN_STREAM("Could not retrieve fruit with fruit id "<<int(fruit_id));
    return false;
  }
  return true;
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
  if(moveToInterestedFruitObservationPose(fruit_id))
  {
    if(fruit_est_initialised_ == false)
    {
      setCurrentEstimateOfTheInterestedFruit(interested_fruit_);
      fruit_est_initialised_ = true;
    }
    follow_fruit_enabled_ = true;

    if(fruit_est_initialised_ && timer_fruit_follower_.hasStarted() == false)
    {
      timer_fruit_follower_.start();
    }
    return true;
  }
  return false;
}

bool ActivePerciever::stopFollowFruit()
{
  timer_fruit_follower_.stop();
//  while(timer_fruit_follower_.hasStarted() == false)
//  {
//    sleep(0.5);
//  }
  follow_fruit_enabled_ = false;
  interested_fruit_ = superellipsoid::DetectedFruit();
  fruit_est_initialised_ = false;
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
  auto fruit= getasRosMsgType(interested_fruit_);
  pub_interested_fruit_.publish(fruit);
}

bool ActivePerciever::isDetectedFruitCurrentInterestedOne(const superellipsoid::DetectedFruit& prev_det, superellipsoid::DetectedFruit& potential_candidate, double center_threshold)
{
  return isFruitCentreSimilar(prev_det.fruit_pose.position, potential_candidate.fruit_pose.position, center_threshold);
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
  //pcl_ros::transformPointCloud(world_frame_, *pc_pcl, *pc_pcl, tf_listener_);
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
  prev_est_interested_fruit_ = curr_est_interested_fruit_;
  for(auto& cluster: clusters)
  {
    superellipsoid::DetectedFruit possible_same_fruit;
    if(se_detector_->completeFruitShape(cluster, possible_same_fruit) == false)
      continue;

    if(isDetectedFruitCurrentInterestedOne(prev_est_interested_fruit_, possible_same_fruit))
    {
      //interested_fruit_detections_buffer_.push_back(possible_same_fruit);
      //interested_fruit_detections_buffer_.pop_back();
      setCurrentEstimateOfTheInterestedFruit(possible_same_fruit);
      return true;
    }
  }
  ROS_ERROR("Could not detect fruit in the curr surface cloud");
  return false;
}

oc2_interface_msgs::DetectedFruit ActivePerciever::getasRosMsgType(const superellipsoid::DetectedFruit& fruit)
{
  oc2_interface_msgs::DetectedFruit fruit_msg;

  fruit_msg.fruit_id = fruit.fruit_id;
  pcl::toROSMsg(*fruit.observed_fruit_shape, fruit_msg.observed_fruit);
  pcl::toROSMsg(*fruit.completed_fruit_shape, fruit_msg.predicted_fruit);
  fruit_msg.fruit_pose = fruit.fruit_pose;

  return fruit_msg;
}

oc2_interface_msgs::InterestedFruit ActivePerciever:: getInterestedFruitAsRosMsgType(const superellipsoid::DetectedFruit& fruit)
{
  oc2_interface_msgs::InterestedFruit fruit_msg;

  pcl::toROSMsg(*fruit.observed_fruit_shape, fruit_msg.observed_fruit);
  pcl::toROSMsg(*fruit.completed_fruit_shape, fruit_msg.predicted_fruit);
  fruit_msg.fruit_pose = fruit.fruit_pose;

  fruit_msg.header.frame_id = world_frame_;
  fruit_msg.header.stamp = ros::Time::now();
  return fruit_msg;
}

void ActivePerciever::receiveSurfacePointCloudCallback(const sensor_msgs::PointCloud2 &pc_msg)
{
  if(follow_fruit_enabled_ == false)
    return;

  if(updateInterestedFruit(pc_msg) == false)
    return;

  pub_interested_fruit_.publish(getasRosMsgType(getCurrentEstimateOfTheInterestedFruit()));
}
