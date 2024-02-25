#include <active_perception_planner/active_perciever.h>
#include <active_perception_planner/utils.h>
#include <deque>
#include <std_msgs/Int8.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "yaml-cpp/yaml.h"
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>

using namespace active_perception_planner;
using namespace utils;

ActivePerciever::ActivePerciever(ros::NodeHandle& nhp, const std::string& planning_group_name, const std::string& base_frame, const std::string& tip_frame, const bool disable_motion):status_(ActivePercieverStatus::IDLE), nhp_(nhp), planning_group_name_(planning_group_name), base_frame_(base_frame), tip_frame_(tip_frame), planner_id_("PTP"), plane_normal_(0, 0, 1), max_num_tries_(10),
  tf_buffer_(ros::Duration(30)),
  tf_listener_(tf_buffer_),
  disable_motion_(disable_motion)
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
  nhp_.param<double>("fruit_separation_z_diff", fruit_separation_z_diff_, 0.04);
  nhp_.param<double>("fruit_separation_xycenter_diff", fruit_separation_xycenter_diff_, 0.06);
  nhp_.param<std::vector<double>>("fixed_view_position", fixed_view_position_, {0.0508, -0.731, 1.45});

  vp_origin_ = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(fixed_view_position_.data(), fixed_view_position_.size());
  

  mapping_motion_type_ = "PTP";
  if(disable_motion_ == false)
  {
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
  }

  roi_detector_ = std::make_shared<pointcloud_roi::DetectedRoiFilter>(nhp_);
  peduncle_estimator_ = std::make_shared<active_perception_planner::PeduncleEstimator>();
  stem_estimator_     = std::make_shared<active_perception_planner::StemEstimator>();
  se_detector_ = std::make_shared<superellipsoid::SuperellipsoidDetector>();
  ik_controller_ = std::make_shared<active_perception_planner::DawnIKController>(nhp_);
  
  reset_map_client_ = nh_.serviceClient<std_srvs::Empty>("/gsm_node/reset_map");
  get_instanceclouds_client_ = nh_.serviceClient<vpp_msgs::GetListInstancePointclouds>("/gsm_node/get_list_instance_pointclouds");
  get_scenecloud_client_ = nh_.serviceClient<vpp_msgs::GetScenePointcloud>("/gsm_node/get_scene_pointcloud");

  timer_fruit_follower_   = nh_.createTimer(ros::Duration(0.2), &ActivePerciever::followFruitCallback, this);
  timer_status_publisher_ = nh_.createTimer(ros::Duration(0.2), &ActivePerciever::publishStatusTimerCallback, this);
  timer_fruit_estimator_  = nh_.createTimer(ros::Duration(0.2), &ActivePerciever::estimateFruitPoseFromCurrenDetectionsCallback, this);

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
    if (!reset_map_client_.call(empty_srv))
    {
      ROS_ERROR("Failed to call reset vpp map service");
      //return false;
    }
  }
  fruit_map_.clear();
  crop_.fruit_with_peduncle_map.clear();
  crop_.stems.clear();
  fruit_map_available_ = false;
  return true;
}

bool ActivePerciever::performLocalMappingMotion()
{

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
    sleep(0.5);
    processDetectionsAtSensorPose();

  }
  return true; //getActivePercieverStatus();
}

void ActivePerciever::processDetectionsAtSensorPose()
{
  //ROS_INFO("ddddddddddddddddddddd processDetectionsAtSensorPose ddddddddddddddddd");
  bool tracked_dets_available = false;
  roi_detector_->enableTrackedDetectionCallback(true);
  auto t_start = ros::Time::now();
  std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> detections = nullptr;
  //auto detections = roi_detector_->readSemanticInstanceDetections();
  do
  {
    //tracked_dets_available = roi_detector_->isTrackedDetectionAvailable();
    detections = roi_detector_->readSemanticInstanceDetections();
    sleep(0.2);
  }
  while(detections == nullptr && (ros::Time::now()-t_start).toSec() < 1.0);
  if(detections != nullptr)
  {
    ROS_WARN_STREAM("Tracked Detections available");
    //auto detections = roi_detector_->readSemanticInstanceDetections();
    peduncle_estimator_->semanticInstanceDetectionsCallback(detections);
    stem_estimator_->semanticInstanceDetectionsCallback(detections);
  }
  else
  {
    ROS_WARN_STREAM("Tracked Detections NOT available");
  }
}

bool ActivePerciever::computeFruitShapes(const std::vector<vpp_msgs::InstancePointcloudwithCentroid>& instance_clouds_with_centroid)
{
  bool atleast_one_completed_fruit = false;
  int num_fruits = 0;
  fruits_list_.clear();
  for(const auto& inst_element:instance_clouds_with_centroid)
  {
    ROS_INFO_STREAM("Instance Fruit size: "<<inst_element.num_points);
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
    fruits_list_.push_back(fruit);
    //fruit_map_.insert(std::pair<uint, superellipsoid::DetectedFruit>(fruit.fruit_id, fruit));
  }
  ROS_WARN_STREAM("Fruit Shape Completion success with "<<fruits_list_.size()<<" completed");
  return atleast_one_completed_fruit;
}

bool ActivePerciever::computeFruitShapes(const pointcloud_roi_msgs::PointCloudInstances& raw_segmented_clouds)
{
  bool atleast_one_completed_fruit = false;
  int num_fruits = 0;
  fruits_list_.clear();

  for(const auto& inst_element:raw_segmented_clouds.pointcloud_instances)
  {
    superellipsoid::DetectedFruit fruit;
    ROS_DEBUG("0");
    pcl::fromROSMsg(inst_element.cloud, *(fruit.observed_fruit_shape));
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
    fruits_list_.push_back(fruit);
      //fruit_map_.insert(std::pair<uint, superellipsoid::DetectedFruit>(fruit.fruit_id, fruit));
  }
  ROS_WARN_STREAM("Fruit Shape Completion success with "<<fruits_list_.size()<<" completed");
  return atleast_one_completed_fruit;
}

int ActivePerciever::removeFalsePositives()
{
  std::vector<size_t> indices_to_remove;
  std::vector<int> indices_to_add;

  for(size_t i = 0; i < fruits_list_.size(); ++i)
  {
    if((std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) && (std::find(indices_to_add.begin(), indices_to_add.end(), i) == indices_to_add.end()))
    {
      size_t bigger_fruit_index = i;
      auto ref_center     = fruits_list_[bigger_fruit_index].fruit_pose.position;
      auto ref_obs_points = fruits_list_[bigger_fruit_index].observed_fruit_shape->size();
      auto ref_fit_score  = fruits_list_[bigger_fruit_index].observation_score;
      for(size_t j = i+1;  j < fruits_list_.size(); ++j)
      {
        if((std::find(indices_to_remove.begin(), indices_to_remove.end(), j) == indices_to_remove.end()) && (std::find(indices_to_add.begin(), indices_to_add.end(), j) == indices_to_add.end()))
        {
          auto smaller_fruit_index = j;
          auto compared_center = fruits_list_[j].fruit_pose.position;
          double z_diff = abs(ref_center.z - compared_center.z);
          double y_diff = abs(ref_center.y - compared_center.y);
          double x_diff = abs(ref_center.x - compared_center.x);

          if(z_diff < fruit_separation_z_diff_)
          {
            if(sqrt(x_diff*x_diff + y_diff*y_diff) < fruit_separation_xycenter_diff_)
            {
              ROS_WARN("Fruits with centers close to each other");
              auto comp_obs_points = fruits_list_[j].observed_fruit_shape->size();
              auto comp_fit_score  = fruits_list_[j].observation_score;
              if(comp_fit_score > ref_fit_score)
              {
                smaller_fruit_index = bigger_fruit_index;
                bigger_fruit_index = j;
                ref_center = fruits_list_[bigger_fruit_index].fruit_pose.position;

                ref_fit_score = fruits_list_[bigger_fruit_index].observation_score;
                comp_fit_score  = fruits_list_[smaller_fruit_index].observation_score;
                ref_obs_points = fruits_list_[bigger_fruit_index].observed_fruit_shape->size();
                comp_obs_points = fruits_list_[smaller_fruit_index].observed_fruit_shape->size();
              }
              ROS_WARN_STREAM("Bigger  fruit: fit score: "<<fruits_list_[bigger_fruit_index].fit_score<<" obs score: "<<fruits_list_[bigger_fruit_index].observation_score<<" num_points: "<<ref_obs_points);
              ROS_WARN_STREAM("Smaller fruit: fit score: "<<fruits_list_[smaller_fruit_index].fit_score<<" obs score: "<<fruits_list_[smaller_fruit_index].observation_score<<" num_points: "<<comp_obs_points);
              indices_to_remove.push_back(smaller_fruit_index);
            }
          }
        }
      }
      indices_to_add.push_back(bigger_fruit_index);
      fruit_map_.insert(std::pair<uint, superellipsoid::DetectedFruit>(fruits_list_[bigger_fruit_index].fruit_id, fruits_list_[bigger_fruit_index]));
    }
  }
  ROS_WARN_STREAM("Fruit Generation success with "<<indices_to_add.size()<<" completed");
  return fruit_map_.size();
}

bool ActivePerciever::createSceneMapWithCompletedFruits(vpp_msgs::GetListInstancePointclouds& instcloud_srv)
{
  setActivePercieverStatus(FRUIT_MAP_GENERATION_IN_PROGRESS);
  fruit_map_.clear();
  sleep(0.5);
  // vpp_msgs::GetScenePointcloud get_scene_srv;
  // if(!get_scenecloud_client_.call(get_scene_srv))
  // {
  //   ROS_ERROR("Failed to call GetScenePointcloud vpp map service");
  //   setActivePercieverStatus(ERROR_MAP_GENERATION);
  //   return false;
  // }
  ROS_WARN("Calling GetListInstancePointclouds Service ...");
  instcloud_srv.request.get_specific_instances = false;
  if(!get_instanceclouds_client_.call(instcloud_srv))
  {
    ROS_ERROR("Failed to call GetListInstancePointclouds vpp map service");
    setActivePercieverStatus(ERROR_MAP_GENERATION);
    return false;
  }
  ROS_WARN("GetListInstancePointclouds Service Call Success ");
  if(instcloud_srv.response.instance_clouds_with_centroid.empty())
  {
    ROS_ERROR("Failed to detect fruits in the 3D map");
    setActivePercieverStatus(ERROR_NO_FRUITS_IN_MAP);
    return false;
  }
  ROS_DEBUG("3");
  if(computeFruitShapes(instcloud_srv.response.instance_clouds_with_centroid) == false)
  {
    setActivePercieverStatus(ERROR_SHAPE_COMPLETION);
    return false;
  }
  ROS_DEBUG("4");
  removeFalsePositives();
  fruit_map_available_ = true;
  setActivePercieverStatus(FRUIT_MAP_GENERATION_SUCCESS);
  ROS_INFO_STREAM("Creating peduncles");
  auto peduncle_map = peduncle_estimator_->getPeduncles();
  ROS_INFO_STREAM("Number of peduncles: "<<peduncle_map.size());
  auto stems = stem_estimator_->getStems();
  ROS_INFO_STREAM("Number of stems: "<<stems.size());
  crop_ = crop_estimator_->createCrop(fruit_map_, peduncle_map, stems);
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
   success = isFruitCentreSimilar(desired_fruit_centre, interested_fruit_.fruit.fruit_pose.position);
  }
  return success;
}

bool ActivePerciever::retrieveInterestedFruit(const uint &desired_fruit_id)
{
  FruitWithPeduncleMapIterator it = crop_.fruit_with_peduncle_map.find(desired_fruit_id);
  bool success = false;
  if(it != crop_.fruit_with_peduncle_map.end())
  {
    interested_fruit_ = it->second;
    return true;
  }
  return false;
}

bool ActivePerciever::findOptimumFruitObservationPoses(std::deque<geometry_msgs::Pose>& optimum_poses)
{

  // Eigen::Vector3d ray_origin, ray_pt;
  // ray_origin(0) = interested_fruit_.fruit_pose.position.x;
  // ray_origin(1) = interested_fruit_.fruit_pose.position.y;
  // ray_origin(2) = interested_fruit_.fruit_pose.position.z;

  // ray_pt(0) = interested_fruit_.observed_shape_centroid.x;
  // ray_pt(1) = interested_fruit_.observed_shape_centroid.y;
  // ray_pt(2) = interested_fruit_.observed_shape_centroid.z;

  // bool success = false;
  // optimum_poses.clear();
  // for(int num_tries = 0; num_tries < max_num_tries_; num_tries++)
  // {
  //   Eigen::Vector3d inter_pt;
  //   plane_point_.setZero();
  //   plane_point_(2) = obs_z_plane_;
  //   if(calcLinePlaneIntersection(inter_pt, ray_origin, ray_pt, plane_normal_, plane_point_))
  //   {
  //     if(map_->getVoxelState(inter_pt) == VoxelState::FREE)
  //     {
  //       success = true;
  //       auto view_pose = fromEigen2GeometryMsg(inter_pt, ray_pt-ray_origin, view_dir_, cam_quat_);
  //       optimum_poses.push_back(view_pose);
  //     }
  //   }
  //   plane_point_(2) = obs_z_plane_ + voxel_factor_*map_->getVoxelSize();
  // }
  return true;
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
  if(retrieveInterestedFruit(fruit_id))
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
    if(timer_fruit_estimator_.hasStarted() == false)
    {
      timer_fruit_estimator_.start();
    }
    return true;
  }
  return false;
}

bool ActivePerciever::stopFollowFruit()
{
  if(timer_fruit_follower_.hasStarted() == true)
    timer_fruit_follower_.stop();
  if(timer_fruit_estimator_.hasStarted() == true)  
    timer_fruit_estimator_.stop();
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
  auto fruit_with_peduncle = getCurrentEstimateOfTheInterestedFruit();
  
  if(initial_motion_done_ == false)
  {
    ik_controller_->stopDawnIK();
    if(use_fixed_pose_)
    {
      bool initial_motion_success = false;
      bool compute_view_pose_success = false;
      geometry_msgs::Pose view_pose;
      compute_view_pose_success = computeInitialFixedViewPose(view_pose, fruit_with_peduncle.fruit.fruit_pose.position);
      if(compute_view_pose_success)
      {
        initial_motion_success = moveToPose(view_pose);
      }
      initial_motion_done_ = true;
      if(initial_motion_success == false)
      {
        ROS_ERROR_STREAM("Cannot move to initial pose using motion planning. Using Lookat Pose directly");
        if(ik_controller_->startDawnIK())
        {
          if(compute_view_pose_success)
          {
            if(ik_controller_->fullPoseTarget(view_pose))
              return;
          }
        }
      }
      else
      {
        ROS_INFO_STREAM("Success! Moved to Initial Fixed View Pose using moveit");
        return; 
      }
    }
  }
  else
  {
    if(use_fixed_pose_)
    {
      if(!ik_controller_->isIKControllerEnabled())
        ik_controller_->startDawnIK();

      Eigen::Vector3d fruit_target_eigen;
      tf2::fromMsg(fruit_with_peduncle.fruit.fruit_pose.position, fruit_target_eigen);
      ik_controller_->lookAtTarget(vp_origin_, fruit_target_eigen); 
    }
    else
    {

    }
  }
  // auto fruit= getasRosMsgType(interested_fruit_);
  // pub_interested_fruit_.publish(fruit);
}

void ActivePerciever::estimateFruitPoseFromCurrenDetectionsCallback(const ros::TimerEvent& e)
{

}
bool ActivePerciever::isDetectedFruitCurrentInterestedOne(const DetectedFruitWithPeduncle& prev_det, DetectedFruitWithPeduncle& potential_candidate, double center_threshold)
{
  return isFruitCentreSimilar(prev_det.fruit.fruit_pose.position, potential_candidate.fruit.fruit_pose.position, center_threshold);
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
    DetectedFruitWithPeduncle possible_same_fruit;
    if(se_detector_->completeFruitShape(cluster, possible_same_fruit.fruit) == false)
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

// oc2_interface_msgs::DetectedFruitOld ActivePerciever::getasRosMsgType(const superellipsoid::DetectedFruit& fruit)
// {
//   oc2_interface_msgs::DetectedFruitOld fruit_msg;

//   fruit_msg.fruit_id = fruit.fruit_id;
//   pcl::toROSMsg(*fruit.observed_fruit_shape, fruit_msg.observed_fruit);
//   pcl::toROSMsg(*fruit.completed_fruit_shape, fruit_msg.predicted_fruit);
//   fruit_msg.fruit_pose = fruit.fruit_pose;

//   return fruit_msg;
// }

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

  //pub_interested_fruit_.publish(getasRosMsgType(getCurrentEstimateOfTheInterestedFruit()));
}

void ActivePerciever::createCrop()
{
  auto stems = stem_estimator_->getStems();
  auto peduncles = peduncle_estimator_->getPeduncles();

  crop_ = crop_estimator_->createCrop(fruit_map_, peduncles, stems);
}

bool ActivePerciever::getPose(geometry_msgs::TransformStamped& transform, const std::string& target_frame, const std::string& original_frame)
{
  try
  {
    transform = tf_buffer_.lookupTransform(target_frame, original_frame, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform from "<<original_frame<<" to "<< target_frame<<" : " << e.what());
    return false;
  }
  return true;
}

bool ActivePerciever::computeInitialFixedViewPose(geometry_msgs::Pose& view_pose, const geometry_msgs::Point& fruit_target)
{
  geometry_msgs::TransformStamped camera_tf;
  if(getPose(camera_tf, "zed2i_camera_left_optical_frame", "world")== false)
  {
    return false;
  }
  tf2::Quaternion cam_quat;
  tf2::fromMsg(camera_tf.transform.rotation, cam_quat);

  Eigen::Vector3d fruit_target_eigen;
  tf2::fromMsg(fruit_target, fruit_target_eigen);

  view_pose = computeViewPose(fruit_target_eigen, vp_origin_, cam_quat);
  return true;
}

geometry_msgs::Pose ActivePerciever::computeViewPose(const Eigen::Vector3d& fruit_target, const Eigen::Vector3d& vp_origin, const tf2::Quaternion& cam_quat)
{
  Eigen::Vector3d dir_vec = (fruit_target - vp_origin).normalized();
  const tf2::Matrix3x3 cam_mat(cam_quat);
  tf2::Vector3 view_dir(cam_mat.getColumn(2));

  auto view_quat = utils::dirVecToQuat(dir_vec, cam_quat,view_dir);
  geometry_msgs::Pose view_pose;

  tf::pointEigenToMsg(vp_origin, view_pose.position);
  view_pose.orientation = tf2::toMsg(view_quat);

  return view_pose;
}

bool ActivePerciever::moveToPose(const geometry_msgs::Pose& view_pose)
{
  motion_planner_->getMoveGroupHandle()->setPlannerId(planner_id_);
  ROS_WARN_STREAM("mapping_motion_type_"<<planner_id_);
  return motion_planner_->movePoseTarget(view_pose, planner_id_);
}