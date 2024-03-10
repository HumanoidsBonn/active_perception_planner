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
#include <std_srvs/Empty.h>
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
  nhp_.param<std::string>("zed2i_cloud", surface_cloud_topic_, "/zed2i/point_cloud/cloud_registered");
  nhp_.param<float>("obs_z_plane", obs_z_plane_, 1.0);
  nhp_.param<std::vector<std::string>>("pose_list", mapping_poses_list_, {"init_pose",  "obs_pose_202", "obs_pose_303", "obs_pose_404"});
  nhp_.param<double>("fruit_separation_z_diff", fruit_separation_z_diff_, 0.04);
  nhp_.param<double>("fruit_separation_xycenter_diff", fruit_separation_xycenter_diff_, 0.06);
  nhp_.param<std::vector<double>>("fixed_view_position", fixed_view_position_, {0.7, 0.3, 0.84});
  nhp_.param<bool>("use_fixed_pose", use_fixed_pose_, true);

  client_start_gsm_node_    = nhp_.serviceClient<std_srvs::Empty>("/gsm_node/enable_gsm_srv");
  client_stop_gsm_node_     = nhp_.serviceClient<std_srvs::Empty>("/gsm_node/disable_gsm_srv");

  pub_control_gsm_          = nhp_.advertise<std_msgs::Bool>("/gsm_node/control", 1);

  vp_origin_ = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(fixed_view_position_.data(), fixed_view_position_.size());
  view_positions_.push_back(vp_origin_);
  Eigen::Vector3d view_left = vp_origin_;
  view_left.x() = vp_origin_.x() - 0.05;
  view_positions_.push_back(view_left);

  view_left.z() = view_left.z() + 0.05;
  view_positions_.push_back(view_left);

  view_left.x() = view_left.x() - 0.05;
  view_left.y() = view_left.y() + 0.05; 
  view_positions_.push_back(view_left);

  Eigen::Vector3d view_right = vp_origin_;
  view_right.x() = vp_origin_.x() + 0.05;
  view_positions_.push_back(view_right);
  view_right.x() = view_right.x() + 0.05;
  view_right.z() = vp_origin_.z() + 0.05;
  view_positions_.push_back(view_right);
  view_right.y() = view_right.y() + 0.05; 
  view_positions_.push_back(view_right);

  Eigen::Vector3d view_front = vp_origin_;
  view_front.y() = vp_origin_.y() + 0.05;
  view_positions_.push_back(view_front);
  view_front.z() = vp_origin_.z() + 0.05;
  view_positions_.push_back(view_front);

  Eigen::Vector3d view_top = vp_origin_;
  view_top.z() = vp_origin_.z() - 0.05;
  view_positions_.push_back(view_top);

  head_base_link_origin_.x() = 0.722;
  head_base_link_origin_.y() = -0.070;
  head_base_link_origin_.z() = 0.487;
  
  arm_left_base_link_origin_.x() = 0.365;
  arm_left_base_link_origin_.y() = 0.11;
  arm_left_base_link_origin_.z() = 0.094;

  arm_right_base_link_origin_.x() = 1.070;
  arm_right_base_link_origin_.y() = 0.10;
  arm_right_base_link_origin_.z() = 0.080;
  

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
  pose_tracker_ = std::make_shared<active_perception_planner::OnlinePoseTracker>(nhp_);
  
  reset_map_client_ = nh_.serviceClient<std_srvs::Empty>("/gsm_node/reset_map");
  get_instanceclouds_client_ = nh_.serviceClient<vpp_msgs::GetListInstancePointclouds>("/gsm_node/get_list_instance_pointclouds");
  get_scenecloud_client_ = nh_.serviceClient<vpp_msgs::GetScenePointcloud>("/gsm_node/get_scene_pointcloud");

  timer_fruit_follower_   = nh_.createTimer(ros::Duration(0.4), &ActivePerciever::followFruitCallback, this);
  timer_status_publisher_ = nh_.createTimer(ros::Duration(0.5), &ActivePerciever::publishStatusTimerCallback, this);
  timer_fruit_estimator_  = nh_.createTimer(ros::Duration(0.1), &ActivePerciever::estimateFruitPoseFromCurrenDetectionsCallback, this);

  sub_surface_cloud_ =  nh_.subscribe(surface_cloud_topic_, 1, &ActivePerciever::receiveSurfacePointCloudCallback, this);

  pub_status_advertiser_ = nhp_.advertise<std_msgs::Int8>("status", 1, true);

  pub_interested_fruit_ = nhp_.advertise<oc2_interface_msgs::DetectedFruit>("interested_fruit", 1, false);

  pub_int_obs_cloud_ = nhp_.advertise<sensor_msgs::PointCloud2>("interested/obs_cloud", 1, false);
  pub_int_pred_cloud_ = nhp_.advertise<sensor_msgs::PointCloud2>("interested/pred_cloud", 1, false);
  pub_int_peduncle_cloud_ = nhp_.advertise<sensor_msgs::PointCloud2>("interested/peduncle_cloud", 1, false);


  listener_ = std::make_unique<tf::TransformListener>();

  timer_status_publisher_.start();
  pose_tracker_->switchXarmToTraj();
  ROS_INFO("ActivePerciever");
}

ActivePercieverStatus ActivePerciever::getActivePercieverStatus()
{
  return status_;
}

bool ActivePerciever::clearSceneMap()
{
  follow_fruit_enabled_.store(false);
  {
    std::scoped_lock lk(mutex_timer_estimator_);
    online_fruit_estimator_.reset();
  }
  pose_tracker_->switchXarmToTraj();
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
  callEmptyService(client_start_gsm_node_);
  processDetectionsAtSensorPose();
  for(const auto& next_pose: mapping_poses_list_)
  {
    //callEmptyService(client_stop_gsm_node_);
    // std_msgs::Bool control;
    // control.data = false;
    // pub_control_gsm_.publish(control);
    sleep(0.1);
    if(moveToNextMappingPose(next_pose)== false)
    {
      setActivePercieverStatus(ERROR_LOCAL_MAPPING_MOTION);
      return false; //getActivePercieverStatus();
    }
    
    sleep(0.5);
    processDetectionsAtSensorPose();

  }
  sleep(0.5);
  // std_msgs::Bool control;
  // control.data = false;
  // pub_control_gsm_.publish(control);
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
    //ROS_WARN_STREAM("Tracked Detections available");
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
  
  auto init_pose = mapping_poses_list_[0];
  return motion_planner_->moveNamedTarget(init_pose, planner_id_);

}

bool ActivePerciever::moveToNextMappingPose(const std::string& next_pose)
{
  if(is_move_group_used_ == false)
    motion_planner_->clearMotionPlanRequestConstraints();
  
  ROS_INFO_STREAM("mapping_motion_type_"<<mapping_motion_type_);
  bool success = motion_planner_->moveNamedTarget(next_pose, mapping_motion_type_);
  if(success == false && mapping_motion_type_ == "LIN" && attempt_with_ptp_on_failure_ == true)
  {
    ROS_WARN_STREAM("Failure retrying mapping_motion_type_"<<planner_id_);
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
    ROS_INFO_STREAM("Found fruit with id: "<<desired_fruit_id);
    return true;
  }
  ROS_ERROR_STREAM("Could NOT find fruit");
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

bool ActivePerciever::startFollowPeduncle(const uint &fruit_id)
{
  start_peduncle_following_.store(true);
  ROS_WARN_STREAM_THROTTLE(1.0, "START FOLLOW PEDUNCLE"<<start_peduncle_following_);
  geometry_msgs::TransformStamped ee_tf;
  std::string ref_frame_id = "trolley_local_link";
  // if(getPose(ee_tf, "arm_right_eef_link", ref_frame_id))
  // {
  //   ROS_WARN_STREAM_THROTTLE(1.0, "START FOLLOW PEDUNCLE. Set Grasping Position");
  //   fruit_grasping_position_.x() = ee_tf.transform.translation.x;
  //   fruit_grasping_position_.y() = ee_tf.transform.translation.y;
  //   fruit_grasping_position_.z() = ee_tf.transform.translation.z;
  //   online_fruit_estimator_->setFruitGraspingPosition(fruit_grasping_position_);
  // }
  return true;
}

bool ActivePerciever::startFollowFruit(const uint &fruit_id)
{
  follow_fruit_enabled_.store(false);
  if(timer_fruit_estimator_.hasStarted())
    timer_fruit_estimator_.stop();
  sleep(0.2);
  fruit_est_initialised_ = false;
  start_peduncle_following_.store(false);
  {
    std::scoped_lock lk(mutex_timer_estimator_);
    online_fruit_estimator_.reset();
  }
  if(retrieveInterestedFruit(fruit_id))
  {
    if(fruit_est_initialised_ == false)
    {
      setCurrentEstimateOfTheInterestedFruit(interested_fruit_);
      fruit_est_initialised_ = true;
      {
        std::scoped_lock lk(mutex_timer_estimator_);
        online_fruit_estimator_ = std::make_shared<active_perception_planner::OnlineFruitEstimator>(nhp_, roi_detector_, se_detector_, interested_fruit_);
      }
    }
    pose_tracker_->switchXarmToOTG();

    if(fruit_est_initialised_ && timer_fruit_follower_.hasStarted() == false)
    {
      timer_fruit_follower_.start();
    }
    if(timer_fruit_estimator_.hasStarted() == false)
    {
      timer_fruit_estimator_.start();
    }
    follow_fruit_enabled_.store(true);
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
  sleep(0.5);
  follow_fruit_enabled_ = false;
  interested_fruit_ = superellipsoid::DetectedFruit();
  fruit_est_initialised_ = false;
  pose_tracker_->stopOTGMotion();
  sleep(0.1);
  pose_tracker_->switchXarmToTraj();
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
  if(follow_fruit_enabled_.load() == false)
    return;
  auto fruit_with_peduncle = getCurrentEstimateOfTheInterestedFruit();
  ROS_INFO_THROTTLE(2.0, "after getCurrentEstimateOfTheInterestedFruit");
  if(pose_tracker_->isOTGModeReady() == false)
  {
    ROS_ERROR_STREAM_THROTTLE(5.0, "OTG Mode not ready"<<pose_tracker_->getOTGStatus().status);
    return;
  }
  if(pose_tracker_->isOTGReadyForNewTarget()== false)
  {
    ROS_ERROR_STREAM_THROTTLE(2.0, "Current OTG Target not reached"<<pose_tracker_->getOTGStatus().status);
    return;
  }
  if(fruit_est_initialised_ == false)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Fruit estimate not initialized");
    return;
  }
  bool initial_motion_success = false;
  bool compute_view_pose_success = false;
  bool perform_motion = false;
  geometry_msgs::PoseStamped view_pose_stamped;
  if(use_fixed_pose_)
  {
    ROS_INFO_STREAM_THROTTLE(2.0, "+++++++++++++++++++++++++++++++++++++++++Entered use_fixed_pose: "<< orig_est_interested_fruit_.fruit.fruit_pose.position);
    compute_view_pose_success = computeInitialFixedViewPose(view_pose_stamped, orig_est_interested_fruit_.fruit.fruit_pose.position);
    if(compute_view_pose_success == false)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Error computing viewpose");
      return;
    }
    if(utils::arePosesApproximatelyEqual(curr_view_pose_.pose, view_pose_stamped.pose))
    {
      ROS_INFO_STREAM_THROTTLE(2.0, "New pose is approx equal to old pose. Hence not sending");
      return;
    }
    perform_motion = true;
  }
  else
  {
    ROS_INFO_THROTTLE(2.0, "VARIABLE POSE*************************************************Entered variable pose***************************************************");
    // geometry_msgs::Point vp;
    // if(computeViewpointOriginForZPlane(vp, orig_est_interested_fruit_.fruit.fruit_pose.position, orig_est_interested_fruit_.fruit.fruit_size, orig_est_interested_fruit_.fruit.observed_shape_centroid, 0.95, 0.25) == false)
    // {
    //   ROS_WARN_STREAM_THROTTLE(2.0, "Error computing vp origin");
    //   return;
    // }
    compute_view_pose_success = computeViewPose(view_pose_stamped, orig_est_interested_fruit_.fruit.fruit_pose.position, orig_est_interested_fruit_.fruit.fruit_size, 0.15, 0.22);
    if(compute_view_pose_success == false)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Error computing viewpose");
      use_fixed_pose_ = true;
      return;
    }
    if(utils::arePosesApproximatelyEqual(curr_view_pose_.pose, view_pose_stamped.pose))
    {
      ROS_INFO_STREAM_THROTTLE(2.0, "New pose is approx equal to old pose. Hence not sending");
      return;
    }
    perform_motion = true;
  }
  if(compute_view_pose_success && perform_motion)
  {
    ROS_INFO_STREAM_THROTTLE(2.0, "VARIABLE POSE: Send to OTG");
    pose_tracker_->updateOTGGoalPose(view_pose_stamped, "zed2i_camera_center");
    initial_motion_done_ = true;
    curr_view_pose_ = view_pose_stamped;
  }
}

void ActivePerciever::estimateFruitPoseFromCurrenDetectionsCallback(const ros::TimerEvent& e)
{
  if(follow_fruit_enabled_.load() == false)
    return;
  std::scoped_lock lk(mutex_timer_estimator_);
  if(pose_tracker_->isOTGRunning() == false)
  {
    ROS_INFO_STREAM_THROTTLE(3.0, "OTG not running. Hence compute fruit estimate");
    bool is_new_estimate_available = false;
    if(start_peduncle_following_.load() == false)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Following Fruit...");
      {
        {
          if(online_fruit_estimator_ !=nullptr)
            online_fruit_estimator_->computeFruitEstimate();
        }
      }
    }
    else
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Following Peduncle...");
      if(online_fruit_estimator_ !=nullptr)
        online_fruit_estimator_->computePeduncleEstimate();
    }
    if(online_fruit_estimator_->isNewEstimateAvailable())
    {
      DetectedFruitWithPeduncle new_estimate;
      if(online_fruit_estimator_ != nullptr)
          new_estimate = online_fruit_estimator_->getUpdatedEstimateofInterestedFruit();

      if(new_estimate.has_peduncle == false)
        ROS_WARN_STREAM_THROTTLE(2.0, "New Fruit Estimate BUT NO PEDUNCLE");
      
      ROS_INFO_STREAM_THROTTLE(0.1, "Got new estimate...");
      pub_interested_fruit_.publish(getInterestedFruitAsRosMsgType(new_estimate));
      setCurrentEstimateOfTheInterestedFruit(new_estimate);
    }
    else
    {
      ROS_ERROR_STREAM_THROTTLE(2.0, "No new estimate");
    }
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(3.0, "OTG running. "<<pose_tracker_->getOTGStatus().status);
  }
}
// bool ActivePerciever::isDetectedFruitCurrentInterestedOne(const DetectedFruitWithPeduncle& prev_det, DetectedFruitWithPeduncle& potential_candidate, double center_threshold)
// {
//   return isFruitCentreSimilar(prev_det.fruit.fruit_pose.position, potential_candidate.fruit.fruit_pose.position, center_threshold);
// }

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
  // // Convert to PCL Pointcloud
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::fromROSMsg(surface_cloud, *pc_pcl);

  // auto clusters = getClusters(pc_pcl);

  // if(clusters.empty())
  //   return false;
  // prev_est_interested_fruit_ = curr_est_interested_fruit_;
  // for(auto& cluster: clusters)
  // {
  //   DetectedFruitWithPeduncle possible_same_fruit;
  //   if(se_detector_->completeFruitShape(cluster, possible_same_fruit.fruit) == false)
  //     continue;

  //   if(isDetectedFruitCurrentInterestedOne(prev_est_interested_fruit_, possible_same_fruit))
  //   {
  //     //interested_fruit_detections_buffer_.push_back(possible_same_fruit);
  //     //interested_fruit_detections_buffer_.pop_back();
  //     setCurrentEstimateOfTheInterestedFruit(possible_same_fruit);
  //     return true;
  //   }
  // }
  // ROS_ERROR("Could not detect fruit in the curr surface cloud");
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

oc2_interface_msgs::InterestedFruit ActivePerciever::getInterestedFruitAsRosMsgType(const superellipsoid::DetectedFruit& fruit)
{
  oc2_interface_msgs::InterestedFruit fruit_msg;

  pcl::toROSMsg(*fruit.observed_fruit_shape, fruit_msg.observed_fruit);
  pcl::toROSMsg(*fruit.completed_fruit_shape, fruit_msg.predicted_fruit);
  fruit_msg.fruit_pose = fruit.fruit_pose;

  fruit_msg.header.frame_id = world_frame_;
  fruit_msg.header.stamp = ros::Time::now();
  return fruit_msg;
}

oc2_interface_msgs::DetectedFruit ActivePerciever::getInterestedFruitAsRosMsgType(const DetectedFruitWithPeduncle& fruit_with_peduncle)
{
  oc2_interface_msgs::DetectedFruit fruit;
  fruit.header.frame_id = "trolley_local_link";
  fruit.header.stamp = ros::Time::now();
  fruit.fruit_id = fruit_with_peduncle.fruit.fruit_id;
  fruit.fruit_pose = fruit_with_peduncle.fruit.fruit_pose;
  fruit.fruit_dimensions = fruit_with_peduncle.fruit.fruit_size;
  fruit.score = fruit_with_peduncle.fruit.observation_score;
  fruit.associated_stem_id = fruit_with_peduncle.nearest_stem;
  

  if(fruit_with_peduncle.fruit.observed_fruit_shape != nullptr)
  {
    pcl::toROSMsg(*(fruit_with_peduncle.fruit.observed_fruit_shape), fruit.observed_fruit);  
    //pcl::toROSMsg(fruit_with_peduncle.fruit.observed_fruit_shape->header, fruit.observed_fruit.header); 
    fruit.observed_fruit.header.frame_id = fruit_with_peduncle.fruit.observed_fruit_shape->header.frame_id;
    fruit.observed_fruit.header.frame_id = "trolley_local_link";
    pub_int_obs_cloud_.publish(fruit.observed_fruit);
  }

  if(fruit_with_peduncle.fruit.completed_fruit_shape != nullptr)
  {
    pcl::toROSMsg(*(fruit_with_peduncle.fruit.completed_fruit_shape), fruit.predicted_fruit);  
    fruit.predicted_fruit.header.frame_id = fruit_with_peduncle.fruit.completed_fruit_shape->header.frame_id; 
    fruit.predicted_fruit.header.frame_id = "trolley_local_link";
    pub_int_pred_cloud_.publish(fruit.predicted_fruit);
  }
  
  if(fruit_with_peduncle.peduncle.peduncle_cloud != nullptr)
  {
    pcl::toROSMsg(*(fruit_with_peduncle.peduncle.peduncle_cloud), fruit.peduncle.cloud);  
    fruit.peduncle.cloud.header.frame_id = fruit.observed_fruit.header.frame_id;
    fruit.peduncle.cloud.header.frame_id = "trolley_local_link";
    pub_int_peduncle_cloud_.publish(fruit.peduncle.cloud);
  }
  fruit.has_peduncle = fruit_with_peduncle.has_peduncle;
  fruit.peduncle.associated_fruit_id = fruit_with_peduncle.peduncle.associated_fruit_id;
  fruit.peduncle.peduncle_id  = fruit_with_peduncle.peduncle.peduncle_id;
  fruit.peduncle.stempoint    = utils::fromEigen2GeometryMsgPoint(fruit_with_peduncle.peduncle.stempoint);
  fruit.peduncle.fruitpoint   = utils::fromEigen2GeometryMsgPoint(fruit_with_peduncle.peduncle.fruitpoint);

  return fruit;
}

void ActivePerciever::receiveSurfacePointCloudCallback(const sensor_msgs::PointCloud2 &pc_msg)
{
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

/// @brief 
/// @param view_pose 
/// @param fruit_target 
/// @return bool 
bool ActivePerciever::computeInitialFixedViewPose(geometry_msgs::PoseStamped& view_pose, const geometry_msgs::Point& fruit_target)
{
  ROS_INFO_THROTTLE(1.0, "Computing new pose");
  geometry_msgs::TransformStamped camera_tf;
  std::string ref_frame_id = "trolley_local_link";
  if(getPose(camera_tf, "zed2i_camera_center", ref_frame_id)== false)
  {
    return false;
  }
  if(initial_motion_done_ == false)
    utils::convert(camera_tf, curr_view_pose_);
  
  tf2::Quaternion cam_quat;
  tf2::fromMsg(camera_tf.transform.rotation, cam_quat);

  Eigen::Vector3d fruit_target_eigen;
  tf2::fromMsg(fruit_target, fruit_target_eigen);

  int idx = pose_iter_ % 5;
  auto vp_origin = vp_origin_;
  vp_origin.y() = vp_origin.y() + 0.02; 
  vp_origin.z() = vp_origin.z() + 0.01; 

  pose_iter_++;
  view_pose.pose = computeViewPose(fruit_target_eigen, vp_origin, cam_quat);
  view_pose.header.frame_id = ref_frame_id;
  ROS_INFO_STREAM_THROTTLE(1.0, "New Pose Computed"<<view_pose.pose);
  return true;
}

geometry_msgs::Pose ActivePerciever::computeViewPose(const Eigen::Vector3d& fruit_target, const Eigen::Vector3d& vp_origin, const tf2::Quaternion& cam_quat)
{
  Eigen::Vector3d dir_vec = (fruit_target - vp_origin).normalized();
  const tf2::Matrix3x3 cam_mat(cam_quat);
  tf2::Vector3 view_dir(cam_mat.getColumn(0));
  
  geometry_msgs::Pose view_pose;

  view_pose.orientation = utils::dirVecToQuat(dir_vec, cam_quat,view_dir);
  tf::pointEigenToMsg(vp_origin, view_pose.position);
  //view_pose.orientation = tf2::toMsg(view_quat);

  return view_pose;
}

bool ActivePerciever::computeViewpointOriginForZPlane(geometry_msgs::Point& vp_origin, const geometry_msgs::Point& fruit_centre, const geometry_msgs::Vector3& fruit_dim, const geometry_msgs::Point& obs_target, double z_plane, double min_distance)
{
  int max_num_tries = 4;
  Eigen::Vector3d ray_origin, ray_pt, ideal_target;
  ray_origin(0) = fruit_centre.x;
  ray_origin(1) = fruit_centre.y + fruit_dim.y;
  ray_origin(2) = fruit_centre.z - fruit_dim.z;

  ray_pt(0) = obs_target.x;
  ray_pt(1) = obs_target.y;
  ray_pt(2) = obs_target.z;

  ideal_target = ray_pt;

  ROS_INFO_STREAM_THROTTLE(1.0, "Origin: "<<ray_origin.transpose()<<"\t Ray target: "<<ideal_target.transpose());

  bool success = false;
  Eigen::Vector3d plane_point;
  plane_point.setZero();
  plane_point(2) = fruit_centre.z + fruit_dim.z + 0.05;
  Eigen::Vector3d inter_pt;
  for(int num_tries = 0; num_tries < max_num_tries; num_tries++)
  {
    if(calcLinePlaneIntersection(inter_pt, ray_origin, ray_pt, plane_normal_, plane_point) == false)
    {      
      inter_pt(2) = obs_target.z + 0.05;
      inter_pt(1) = obs_target.y - 0.2;
      inter_pt(0) = obs_target.x;
      ROS_INFO_STREAM_THROTTLE(1.0, "Ray plane intersection failure as ray parallel to z axis. Hence recalculating..."<<inter_pt.transpose());

    }
    if(((inter_pt - head_base_link_origin_).norm() > 0.6))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Intersection point unreachable. Hence recalculating with lower z "<<inter_pt.transpose());
      plane_point(2) = plane_point(2) - 0.05;
      continue;
    }
    if(((inter_pt.head(2) - head_base_link_origin_.head(2)).norm() > 0.54))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Intersection point unreachable in x-y plane. Hence recalculating with fruit centre instead of observed centroid"<<inter_pt.transpose()<<"x-y dist: "<<(inter_pt.head(2) - head_base_link_origin_.head(2)).norm());
      ray_pt(0) = fruit_centre.x;
      ray_pt(1) = fruit_centre.y;
      ray_pt(2) = fruit_centre.z;
      continue;
    }
    if((inter_pt-ray_origin).norm() < min_distance)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Intersection point close to ray origin. Hence recalculating with higher z"<<inter_pt.transpose());
      plane_point(2) = plane_point(2) + 0.05;
      continue;
    }
    if(inter_pt(0) < arm_left_base_link_origin_(0))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Intersection pt is too much to the left"<<inter_pt.transpose());
      ray_pt(0) = fruit_centre.x;
      ray_pt(1) = fruit_centre.y;
      ray_pt(2) = fruit_centre.z;
      continue;
    }
    if(inter_pt(0) > arm_right_base_link_origin_(0))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Intersection pt is too much to the right"<<inter_pt.transpose());
      ray_pt(0) = fruit_centre.x;
      ray_pt(1) = fruit_centre.y;
      ray_pt(2) = fruit_centre.z;
      continue;
    }
    success = true;
    break;
  }
  if(success)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "GR88888888888888. Interpt found: "<<inter_pt.transpose());
    vp_origin.x = inter_pt.x();
    vp_origin.y = inter_pt.y();
    vp_origin.z = inter_pt.z();
    return true;
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OOOOOOOOOOOOPS. Interpt not found: "<<inter_pt.transpose());
    Eigen::Vector3d vp_closest;
    double max_cosine_distance = -2;
    int max_cosine_distance_index = -1;
    int max_similarity_score = -1000;
    for(int i = 0; i < view_positions_.size(); ++i)
    {
      double cosine_distance, euc_distance, similarity_score;
      auto vp = view_positions_[i];
      if(utils::computeSimilarity(similarity_score, cosine_distance, euc_distance, ray_origin, ideal_target, vp))
      {
        if(similarity_score > max_similarity_score)
        {
          max_similarity_score = similarity_score;
          max_cosine_distance_index = i;
        }
      }
    }
    if(max_cosine_distance_index > -1)
    {
      vp_closest = view_positions_[max_cosine_distance_index];
      vp_origin.x = vp_closest.x();
      vp_origin.y = vp_closest.y();
      vp_origin.z = vp_closest.z();
      ROS_INFO_STREAM_THROTTLE(1.0, "Pose similar to ideal: "<<vp_closest.transpose()<<" similarity score: "<<max_similarity_score);
      return true;
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "None of the poses are similarly aligned");
      return false;
    }
  }
}
bool ActivePerciever::computeViewPoseForZPlane(geometry_msgs::PoseStamped& view_pose, const geometry_msgs::Point& fruit_target, const geometry_msgs::Point& vp_origin)
{
  ROS_INFO_THROTTLE(1.0, "Computing new pose");
  geometry_msgs::TransformStamped camera_tf;
  std::string ref_frame_id = "trolley_local_link";
  if(getPose(camera_tf, "zed2i_camera_center", ref_frame_id)== false)
  {
    return false;
  }
  if(initial_motion_done_ == false)
    utils::convert(camera_tf, curr_view_pose_);
  
  tf2::Quaternion cam_quat;
  tf2::fromMsg(camera_tf.transform.rotation, cam_quat);

  Eigen::Vector3d fruit_target_eigen;
  tf2::fromMsg(fruit_target, fruit_target_eigen);

  Eigen::Vector3d vp_eigen;
  tf2::fromMsg(vp_origin, vp_eigen);

  view_pose.pose = computeViewPose(fruit_target_eigen, vp_eigen, cam_quat);
  view_pose.header.frame_id = ref_frame_id;
  ROS_INFO_STREAM_THROTTLE(1.0, "New Pose Computed"<<view_pose.pose);
  return true;
}


bool ActivePerciever::moveToPose(const geometry_msgs::Pose& view_pose)
{
  motion_planner_->getMoveGroupHandle()->setPlannerId(planner_id_);
  ROS_WARN_STREAM("mapping_motion_type_"<<planner_id_);
  return motion_planner_->movePoseTarget(view_pose, planner_id_);
}

bool ActivePerciever::callEmptyService(ros::ServiceClient& client)
{
  std_srvs::Empty empty_srv;
  if (!client.call(empty_srv))
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
  return true;
}

bool ActivePerciever::computeViewPose(geometry_msgs::PoseStamped& view_pose, const geometry_msgs::Point& fruit_centre, const geometry_msgs::Vector3& fruit_dim, double z_height , double y_coordinate)
{
  double fruit_x = fruit_centre.x;
  double head_base_x  = head_base_link_origin_.x();
  double left_base_x =  arm_left_base_link_origin_.x();
  double right_base_x = arm_right_base_link_origin_.x();

  double vp_x = head_base_x + 0.8*(fruit_x - head_base_x);
  if(vp_x < left_base_x+0.05)
    vp_x = head_base_x + 0.7*(left_base_x - head_base_x);
  else if(vp_x > right_base_x-0.05)
    vp_x = head_base_x + 0.7*(right_base_x - head_base_x);

  double vp_y = y_coordinate;
  
  double fruit_top = fruit_centre.z + fruit_dim.z;
  double vp_z = fruit_top + z_height;

  double z_reach = head_base_link_origin_.z() + 0.58;
  if(vp_z > z_reach)
  {
    vp_z = z_reach;
    vp_y = y_coordinate - 0.05;
  }
  
  Eigen::Vector3d vp_origin;
  vp_origin.x() = vp_x;
  vp_origin.y() = vp_y;
  vp_origin.z() = vp_z;

  ROS_INFO_STREAM_THROTTLE(2.0, "VP Origin: "<<vp_origin.transpose());

  //ROS_INFO_THROTTLE(1.0, "Computing new pose");
  geometry_msgs::TransformStamped camera_tf;
  std::string ref_frame_id = "trolley_local_link";
  if(getPose(camera_tf, "zed2i_camera_center", ref_frame_id)== false)
  {
    return false;
  }
  if(initial_motion_done_ == false)
    utils::convert(camera_tf, curr_view_pose_);
  
  tf2::Quaternion cam_quat;
  tf2::fromMsg(camera_tf.transform.rotation, cam_quat);

  Eigen::Vector3d fruit_target_eigen;
  tf2::fromMsg(fruit_centre, fruit_target_eigen);

  //fruit_target_eigen.z() +=  fruit_dim.z();

  view_pose.pose = computeViewPose(fruit_target_eigen, vp_origin, cam_quat);
  view_pose.header.frame_id = ref_frame_id;
  ROS_INFO_STREAM_THROTTLE(2.0, "New Pose Computed"<<view_pose.pose);
  return true;

}
