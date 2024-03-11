#ifndef ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H
#define ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H

#include<vector>
#include<memory>



#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pointcloud_roi_msgs/PointCloudInstances.h>
#include<oc2_interface_msgs/InterestedFruit.h>
#include<oc2_interface_msgs/DetectedFruits.h>
#include<oc2_interface_msgs/Crop.h>

#include<vpp_msgs/GetScenePointcloud.h>
#include<vpp_msgs/GetListInstancePointclouds.h>
#include<pcl_conversions/pcl_conversions.h>

#include <voxblox_msgs/Layer.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/tsdf_server.h>

#include<moveit_motion_planner/motion_planner.h>
#include<capsicum_superellipsoid_detector/se_detector.h>
#include <pointcloud_roi/detected_roi_filter.h>
#include "active_perception_planner/common.h"
#include "active_perception_planner/peduncle_estimator.h"
#include "active_perception_planner/stem_estimator.h"
#include "active_perception_planner/crop_estimator.h"
#include "active_perception_planner/online_fruit_estimator.h"

#include "active_perception_planner/online_pose_tracker.h"
#include "active_perception_planner/voxblox_tsdf_map.h"


namespace active_perception_planner {

enum ActivePercieverStatus
{
  IDLE = 0,
  LOCAL_MAPPING_MOTION_IN_PROGRESS = 1,
  LOCAL_MAPPING_MOTION_SUCCESS = 2,
  FRUIT_MAP_GENERATION_IN_PROGRESS = 3,

  FRUIT_MAP_GENERATION_SUCCESS = 4,
  FRUIT_FOLLOWING_IN_PROGRESS = 5,
  PEDUNCLE_DETECTION_IN_PROGRESS = 6,

  ERROR_MANIP_ARMS_NOT_STOWED = -1,
  ERROR_MOVE_TO_TRANSPORT_POSE = -2,
  ERROR_LOCAL_MAPPING_MOTION = -3,
  ERROR_MAP_GENERATION = -4,
  ERROR_NO_FRUITS_IN_MAP = -5,
  ERROR_SHAPE_COMPLETION = -6,
  ERROR_FRUIT_FOLLOWING = -7
};

class ActivePerciever{

public:

  ActivePerciever(ros::NodeHandle& nhp, const std::string& planning_group_name, const std::string& base_frame, const std::string& tip_frame, const bool disable_motion=false);

  ActivePercieverStatus getActivePercieverStatus();

  bool clearSceneMap();

  bool moveToInitialMappingPose();

  bool performLocalMappingMotion();

  bool createSceneMapWithCompletedFruits(vpp_msgs::GetListInstancePointclouds& instcloud_srv);

  bool moveToInterestedFruitObservationPose(const uint &fruit_id);

  bool startFollowFruit(const uint &fruit_id);
  
  bool startFollowPeduncle(const uint &fruit_id);

  bool stopFollowFruit();

  ActivePercieverStatus followFruit(const uint& fruit_id, const geometry_msgs::Point& fruit_centre);

  inline FruitMap getFruitMap()
  {
    return fruit_map_;
  }

  inline sensor_msgs::PointCloud2 getScenePointCloud()
  {
    return scene_cloud_;
  }

  void receiveSurfacePointCloudCallback(const sensor_msgs::PointCloud2& pc_msg);

  void receiveCameraPointCloudCallback(const sensor_msgs::PointCloud2& pc_msg);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroupHandle()
  {
      return motion_planner_->getMoveGroupHandle();
  }

  Crop crop_;
  void createCrop();

  inline Crop getCrop()
  {
    return crop_;
  }



protected:
  ActivePercieverStatus status_;
  ros::NodeHandle nh_, nhp_;

  ros::Timer timer_fruit_follower_, timer_status_publisher_, timer_fruit_estimator_;
  std::mutex mutex_timer_estimator_;
  bool is_move_group_used_ = true;

  std::shared_ptr<motion_planning::MotionPlanner> motion_planner_;
  std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector_;

  std::shared_ptr<active_perception_planner::PeduncleEstimator> peduncle_estimator_;
  std::shared_ptr<active_perception_planner::StemEstimator> stem_estimator_;
  std::shared_ptr<active_perception_planner::CropEstimator> crop_estimator_;

  std::shared_ptr<active_perception_planner::OnlineFruitEstimator> online_fruit_estimator_;

  ros::ServiceClient client_start_gsm_node_, client_stop_gsm_node_;
  ros::Publisher pub_control_gsm_;

  bool callEmptyService(ros::ServiceClient& client);


  //voxblox map
  // std::mutex layer_mutex_;
  // bool layer_present_;
  // std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> map_layer_;
  // std::shared_ptr<voxblox::Block<voxblox::TsdfVoxel>> block_;
  // voxblox::TsdfVoxel voxel_;

  std::shared_ptr<active_perception_planner::OnlinePoseTracker> pose_tracker_;


  std::string planning_group_name_;
  std::string base_frame_;
  std::string tip_frame_;

  std::string moveit_planner_type_;
  std::string planner_id_;

  int num_planning_attempts_;
  float allowed_planning_time_;

  bool attempt_with_ptp_on_failure_ = true;

  ros::ServiceClient reset_map_client_;
  ros::ServiceClient get_instanceclouds_client_;
  ros::ServiceClient get_scenecloud_client_;

  ros::Publisher pub_status_advertiser_;
  std::atomic<int8_t> status_atomic_;

  ros::Publisher pub_interested_fruit_;

  ros::Subscriber sub_surface_cloud_;
  ros::Subscriber sub_point_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<tf::TransformListener> listener_;

  std::vector<superellipsoid::DetectedFruit> fruits_list_;
  FruitMap fruit_map_;
  bool fruit_map_available_ = false;
  DetectedFruitWithPeduncle interested_fruit_;

  float obs_z_plane_;

  Eigen::Vector3d plane_point_, plane_normal_;
  int voxel_factor_, max_num_tries_;

  geometry_msgs::TransformStamped came_frame_tf_;

  Eigen::Vector3d head_base_link_origin_;
  Eigen::Vector3d arm_left_base_link_origin_;
  Eigen::Vector3d arm_right_base_link_origin_;

  tf2::Quaternion cam_quat_;
  tf2::Vector3 view_dir_;

  sensor_msgs::PointCloud2 scene_cloud_;

  std::string surface_cloud_topic_;

  std::atomic_bool follow_fruit_enabled_ = false;
  bool fruit_est_initialised_ = false;

  std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector_;
  bool recd_tracked_detections_ = false;

  bool use_fixed_pose_ = true; 
  bool initial_motion_done_ = false;
  std::vector<double> fixed_view_position_; 
  Eigen::Vector3d vp_origin_;
  geometry_msgs::PoseStamped curr_view_pose_;
  std::vector<Eigen::Vector3d> view_positions_;
  uint64_t pose_iter_ = 0; 

  std::atomic_bool curr_view_pose_reached_ = false;

  std::atomic_bool peduncle_detected_ = false;
  
  bool moveToPose(const geometry_msgs::Pose&);
  geometry_msgs::Pose computeViewPose(const Eigen::Vector3d& fruit_target, const Eigen::Vector3d& vp_origin, const tf2::Quaternion& cam_quat);
  bool computeInitialFixedViewPose(geometry_msgs::PoseStamped& view_pose, const geometry_msgs::Point& fruit_target);
  bool getPose(geometry_msgs::TransformStamped& transform, const std::string& target_frame, const std::string& original_frame = "world");

  bool computeViewpointOriginForZPlane(geometry_msgs::Point& vp_origin, const geometry_msgs::Point& fruit_centre, const geometry_msgs::Vector3& fruit_dim, const geometry_msgs::Point& obs_target, double z_plane = 0.95, double min_distance=0.30);
  bool computeViewPoseForZPlane(geometry_msgs::PoseStamped& view_pose, const geometry_msgs::Point& fruit_target, const geometry_msgs::Point& vp_origin);
  
  bool computeViewPose(geometry_msgs::PoseStamped& view_pose, const geometry_msgs::Point& fruit_centre, const geometry_msgs::Vector3& fruit_dim, double z_height = 0.1, double y_coordinate = 0.20);
  //sensor_msgs::JointState

  std::vector<std::string> mapping_poses_list_; // list of named poses for exploring the current local poses
  std::string mapping_motion_type_;  // LIN for linear motion between 2 poses, and PTP for joint motion between 2 poses

  double fruit_separation_xycenter_diff_;
  double fruit_separation_z_diff_;

  bool disable_motion_ = false;

  bool enable_check_manipulation_arms_stowed_ = false;
  bool checkManipulationArmsStowed();

  Eigen::Vector3d fruit_grasping_position_; 

  std::atomic_bool start_peduncle_following_ = false;

  bool moveToNextMappingPose(const std::string &next_pose);

  bool computeFruitShapes(const std::vector<vpp_msgs::InstancePointcloudwithCentroid>& instance_clouds_with_centroid);
  bool computeFruitShapes(const pointcloud_roi_msgs::PointCloudInstances& pointcloud_instances);
;
  int removeFalsePositives();

  inline void setActivePercieverStatus(const ActivePercieverStatus& status)
  {
    status_atomic_.store(int(status));
    status_ = status;
  }

  bool retrieveInterestedFruit(const uint &fruit_id, const geometry_msgs::Point& fruit_centre);
  bool retrieveInterestedFruit(const uint &desired_fruit_id);

  std::deque<DetectedFruitWithPeduncle> interested_fruit_detections_buffer_;
  DetectedFruitWithPeduncle curr_est_interested_fruit_, prev_est_interested_fruit_, orig_est_interested_fruit_;


  bool updateInterestedFruit(const sensor_msgs::PointCloud2 &surface_cloud); // const geometry_msgs::Point& fruit_centre);


  bool findOptimumFruitObservationPoses(std::deque<geometry_msgs::Pose>& view_poses);

  inline void setScenePointCloud(const sensor_msgs::PointCloud2& cloud)
  {
    scene_cloud_.data.clear();
    scene_cloud_ = cloud;
  }

  void publishStatusTimerCallback(const ros::TimerEvent&);

  void followFruitCallback(const ros::TimerEvent&);

  void estimateFruitPoseFromCurrenDetectionsCallback(const ros::TimerEvent&);

  int min_cluster_size_, max_cluster_size_, cluster_tolerance_;
  std::string world_frame_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl);

  std::mutex mutex_;

  inline DetectedFruitWithPeduncle getCurrentEstimateOfTheInterestedFruit()
  {
    DetectedFruitWithPeduncle fruit_with_peduncle;
    {
      std::scoped_lock lk(mutex_);
      fruit_with_peduncle = curr_est_interested_fruit_;
    }
    return fruit_with_peduncle;
  }

  inline void setCurrentEstimateOfTheInterestedFruit(const DetectedFruitWithPeduncle& fruit, bool fruit_est_initialised = false)
  {
    std::scoped_lock lk(mutex_);
    if(fruit_est_initialised_)
      prev_est_interested_fruit_ = curr_est_interested_fruit_;
    else
    {
      orig_est_interested_fruit_ = fruit;
      prev_est_interested_fruit_ = fruit;
    }

    curr_est_interested_fruit_ = fruit;
  }

  void processDetectionsAtSensorPose();

  //oc2_interface_msgs::DetectedFruitOld getasRosMsgType(const superellipsoid::DetectedFruit& fruit);
  oc2_interface_msgs::InterestedFruit getInterestedFruitAsRosMsgType(const superellipsoid::DetectedFruit& fruit);

  oc2_interface_msgs::DetectedFruit getInterestedFruitAsRosMsgType(const DetectedFruitWithPeduncle& fruit_with_peduncle);

  ros::Publisher pub_int_obs_cloud_, pub_int_pred_cloud_, pub_int_peduncle_cloud_;

};

} // namespace active_perception_planner

#endif // ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H
