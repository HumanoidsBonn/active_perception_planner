#ifndef ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H
#define ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H

#include<vector>
#include<memory>



#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include<oc2_interface_msgs/InterestedFruit.h>
#include<oc2_interface_msgs/DetectedFruits.h>
#include<vpp_msgs/GetScenePointcloud.h>
#include<vpp_msgs/GetListInstancePointclouds.h>
#include<pcl_conversions/pcl_conversions.h>

#include <voxblox_msgs/Layer.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/tsdf_server.h>

#include<moveit_motion_planner/motion_planner.h>
#include<capsicum_superellipsoid_detector/se_detector.h>
#include <active_perception_planner/voxblox_map.h>



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

struct InterestedFruit: public superellipsoid::DetectedFruit
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_observation;

  InterestedFruit():superellipsoid::DetectedFruit(), current_observation(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
  }

  InterestedFruit(const superellipsoid::DetectedFruit& det_fruit):current_observation(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    this->valid_shape_completion_available = det_fruit.valid_shape_completion_available;
    this->observed_fruit_shape = det_fruit.observed_fruit_shape;
    this->completed_fruit_shape = det_fruit.completed_fruit_shape;
    this->fruit_pose = det_fruit.fruit_pose;
    this->observed_shape_centroid = det_fruit.observed_shape_centroid;
  }
};


class ActivePerciever{

public:

  ActivePerciever(ros::NodeHandle& nhp, const std::string& planning_group_name, const std::string& base_frame, const std::string& tip_frame);

  ActivePercieverStatus getActivePercieverStatus();

  bool clearSceneMap();

  bool performLocalMappingMotion();

  bool createSceneMapWithCompletedFruits(vpp_msgs::GetListInstancePointclouds& instcloud_srv);

  bool moveToInterestedFruitObservationPose(const uint &fruit_id);

  bool startFollowFruit(const uint &fruit_id);

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


protected:
  ActivePercieverStatus status_;
  ros::NodeHandle nh_, nhp_;

  ros::Timer timer_fruit_follower_, timer_status_publisher_;

  bool is_move_group_used_ = true;

  std::shared_ptr<motion_planning::MotionPlanner> motion_planner_;
  std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector_;
  std::shared_ptr<active_perception_planner::VoxbloxMap> map_;

  //voxblox map
  std::mutex layer_mutex_;
  bool layer_present_;
  std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> map_layer_;
  std::shared_ptr<voxblox::Block<voxblox::TsdfVoxel>> block_;
  voxblox::TsdfVoxel voxel_;

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


  FruitMap fruit_map_;
  bool fruit_map_available_ = false;
  superellipsoid::DetectedFruit interested_fruit_;

  float obs_z_plane_;

  Eigen::Vector3d plane_point_, plane_normal_;
  int voxel_factor_, max_num_tries_;

  geometry_msgs::TransformStamped came_frame_tf_;

  tf2::Quaternion cam_quat_;
  tf2::Vector3 view_dir_;

  sensor_msgs::PointCloud2 scene_cloud_;

  std::string surface_cloud_topic_;

  bool follow_fruit_enabled_ = false;
  bool fruit_est_initialised_ = false;

  //sensor_msgs::JointState

  std::vector<std::string> mapping_poses_list_; // list of named poses for exploring the current local poses
  std::string mapping_motion_type_;  // LIN for linear motion between 2 poses, and PTP for joint motion between 2 poses

  bool enable_check_manipulation_arms_stowed_ = false;
  bool checkManipulationArmsStowed();


  bool moveToInitialMappingPose();
  bool moveToNextMappingPose(const std::string &next_pose);

  inline void setActivePercieverStatus(const ActivePercieverStatus& status)
  {
    status_atomic_.store(int(status));
    status_ = status;
  }

  bool retrieveInterestedFruit(const uint &fruit_id, const geometry_msgs::Point& fruit_centre);
  bool retrieveInterestedFruit(const uint &desired_fruit_id);

  std::deque<superellipsoid::DetectedFruit> interested_fruit_detections_buffer_;
  superellipsoid::DetectedFruit curr_est_interested_fruit_, prev_est_interested_fruit_;

  bool isDetectedFruitCurrentInterestedOne(const superellipsoid::DetectedFruit& prev_det, superellipsoid::DetectedFruit& potential_candidate, double center_threshold = 0.01);

  bool updateInterestedFruit(const sensor_msgs::PointCloud2 &surface_cloud); // const geometry_msgs::Point& fruit_centre);


  bool findOptimumFruitObservationPoses(std::deque<geometry_msgs::Pose>& view_poses);

  inline void setScenePointCloud(const sensor_msgs::PointCloud2& cloud)
  {
    scene_cloud_.data.clear();
    scene_cloud_ = cloud;
  }

  void publishStatusTimerCallback(const ros::TimerEvent&);

  void followFruitCallback(const ros::TimerEvent&);

  int min_cluster_size_, max_cluster_size_, cluster_tolerance_;
  std::string world_frame_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl);

  inline superellipsoid::DetectedFruit getCurrentEstimateOfTheInterestedFruit()
  {
    return curr_est_interested_fruit_;
  }

  inline void setCurrentEstimateOfTheInterestedFruit(const superellipsoid::DetectedFruit& fruit, bool fruit_est_initialised = true)
  {
    if(fruit_est_initialised)
      prev_est_interested_fruit_ = curr_est_interested_fruit_;
    else
      prev_est_interested_fruit_ = fruit;

    curr_est_interested_fruit_ = fruit;
  }

  oc2_interface_msgs::DetectedFruit getasRosMsgType(const superellipsoid::DetectedFruit& fruit);
  oc2_interface_msgs::InterestedFruit getInterestedFruitAsRosMsgType(const superellipsoid::DetectedFruit& fruit);
};

} // namespace active_perception_planner

#endif // ACTIVE_PERCEPTION_PLANNER_ACTIVE_PERCIEVER_H
