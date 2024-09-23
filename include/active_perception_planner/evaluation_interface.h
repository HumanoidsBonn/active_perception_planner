#ifndef ACTIVE_PERCEPTION_PLANNER_EVALUATION_INTERFACE_H
#define ACTIVE_PERCEPTION_PLANNER_EVALUATION_INTERFACE_H

#include "ros/ros.h"
#include "oc2_interface_msgs/DetectFruits.h"
#include "oc2_interface_msgs/StartFollowFruit.h"
#include "oc2_interface_msgs/StopFollowFruit.h"
#include "oc2_interface_msgs/SaveJointState.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include<active_perception_planner/active_perciever.h>

#include <actionlib/server/simple_action_server.h>
#include <oc2_interface_msgs/DetectFruitsAction.h>
#include <oc2_interface_msgs/StartFollowPeduncle.h>

#include <yolact_ros_msgs/SetConfidence.h>
#include <yolact_ros_msgs/SetRegionOfInterest.h>
#include <std_srvs/Empty.h>

#include "active_perception_planner/fruit_shape_estimator.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
namespace active_perception_planner
{

class EvaluationInterface
{
public:
  EvaluationInterface();

  bool processEvaluateFineServiceCalllback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool processEvaluateCoarseServiceCalllback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool processEvaluateCoarse2FineServiceCalllback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool saveScenePointCloudCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

// Callbacks for depth images and camera info
  void depthCallback1(const sensor_msgs::ImageConstPtr& depth_msg);
  void cameraInfoCallback1(const sensor_msgs::CameraInfoConstPtr& info_msg);

  void depthCallback2(const sensor_msgs::ImageConstPtr& depth_msg);
  void cameraInfoCallback2(const sensor_msgs::CameraInfoConstPtr& info_msg);

  void depthCallback3(const sensor_msgs::ImageConstPtr& depth_msg);
  void cameraInfoCallback3(const sensor_msgs::CameraInfoConstPtr& info_msg);

  // Convert depth image to point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(const sensor_msgs::ImageConstPtr& depth_msg, 
                                                        const sensor_msgs::CameraInfo& camera_info);

  // Transform point cloud to a target frame
  void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string target_frame, std_msgs::Header header);

  // Downsample point cloud using Voxel Grid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  // Process depth images, update point clouds, and perform necessary transformations
  void processDepthImage(const sensor_msgs::ImageConstPtr& depth_msg, 
                          const sensor_msgs::CameraInfo& camera_info, 
                          const std::string& frame_id, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

  bool saveResults(FruitMap& fruit_map);

protected:

  ros::NodeHandle nh_, nhp_;

  ros::Subscriber sub_depth_1_, sub_camera_info_1_;
  ros::Subscriber sub_depth_2_, sub_camera_info_2_;
  ros::Subscriber sub_depth_3_, sub_camera_info_3_;

  sensor_msgs::CameraInfo camera_info_1_, camera_info_2_, camera_info_3_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud1_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud3_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  std::mutex mutex_; 

  std::shared_ptr<active_perception_planner::FruitShapeEstimator> fruit_shape_estimator_;

  ros::ServiceServer srv_evaluate_fine_, srv_evaluate_coarse, srv_evaluate_coarse2fine, srv_save_scenecloud_;

  ros::ServiceClient get_instanceclouds_client_, get_scenecloud_client_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  std::string config_package_path_;

  std::string folder_path_;

  std::vector<std::string> headers_  = {"fruit_id", "c_x", "c_y", "c_z", "q_w", "q_x", "q_y", "q_z", "volume", "obs_c_x", "obs_c_y", "obs_c_z", "l_x", "l_y", "l_z", "fit_score", "obs_score"};
;

  std::string csv_filename_;

protected:

  bool createCSV(const std::vector<std::string>& headers, FruitMap& fruit_map, std::string filename="results.csv");
  void processAndSaveClouds(FruitMap& fruit_map);


};
}
#endif // ACTIVE_PERCEPTION_PLANNER_EVALUATION_INTERFACE_H
