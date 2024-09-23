#include "active_perception_planner/evaluation_interface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include<tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "active_perception_planner/utils.h"
#include "active_perception_planner/utils_pointcloud_distances.h"
#include <vpp_msgs/GetScenePointcloud.h>

using namespace active_perception_planner;

EvaluationInterface::EvaluationInterface():nhp_("~"), tfListener_(tfBuffer_)
{
  fruit_shape_estimator_ = std::make_shared<active_perception_planner::FruitShapeEstimator>(nhp_);
  srv_evaluate_fine_ = nhp_.advertiseService("evaluate_fine_map", &EvaluationInterface::processEvaluateFineServiceCalllback, this);
  srv_save_scenecloud_ = nhp_.advertiseService("save_scene_cloud", &EvaluationInterface::saveScenePointCloudCallback, this);
  //client_stop_obj_det_ = nhp_.serviceClient<std_srvs::Empty>("/yolo_segmentation_ros_node/stop"); 
  
  get_instanceclouds_client_ = nh_.serviceClient<vpp_msgs::GetListInstancePointclouds>("/gsm_node/get_list_instance_pointclouds");
  get_scenecloud_client_ = nh_.serviceClient<vpp_msgs::GetScenePointcloud>("/gsm_node/get_scene_pointcloud");

  std::string package_path = ros::package::getPath("active_perception_planner");
  std::string timestamp = utils::getCurrentTimestamp();
  folder_path_ = package_path + "/results/" + timestamp;

  // Create the results directory if it doesn't exist
  utils::createDirectory(package_path + "/results");

  // Create the timestamp folder inside the results directory
  utils::createDirectory(folder_path_);

  sub_depth_1_ = nh_.subscribe("/trollomatic/camera1/aligned_depth_to_color/image_raw", 1, &EvaluationInterface::depthCallback1, this);
  sub_camera_info_1_ = nh_.subscribe("/trollomatic/camera2/color/camera_info", 1, &EvaluationInterface::cameraInfoCallback1, this);

  sub_depth_2_ = nh_.subscribe("/trollomatic/camera2/aligned_depth_to_color/image_raw", 1, &EvaluationInterface::depthCallback2, this);
  sub_camera_info_2_ = nh_.subscribe("/trollomatic/camera2/color/camera_info", 1, &EvaluationInterface::cameraInfoCallback2, this);

  sub_depth_3_ = nh_.subscribe("/trollomatic/camera3/aligned_depth_to_color/image_raw", 1, &EvaluationInterface::depthCallback3, this);
  sub_camera_info_3_ = nh_.subscribe("/trollomatic/camera3/color/camera_info", 1, &EvaluationInterface::cameraInfoCallback3, this);


  ROS_INFO_ONCE("EvaluationInterface");
}

bool EvaluationInterface::processEvaluateFineServiceCalllback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  vpp_msgs::GetListInstancePointclouds instcloud_srv;

  ROS_INFO("Ready to evaluate");

  ROS_WARN("Calling GetListInstancePointclouds Service ...");
  instcloud_srv.request.get_specific_instances = false;
  if(!get_instanceclouds_client_.call(instcloud_srv))
  {
    ROS_ERROR("Failed to call GetListInstancePointclouds vpp map service");
    return false;
  }
  ROS_WARN("GetListInstancePointclouds Service Call Success ");
  if(instcloud_srv.response.instance_clouds_with_centroid.empty())
  {
    ROS_ERROR("Failed to detect fruits in the 3D map");
    return false;
  }
  ROS_DEBUG("3");
  if(fruit_shape_estimator_->computeFruitShapes(instcloud_srv.response.instance_clouds_with_centroid) == false)
  {
    ROS_ERROR("Failed to complete fruit shapes");
    return false;
  }
  ROS_DEBUG("4");
  if(fruit_shape_estimator_->removeFalsePositives() < 1)
  {
    ROS_ERROR("False Positive Removal Failure");
    return false;
  }
  auto fruit_map = fruit_shape_estimator_->getFruitMap();

  if(saveResults(fruit_map))
    return true;
}

bool EvaluationInterface::saveScenePointCloudCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  std::string package_path = ros::package::getPath("active_perception_planner");
  std::string timestamp = utils::getCurrentTimestamp();
  std::string folder_path = package_path + "/results/" + timestamp;

  // Create the results directory if it doesn't exist
  utils::createDirectory(package_path + "/results");

  // Create the timestamp folder inside the results directory
  utils::createDirectory(folder_path);

  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Thread-safe access to shared point clouds
  std::lock_guard<std::mutex> lock(mutex_);
  *stitched_cloud = *pointcloud1_ + *pointcloud2_ + *pointcloud3_;

  // Convert stitched cloud to sensor_msgs::PointCloud2
 
  // Save concatenated XYZ cloud
  utils::savePointCloud<pcl::PointXYZ>(stitched_cloud, "scene_pointcloud", folder_path);

  return true;

}

bool EvaluationInterface::createCSV(const std::vector<std::string>& headers, FruitMap& fruit_map, std::string filename) 
{
  std::string full_filename = folder_path_ + "/" + filename;
  std::ofstream file(full_filename);

  if (!file.is_open()) 
  {
    std::cerr << "Error opening file for writing: " << filename << std::endl;
    return false;
  }

    // Write headers
  for (size_t i = 0; i < headers.size(); ++i) 
  {
    file << headers[i];
    if (i < headers.size() - 1) 
    {
      file << ",";
    }
  }
  file << "\n";

  // Iterate over the map and write the values to the CSV file
  //  headers_ = {"fruit_id", "c_x", "c_y", "c_z", "q_w", "q_x", "q_y", "q_z", "volume", "obs_c_x", "obs_c_y", "obs_c_z", "l_x", "l_y", "l_z", "fit_score", "obs_score"};

  for (const auto& pair : fruit_map) {
    const superellipsoid::DetectedFruit& data = pair.second;

    file  << data.fruit_id << ","    // fruit_id
          << data.fruit_pose.position.x << "," // c_x
          << data.fruit_pose.position.y << "," // c_y
          << data.fruit_pose.position.z << "," // c_z
          << data.fruit_pose.orientation.w << "," // q_w
          << data.fruit_pose.orientation.x << "," // q_x
          << data.fruit_pose.orientation.y << "," // q_y
          << data.fruit_pose.orientation.z << "," // q_z
          << data.volume <<"," // volume
          << data.observed_shape_centroid.x << "," //obs_c_x
          << data.observed_shape_centroid.y << "," //obs_c_y
          << data.observed_shape_centroid.z << "," //obs_c_z
          << data.fruit_size.x << "," //l_x
          << data.fruit_size.y << "," //l_y
          << data.fruit_size.z << "," //l_z
          << data.fit_score << ","    //fit_score
          << data.observation_score <<"\n"; // obs_score
  }

  file.close();
  std::cout << "CSV file created successfully: " << filename << std::endl;
  return true;
}

// Main function to process the map, save individual clouds, and concatenate them
void EvaluationInterface::processAndSaveClouds(FruitMap& fruit_map) 
{
  // Create point clouds to hold concatenated results
  pcl::PointCloud<pcl::PointXYZ>::Ptr concatenated_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenated_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Iterate over the map
  for (const auto& entry : fruit_map) 
  {
    const superellipsoid::DetectedFruit& data = entry.second;
    std::string base_filename = "fruit_" + std::to_string(data.fruit_id);

    // Save individual XYZ cloud
    if (data.completed_fruit_shape && !data.completed_fruit_shape->empty()) 
    {
      utils::savePointCloud<pcl::PointXYZ>(data.completed_fruit_shape, base_filename + "_predicted", folder_path_);
      *concatenated_cloud_xyz += *data.completed_fruit_shape;  // Concatenate XYZ
    }

    // Save individual XYZRGB cloud
    if (data.observed_fruit_shape && !data.observed_fruit_shape->empty()) 
    {
      utils::savePointCloud<pcl::PointXYZRGB>(data.observed_fruit_shape, base_filename + "_observed", folder_path_);
      *concatenated_cloud_xyzrgb += *data.observed_fruit_shape;  // Concatenate XYZRGB
    }
  }
  // Save concatenated XYZ cloud
  utils::savePointCloud<pcl::PointXYZ>(concatenated_cloud_xyz, "concatenated_predicted", folder_path_);

  // Save concatenated XYZRGB cloud
  utils::savePointCloud<pcl::PointXYZRGB>(concatenated_cloud_xyzrgb, "concatenated_observed", folder_path_);
}

// Callbacks for each camera
void EvaluationInterface::depthCallback1(const sensor_msgs::ImageConstPtr& depth_msg)
{
  processDepthImage(depth_msg, camera_info_1_, "map", pointcloud1_);
}

void EvaluationInterface::cameraInfoCallback1(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  camera_info_1_ = *info_msg;
}

void EvaluationInterface::depthCallback2(const sensor_msgs::ImageConstPtr& depth_msg)
{
  processDepthImage(depth_msg, camera_info_2_, "map", pointcloud2_);
}

void EvaluationInterface::cameraInfoCallback2(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  camera_info_2_ = *info_msg;
}

void EvaluationInterface::depthCallback3(const sensor_msgs::ImageConstPtr& depth_msg)
{
    processDepthImage(depth_msg, camera_info_3_, "map", pointcloud3_);
}

void EvaluationInterface::cameraInfoCallback3(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  camera_info_3_ = *info_msg;
}


bool EvaluationInterface::saveResults(FruitMap& fruit_map)
{
  if(createCSV(headers_, fruit_map) == false)
  {
    ROS_ERROR("Could not create CSV file");
    return false;
  }
  processAndSaveClouds(fruit_map);
  return true;
}

void EvaluationInterface::processDepthImage(const sensor_msgs::ImageConstPtr& depth_msg, 
                           const sensor_msgs::CameraInfo& camera_info, 
                           const std::string& frame_id, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
  // Convert depth image to point cloud
  ROS_WARN_STREAM_THROTTLE(1.0, "depth_msg header: "<<depth_msg->header.frame_id);
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud = depthToPointCloud(depth_msg, camera_info);

  // Transform the point cloud to the color optical frame
  transformPointCloud(new_cloud, frame_id, depth_msg->header);

  // Downsample the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud = downsamplePointCloud(new_cloud);

  // Thread-safe access to shared point cloud
  std::lock_guard<std::mutex> lock(mutex_);
  *pointcloud += *downsampled_cloud;
}

void EvaluationInterface::transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string target_frame, std_msgs::Header header)
{
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    ros::Duration timeout(0.1);
    transform_stamped = tfBuffer_.lookupTransform(target_frame, header.frame_id, header.stamp,timeout);
    //ROS_WARN_STREAM_THROTTLE(1.0, "")
    Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped);
    pcl::transformPointCloud(*cloud, *cloud, transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("Transform error: %s", ex.what());
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EvaluationInterface::downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);  // Set voxel grid resolution

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*downsampled_cloud);
  return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EvaluationInterface::depthToPointCloud(const sensor_msgs::ImageConstPtr& depth_msg, 
                                                      const sensor_msgs::CameraInfo& camera_info)
{
  // Create a new point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Ensure the depth image is of the correct format (e.g., 16UC1 or 32FC1)
  if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 && 
      depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
      ROS_ERROR("Unsupported depth image encoding: %s", depth_msg->encoding.c_str());
      return cloud;
  }

  // Extract camera intrinsic parameters from CameraInfo
  const double fx = camera_info.K[0];  // Focal length in x
  const double fy = camera_info.K[4];  // Focal length in y
  const double cx = camera_info.K[2];  // Optical center in x
  const double cy = camera_info.K[5];  // Optical center in y

  const unsigned int width = depth_msg->width;
  const unsigned int height = depth_msg->height;

  // Resize point cloud to fit the depth image size
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;  // Set to false as depth image may contain invalid points
  cloud->points.resize(width * height);
  cloud->header.frame_id = depth_msg->header.frame_id;

  // Iterate over depth image pixels and project to 3D
  for (unsigned int v = 0; v < height; ++v)
  {
    for (unsigned int u = 0; u < width; ++u)
    {
      // Get depth value for pixel (u, v)
      float depth = 0.0f;
      if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
        depth = static_cast<float>(depth_row[v * width + u]) * 0.001f;  // Convert depth to meters
      }
      else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
        depth = depth_row[v * width + u];
      }

      // Skip invalid depth values (commonly represented by 0 or NaN)
      if (depth <= 0.0f || std::isnan(depth) || depth > 1.3)
      {
        continue;
      }

      // Calculate 3D point (X, Y, Z) from pixel (u, v) and depth
      pcl::PointXYZ& point = cloud->points[v * width + u];
      point.x = (u - cx) * depth / fx;
      point.y = (v - cy) * depth / fy;
      point.z = depth;
    }
  }
// Remove NaN points from the point cloud
  std::vector<int> indices;  // Indices of the valid points
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  return cloud;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_map_node");
  ros::NodeHandle nh; 
  ros::AsyncSpinner spinner(0);
  EvaluationInterface eval_interface;
  ROS_INFO("Ready to Evaluate.");
  spinner.start();

  ros::waitForShutdown();

  return 0;
}