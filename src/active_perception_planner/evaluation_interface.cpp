#include "active_perception_planner/evaluation_interface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include<tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "active_perception_planner/utils.h"
#include "active_perception_planner/utils_pointcloud_distances.h"

using namespace active_perception_planner;

EvaluationInterface::EvaluationInterface():nhp_("~"), tfListener_(tfBuffer_)
{
  fruit_shape_estimator_ = std::make_shared<active_perception_planner::FruitShapeEstimator>(nhp_);
  srv_evaluate_fine_ = nhp_.advertiseService("evaluate_fine_map", &EvaluationInterface::processEvaluateFineServiceCalllback, this);
  //client_stop_obj_det_ = nhp_.serviceClient<std_srvs::Empty>("/yolo_segmentation_ros_node/stop"); 
  
  get_instanceclouds_client_ = nh_.serviceClient<vpp_msgs::GetListInstancePointclouds>("/gsm_node/get_list_instance_pointclouds");

  std::string package_path = ros::package::getPath("active_perception_planner");
  std::string timestamp = utils::getCurrentTimestamp();
  folder_path_ = package_path + "/results/" + timestamp;

  // Create the results directory if it doesn't exist
  utils::createDirectory(package_path + "/results");

  // Create the timestamp folder inside the results directory
  utils::createDirectory(folder_path_);


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