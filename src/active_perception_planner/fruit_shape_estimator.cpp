#include "active_perception_planner/fruit_shape_estimator.h"
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

FruitShapeEstimator::FruitShapeEstimator(ros::NodeHandle& nhp):nhp_(nhp),
  tf_buffer_(ros::Duration(30)),
  tf_listener_(tf_buffer_)
{
  nhp_.param<double>("fruit_separation_z_diff", fruit_separation_z_diff_, 0.04);
  nhp_.param<double>("fruit_separation_xycenter_diff", fruit_separation_xycenter_diff_, 0.06);
  se_detector_ = std::make_shared<superellipsoid::SuperellipsoidDetector>();
  listener_ = std::make_unique<tf::TransformListener>();

  ROS_INFO("Fruit Shape Estimator");

}

bool FruitShapeEstimator::callEmptyService(ros::ServiceClient& client)
{
  std_srvs::Empty empty_srv;
  if (!client.call(empty_srv))
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
  return true;
}

bool FruitShapeEstimator::computeFruitShapes(const std::vector<vpp_msgs::InstancePointcloudwithCentroid>& instance_clouds_with_centroid)
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

int FruitShapeEstimator::removeFalsePositives()
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