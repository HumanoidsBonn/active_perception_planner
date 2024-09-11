#ifndef ACTIVE_PERCEPTION_PLANNER_FRUIT_SHAPE_ESTIMATOR_H
#define ACTIVE_PERCEPTION_PLANNER_FRUIT_SHAPE_ESTIMATOR_H

#include "active_perception_planner/common.h"

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include<vector>
#include<memory>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <opencv2/opencv.hpp>
#include <pointcloud_roi/detected_roi_filter.h>
#include <unordered_set>

#include<capsicum_superellipsoid_detector/se_detector.h>
#include <pointcloud_roi/detected_roi_filter.h>
#include "active_perception_planner/common.h"
#include <pointcloud_roi_msgs/PointCloudInstances.h>

#include<vpp_msgs/GetScenePointcloud.h>
#include<vpp_msgs/GetListInstancePointclouds.h>
#include<pcl_conversions/pcl_conversions.h>

namespace active_perception_planner
{

class FruitShapeEstimator
{
public:

  FruitShapeEstimator(ros::NodeHandle& nhp);
  bool computeFruitShapes(const std::vector<vpp_msgs::InstancePointcloudwithCentroid>& instance_clouds_with_centroid);
  bool computeFruitShapes(const pointcloud_roi_msgs::PointCloudInstances& pointcloud_instances);
  int removeFalsePositives();

  inline FruitMap getFruitMap()
  {
    return fruit_map_;
  }

protected:
  ros::NodeHandle nh_, nhp_;
  std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<tf::TransformListener> listener_;

  std::vector<superellipsoid::DetectedFruit> fruits_list_;
  FruitMap fruit_map_;
  bool fruit_map_available_ = false;

  double fruit_separation_xycenter_diff_;
  double fruit_separation_z_diff_;


  bool callEmptyService(ros::ServiceClient& client);

};
}// namespace active_perception_planner

#endif // ACTIVE_PERCEPTION_PLANNER_FRUIT_SHAPE_ESTIMATOR_H
