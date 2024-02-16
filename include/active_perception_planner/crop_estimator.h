#ifndef ACTIVE_PERCEPTION_PLANNER_CROP_ESTIMATOR_H
#define ACTIVE_PERCEPTION_PLANNER_CROP_ESTIMATOR_H

#include "active_perception_planner/common.h"

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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace active_perception_planner
{
  class CropEstimator
  {
    public:
    CropEstimator();


    Crop createCrop(const FruitMap& fruit_map, const std::unordered_map<uint32_t, Peduncle>& peduncle_map, const std::vector<Stem>& stems);

    private:

    

    std::unordered_map<uint32_t, DetectedFruitWithPeduncle> attachDetectedPedunclesToFruits(const FruitMap& fruit_map, const std::unordered_map<uint32_t, Peduncle>& peduncle_map);

    void associateStemsWithFruits(std::unordered_map<uint32_t, DetectedFruitWithPeduncle> &fruit_with_peduncle_map, const std::vector<Stem> &stems, const float &belonging_threshold, const float &nearness_threshold);

    void calculatePeduncleEndpoints(Peduncle& peduncle);

    void estimatePeduncleEndpointsWithNearestStem(DetectedFruitWithPeduncle& fruit_with_peduncle, const Stem& stem);

    bool isPeduncleAtTopCenterOfFruit(superellipsoid::DetectedFruit& fruit, Peduncle& peduncle, const float& threshold = 0.02);

    float pointLineDistance(const geometry_msgs::Point& fruit_centre, const Eigen::Vector3f& line_origin, const Eigen::Vector3f& line_direction);

    Eigen::Vector3f findIntersection(const Eigen::Vector3f& outside_point, const Eigen::Vector3f& line_origin, const Eigen::Vector3f& line_direction);


  };
}

#endif