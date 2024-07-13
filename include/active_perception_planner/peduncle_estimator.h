#ifndef ACTIVE_PERCEPTION_PLANNER_PEDUNCLE_ESTIMATOR_H
#define ACTIVE_PERCEPTION_PLANNER_PEDUNCLE_ESTIMATOR_H

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
#include <unordered_set>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace active_perception_planner
{

class PeduncleEstimator
{
  public:

  PeduncleEstimator();

  virtual ~PeduncleEstimator();

  void semanticInstanceDetectionsCallback(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets);

  std::unordered_map<uint32_t, Peduncle> getPeduncles();

  private:

  void processingThread();

  //void setCurrentPeduncleDetections(std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets); 

  void processPeduncleDetections(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets);

  void mergePedunclesByPepperID(std::vector<pointcloud_roi::SemanticInstanceDetection>& curr_peduncle_dets);

  void calculatePeduncleEndpoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& fruitpoint, Eigen::Vector3f& stempoint);
  bool IsThereOverlapBetweenPeduncles(const Peduncle& existing_peduncle, Eigen::Vector3f& new_fp, const Eigen::Vector3f& new_sp);


  std::vector<pointcloud_roi::SemanticInstanceDetection> peduncle_detections_;
  std::unordered_map<uint32_t, Peduncle> peduncle_map_;
  cv::Mat rgb_image_;
  cv::Mat depth_image_;


  std::mutex mutex_peduncles_;
  std::condition_variable cv_;
  std::thread processing_thread_;
  std::atomic_bool is_running_;
  std::mutex mutex_dets_;

  std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets_;

};

}

#endif
