#include "active_perception_planner/peduncle_estimator.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
using namespace active_perception_planner;

PeduncleEstimator::PeduncleEstimator()
{
  is_running_ = true;

  processing_thread_ = std::thread(&PeduncleEstimator::processingThread, this);
}

PeduncleEstimator::~PeduncleEstimator()
{
  is_running_ = false;
  cv_.notify_one();
  try
  {
    processing_thread_.join();
  }
  catch (std::system_error&) {}
}

std::unordered_map<uint32_t, Peduncle> PeduncleEstimator::getPeduncles()
{
  std::unordered_map<uint32_t, Peduncle>  peduncles;
  std::scoped_lock(mutex_peduncles_);
  peduncles = peduncle_map_;
  return peduncles;
}

void PeduncleEstimator::semanticInstanceDetectionsCallback(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets)
{
  ROS_INFO("PeduncleEstimator detectionCallback");
  {
    //ROS_INFO("PeduncleEstimator 1");
    std::lock_guard<std::mutex> lk(mutex_dets_);
    //ROS_INFO("PeduncleEstimator 2");
    if(dets == nullptr)
    {
      ROS_WARN("SemanticInstanceDetections: nullptr");
      return;
    }
    dets_ = dets;
  }
  cv_.notify_one();
}

void PeduncleEstimator::processingThread()
{
  while (is_running_ && ros::ok())
  {
    //ROS_WARN_STREAM("PeduncleEstimator: Is running");
    std::unique_lock<std::mutex> lk(mutex_dets_);
    cv_.wait(lk, [this]{return this->dets_ != nullptr || !this->is_running_;});
    if (!this->is_running_)
      return;
    
    std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> current_dets = dets_;
    dets_ = nullptr;

    processPeduncleDetections(current_dets);

    lk.unlock();
  }
}

void PeduncleEstimator::processPeduncleDetections(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets)
{
  rgb_image_ = dets->rgb_image.clone();
  depth_image_ = dets->depth_image.clone();

  std::vector<pointcloud_roi::SemanticInstanceDetection> curr_peduncle_dets; 
  curr_peduncle_dets.insert(curr_peduncle_dets.begin(), dets->peduncles.begin(), dets->peduncles.end());

  mergePedunclesByPepperID(curr_peduncle_dets);
}

void PeduncleEstimator::mergePedunclesByPepperID(const std::vector<pointcloud_roi::SemanticInstanceDetection> curr_peduncle_dets)
{
 
  for(const pointcloud_roi::SemanticInstanceDetection& peduncle_det: curr_peduncle_dets)
  {
    if(peduncle_det.cloud_segment == nullptr || peduncle_det.cloud_segment->empty())
      continue;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(peduncle_det.cloud_segment);
    sor.setMeanK(30); // Set the number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(2.0); // Set the standard deviation multiplier threshold
    sor.filter(*peduncle_det.cloud_segment);
    
    uint32_t pepper_id = peduncle_det.associated_instance;
    if(peduncle_map_.find(pepper_id) == peduncle_map_.end())
    {
      peduncle_map_[pepper_id].peduncle_cloud = peduncle_det.cloud_segment;
      peduncle_map_[pepper_id].associated_fruit_id = pepper_id;
      peduncle_map_[pepper_id].peduncle_id = peduncle_det.instance_id;
    }
    else
    {
      *(peduncle_map_[pepper_id].peduncle_cloud) += *(peduncle_det.cloud_segment);
    }
  }
}