#include "active_perception_planner/peduncle_estimator.h"
using namespace active_perception_planner;

PeduncleEstimator::PeduncleEstimator()
{

}

void PeduncleEstimator::semanticInstanceDetectionsCallback(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets)
{
  ROS_WARN("PeduncleEstimator detectionCallback");
  {
    std::lock_guard<std::mutex> lk(mutex_dets_);
    dets_ = dets;
  }
  cv_.notify_one();
}

void PeduncleEstimator::processingThread()
{
  while (is_running_ && ros::ok())
  {
    ROS_WARN_STREAM("PeduncleEstimator: Is running");
    std::unique_lock<std::mutex> lk(mutex_);
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