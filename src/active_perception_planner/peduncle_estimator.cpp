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

void PeduncleEstimator::calculatePeduncleEndpoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& fruitpoint, Eigen::Vector3f& stempoint)
{
  std::sort(cloud->begin(), cloud->end(), [](const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b) {
        return a.z < b.z;
  });

  fruitpoint = cloud->front().getVector3fMap();
  stempoint  = cloud->back().getVector3fMap();

}

bool PeduncleEstimator::IsThereOverlapBetweenPeduncles(const Peduncle& existing_peduncle, Eigen::Vector3f& new_fp, const Eigen::Vector3f& new_sp)
{
  double dist_between_fruitpoints = (existing_peduncle.fruitpoint - new_fp).norm();
  double dist_between_stempoints  = (existing_peduncle.stempoint - new_sp).norm();

  if(dist_between_fruitpoints < 0.03 && dist_between_stempoints < 0.03)
    return true;

  return false;
}

void PeduncleEstimator::mergePedunclesByPepperID(std::vector<pointcloud_roi::SemanticInstanceDetection>& curr_peduncle_dets)
{
 
  for(pointcloud_roi::SemanticInstanceDetection& peduncle_det: curr_peduncle_dets)
  {
    if(peduncle_det.cloud_segment == nullptr || peduncle_det.cloud_segment->empty())
      continue;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(peduncle_det.cloud_segment);
    sor.setMeanK(30); // Set the number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(2.0); // Set the standard deviation multiplier threshold
    sor.filter(*peduncle_det.cloud_segment);
    
    uint32_t pepper_id = peduncle_det.associated_instance;
    auto it = peduncle_map_.find(pepper_id);
    if(it == peduncle_map_.end())
    {
      peduncle_map_[pepper_id].peduncle_cloud = peduncle_det.cloud_segment;
      peduncle_map_[pepper_id].associated_fruit_id = pepper_id;
      peduncle_map_[pepper_id].peduncle_id = peduncle_det.instance_id;
      calculatePeduncleEndpoints(peduncle_map_[pepper_id].peduncle_cloud, peduncle_map_[pepper_id].fruitpoint, peduncle_map_[pepper_id].stempoint);
    }
    else
    {
      Eigen::Vector3f ped_det_fp, ped_det_sp;
      calculatePeduncleEndpoints(peduncle_det.cloud_segment, ped_det_fp, ped_det_sp);
      if(IsThereOverlapBetweenPeduncles(it->second, ped_det_fp, ped_det_sp))
        *(peduncle_map_[pepper_id].peduncle_cloud) += *(peduncle_det.cloud_segment);
    }
  }
}