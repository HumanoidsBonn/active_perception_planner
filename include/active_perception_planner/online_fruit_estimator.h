#ifndef ACTIVE_PERCEPTION_PLANNER_ONLINE_FRUIT_ESTIMATOR_H
#define ACTIVE_PERCEPTION_PLANNER_ONLINE_FRUIT_ESTIMATOR_H

#include "active_perception_planner/common.h"
#include "pointcloud_roi/detected_roi_filter.h"
#include<capsicum_superellipsoid_detector/se_detector.h>

namespace active_perception_planner
{
class OnlineFruitEstimator
{
  public:
  OnlineFruitEstimator(std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector, std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector, const DetectedFruitWithPeduncle& interested_fruit);

  inline bool isNewEstimateAvailable()
  {
    return is_new_estimate_available_;
  }
  inline DetectedFruitWithPeduncle getUpdatedEstimateofInterestedFruit()
  {
    DetectedFruitWithPeduncle fruit_with_peduncle;
    {
      fruit_with_peduncle = curr_est_interested_fruit_;
      is_new_estimate_available_ = false;
    }
    return fruit_with_peduncle;
  }

  void computeFruitEstimate();
  
  void computePeduncleEstimate();

  private: 

  std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector_;
  std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector_;
  
  DetectedFruitWithPeduncle orig_est_interested_fruit_;
  DetectedFruitWithPeduncle curr_est_interested_fruit_, prev_est_interested_fruit_;

  pointcloud_roi::SemanticInstanceDetection curr_pepper_det_, prev_pepper_det_;
  pointcloud_roi::SemanticInstanceDetection curr_peduncle_det_, prev_peduncle_det_;

  bool is_pepper_det_init_ = false;
  bool is_peduncle_det_init_ = false;
  bool is_new_estimate_available_ = false;

  double new_fruit_gain_ = 0.2;

  std::mutex mutex_;

  private:

  inline void setCurrentEstimateOfTheInterestedFruit(const DetectedFruitWithPeduncle& fruit, bool fruit_est_initialised = true)
  {
    prev_est_interested_fruit_ = curr_est_interested_fruit_;
    curr_est_interested_fruit_ = fruit;
    is_new_estimate_available_ = true;
  }

  inline void setCurrentEstimateOfPepperDetection(const pointcloud_roi::SemanticInstanceDetection& pepper_det)
  {
    if(is_pepper_det_init_ == false)
    {
      prev_pepper_det_ = pepper_det;
      is_pepper_det_init_ = true;
    }
    else
      prev_pepper_det_ = curr_pepper_det_;
    
    curr_pepper_det_ = pepper_det;
  }

  inline void setCurrentEstimateOfPeduncleDetection(const pointcloud_roi::SemanticInstanceDetection& peduncle_det)
  {
    if(is_peduncle_det_init_ == false)
    {
      prev_peduncle_det_ = peduncle_det; 
      is_peduncle_det_init_ = true; 
    }
    else
      prev_peduncle_det_ = curr_peduncle_det_;
    
    curr_peduncle_det_ = peduncle_det;
  }

  bool isDetectedFruitCurrentInterestedOne(const DetectedFruitWithPeduncle& prev_det, DetectedFruitWithPeduncle& potential_candidate, double center_threshold = 0.01);

  int findPedunclebyAssociatedFruitID(const std::vector<pointcloud_roi::SemanticInstanceDetection>& peduncles, uint32_t fruit_id);
  
  void addPeduncleToPepper(DetectedFruitWithPeduncle& fruit, const pointcloud_roi::SemanticInstanceDetection& peduncle);

  void calculatePeduncleEndpoints(Peduncle& peduncle);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double search_radius =0.03, int polynomial_order=2);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr createSurfaceCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);

  geometry_msgs::Pose filter(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, double alpha);


};



}

#endif