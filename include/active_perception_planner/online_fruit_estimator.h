#ifndef ACTIVE_PERCEPTION_PLANNER_ONLINE_FRUIT_ESTIMATOR_H
#define ACTIVE_PERCEPTION_PLANNER_ONLINE_FRUIT_ESTIMATOR_H

#include "active_perception_planner/common.h"
#include "pointcloud_roi/detected_roi_filter.h"

namespace active_perception_planner
{
class OnlineFruitEstimator
{
  public:
  OnlineFruitEstimator(std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector, const DetectedFruitWithPeduncle& interested_fruit);

  inline bool isNewEstimateAvailable()
  {
    return is_new_estimate_available_;
  }
  DetectedFruitWithPeduncle getUpdatedEstimateofInterestedFruit();

  void computeFruitEstimate();
  
  void computePeduncleEstimate();

  private: 

  std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector_;
  
  DetectedFruitWithPeduncle orig_est_interested_fruit_;
  DetectedFruitWithPeduncle curr_est_interested_fruit_, prev_est_interested_fruit_;

  bool is_new_estimate_available_ = false;
  
};


}

#endif