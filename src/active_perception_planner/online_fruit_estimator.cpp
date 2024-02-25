#include "active_perception_planner/online_fruit_estimator.h"
#include "active_perception_planner/common.h"
using namespace active_perception_planner;

OnlineFruitEstimator::OnlineFruitEstimator(std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector, const DetectedFruitWithPeduncle& interested_fruit):roi_detector_(roi_detector), orig_est_interested_fruit_(interested_fruit), prev_est_interested_fruit_(interested_fruit)
{
  roi_detector_->enableContinuousDetections(true);
}

void OnlineFruitEstimator::computeFruitEstimate()
{
  std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> detections = nullptr;
  detections = roi_detector_->readSemanticInstanceDetections();

  if(detections == nullptr)
  {
    is_new_estimate_available_ = false;
    return;
  }

}