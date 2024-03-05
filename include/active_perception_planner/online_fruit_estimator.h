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
  OnlineFruitEstimator(ros::NodeHandle& nhp, std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector, std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector, const DetectedFruitWithPeduncle& interested_fruit);

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

  inline void setFruitGraspingPosition(const Eigen::Vector3d& pos)
  {
    use_ee_pos_ = true;
    fruit_grasping_position_ = pos;
    fruit_at_grasp_ = curr_est_interested_fruit_;
    prev_ee_pos_ = pos;
  }

  private: 

  ros::NodeHandle nhp_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector_;
  std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector_;
  
  DetectedFruitWithPeduncle orig_est_interested_fruit_;
  DetectedFruitWithPeduncle curr_est_interested_fruit_, prev_est_interested_fruit_;

  pointcloud_roi::SemanticInstanceDetection curr_pepper_det_, prev_pepper_det_;
  pointcloud_roi::SemanticInstanceDetection curr_peduncle_det_, prev_peduncle_det_;

  Peduncle best_peduncle_;
  bool is_peduncle_available_ = false;

  bool is_pepper_det_init_ = false;
  bool is_peduncle_det_init_ = false;
  bool is_new_estimate_available_ = false;

  int num_pepper_detections_missed_ = 0;
  int num_peduncle_detections_missed_ = 0;

  double new_fruit_gain_ = 0.2;

  std::mutex mutex_;

  Eigen::Vector3d fruit_grasping_position_; 
  Eigen::Vector3d prev_ee_pos_;
  bool use_ee_pos_= false;
  DetectedFruitWithPeduncle fruit_at_grasp_;

  bool ee_pos_update_available_ = false;

  ros::Publisher pub_peduncle_fruitpoint_, pub_peduncle_stempoint_, pub_fruit_top_;

  int fruit_id_of_confident_detection_ = -1;
  bool confident_id_available_ = false;

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
    num_pepper_detections_missed_ = 0;
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
    num_peduncle_detections_missed_ = 0;
  }

  bool isDetectedFruitCurrentInterestedOne(const DetectedFruitWithPeduncle& prev_det, DetectedFruitWithPeduncle& potential_candidate, double center_threshold = 0.01);

  int findPedunclebyAssociatedFruitID(const std::vector<pointcloud_roi::SemanticInstanceDetection>& peduncles, uint32_t fruit_id);
  
  void addPeduncleToPepper(DetectedFruitWithPeduncle& fruit, const pointcloud_roi::SemanticInstanceDetection& peduncle);

  void addPeduncleToPepper(DetectedFruitWithPeduncle& fruit, const Peduncle& peduncle);

  bool IsNewPeduncleGoodEnough(const Peduncle& pnew, const Peduncle& pold, double min_points = 50);

  inline void setBestPeduncle(const Peduncle& ped)
  {
    best_peduncle_ = ped;
    is_peduncle_available_ = true;
  }

  void transformPedunclePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr peduncle_cloud,
                                 const geometry_msgs::Point& p_new,
                                 const geometry_msgs::Point& p_old);

  void calculatePeduncleEndpoints(Peduncle& peduncle);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double search_radius =0.03, int polynomial_order=2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr createSurfaceCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);

  geometry_msgs::Pose filter(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, double alpha= 0.1);

  Eigen::Vector3f filter(const Eigen::Vector3f&& prev_point, const Eigen::Vector3f& curr_point, double alpha= 0.1);

  geometry_msgs::Pose updateAccordingToGripperPosition(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, double alpha = 0.1) ;


  Peduncle convert(pointcloud_roi::SemanticInstanceDetection& ped_det);

  bool getPose(geometry_msgs::TransformStamped& transform, const std::string& target_frame, const std::string& original_frame);

  template <typename PointT>
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud, float search_radius);

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr filterCloud(const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud, DetectedFruitWithPeduncle& fruit);

  void publishDebug();


};



}

#endif