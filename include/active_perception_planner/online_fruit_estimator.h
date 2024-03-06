#ifndef ACTIVE_PERCEPTION_PLANNER_ONLINE_FRUIT_ESTIMATOR_H
#define ACTIVE_PERCEPTION_PLANNER_ONLINE_FRUIT_ESTIMATOR_H

#include "active_perception_planner/common.h"
#include "pointcloud_roi/detected_roi_filter.h"
#include<capsicum_superellipsoid_detector/se_detector.h>
#include "active_perception_planner/utils.h"

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
namespace active_perception_planner
{

template<typename PointT>
class PointCloudBuffer {
private:
  std::deque<std::pair<typename pcl::PointCloud<PointT>::Ptr, ros::Time>> buffer;
  int buffer_size; // Maximum number of frames in buffer
  double max_age;  // Maximum age of a frame in seconds
  double search_radius = 0.01f;
  int polynomial_order = 2;
  ;
public:
  PointCloudBuffer(int buffer_size, double max_age) : buffer_size(buffer_size), max_age(max_age) {}

  void addFrame(const typename pcl::PointCloud<PointT>::Ptr& frame, const ros::Time& timestamp) {
      // Check if the frame is empty or null
      if (frame->empty() || frame == nullptr) {
          ROS_WARN_THROTTLE(2, "Empty or null point cloud frame, skipping.");
          return;
      }

      // Check if the frame is too old
      ros::Time current_time = ros::Time::now();
      if ((current_time - timestamp).toSec() > max_age) {
          ROS_WARN_THROTTLE(2, "Frame too old, skipping.");
          return;
      }

      // Add frame to buffer
      if (buffer.size() < buffer_size) {
          buffer.push_front(std::make_pair(frame, timestamp));
      } else {
          ROS_WARN_THROTTLE(2, "Buffer full, dropping oldest frame.");
          buffer.pop_back(); // Remove oldest frame
          buffer.push_front(std::make_pair(frame, timestamp));
      }
  }

  typename pcl::PointCloud<PointT>::Ptr getMergedSmoothedFrame() 
  {
    // Check if all frames in the buffer are too old
    bool all_frames_old = true;
    for (auto it = buffer.rbegin(); it != buffer.rend(); ++it) 
    {
      if ((ros::Time::now() - it->second).toSec() <= max_age) 
      {
        all_frames_old = false;
        break; // No need to check frames more in the front
      }
    }
    // Check if all frames in the buffer are too old
    if (all_frames_old) 
    {
      ROS_WARN_THROTTLE(2, "All frames in buffer are too old, returning nullptr.");
      return nullptr;
    }
    // Merge all valid frames in the buffer
    typename pcl::PointCloud<PointT>::Ptr merged_cloud(new pcl::PointCloud<PointT>);
    for (const auto& frame : buffer) 
    {
      *merged_cloud += *frame.first;
    }
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(merged_cloud);
    // Smooth merged cloud with MLS filter
    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setComputeNormals(false); // Disable normal estimation
    mls.setInputCloud(merged_cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius); // Adjust this value as needed
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(polynomial_order); // Adjust this value as needed
    typename pcl::PointCloud<PointT>::Ptr smoothed_cloud(new pcl::PointCloud<PointT>);
    mls.process(*smoothed_cloud);

    return smoothed_cloud;
  }
};

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

  bool computeFruitEstimate();
  
  bool computePeduncleEstimate();

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

  std::unique_ptr<PointCloudBuffer<pcl::PointXYZRGB>> peduncle_buffer_;

  Peduncle best_peduncle_;
  bool is_peduncle_available_ = false;

  bool is_pepper_det_init_ = false;
  bool is_peduncle_det_init_ = false;
  bool is_new_estimate_available_ = false;
  bool is_new_peduncle_estimate_available_ = true;

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
  ros::Publisher pub_peduncle_cloud_;

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

  Vector6d filterUsingVariances(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, const Eigen::Vector3d& prev_var, const Eigen::Vector3d& curr_var, double alpha = 0.7);

  geometry_msgs::Pose updateAccordingToGripperPosition(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, double alpha = 0.1) ;


  Peduncle convert(pointcloud_roi::SemanticInstanceDetection& ped_det);

  bool getPose(geometry_msgs::TransformStamped& transform, const std::string& target_frame, const std::string& original_frame);

  template <typename PointT>
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud, float search_radius);

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr filterCloud(const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud, DetectedFruitWithPeduncle& fruit);

  void publishDebug();

  bool isPeduncleCloudValid(const pointcloud_roi::SemanticInstanceDetection& peduncle);

};



}

#endif