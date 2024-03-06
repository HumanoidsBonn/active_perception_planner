#include "active_perception_planner/online_fruit_estimator.h"
#include "active_perception_planner/common.h"
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


using namespace active_perception_planner;

OnlineFruitEstimator::OnlineFruitEstimator(ros::NodeHandle& nhp, std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector, std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector, const DetectedFruitWithPeduncle& interested_fruit):nhp_(nhp), roi_detector_(roi_detector), se_detector_(se_detector), orig_est_interested_fruit_(interested_fruit), prev_est_interested_fruit_(interested_fruit), curr_est_interested_fruit_(interested_fruit)
{
  if(orig_est_interested_fruit_.has_peduncle)
  {
    is_peduncle_available_ = true;
    best_peduncle_ = orig_est_interested_fruit_.peduncle;
    best_peduncle_.associated_fruit_pose = orig_est_interested_fruit_.fruit.fruit_pose;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(best_peduncle_.peduncle_cloud);
    sor.setMeanK(30); // Set the number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(1.0); // Set the standard deviation multiplier threshold
    sor.filter(*best_peduncle_.peduncle_cloud);
  }
  roi_detector_->enableContinuousDetections(false);

  pub_peduncle_fruitpoint_  = nhp_.advertise<geometry_msgs::Point>("peduncle_fruitpoint", 1, false);
  pub_peduncle_stempoint_   = nhp_.advertise<geometry_msgs::Point>("peduncle_stempoint", 1, false);
  pub_fruit_top_            = nhp_.advertise<geometry_msgs::Point>("fruit_top_", 1, false);
}

void OnlineFruitEstimator::computeFruitEstimate()
{
  std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> detections = nullptr;
  roi_detector_->enableTrackedDetectionCallback(true);
  auto t_start = ros::Time::now();
  sleep(0.05);
  do
  { //tracked_dets_available = roi_detector_->isTrackedDetectionAvailable();
    detections = roi_detector_->readSemanticInstanceDetections();
    sleep(0.05);
  }
  while(detections == nullptr && (ros::Time::now()-t_start).toSec() < 0.25);


  if(detections == nullptr)
  {
    is_new_estimate_available_ = false;
    return;
  }
  
  DetectedFruitWithPeduncle nearest_fruit; 
  int nearest_fruit_id = -1;
  double nearest_fruit_distance = 1000;
  for(int i = 0; i < detections->peppers.size(); ++i)
  {
    auto fruit_det = detections->peppers[i];
    if(fruit_det.cloud_segment == nullptr)
    {
      ROS_WARN_THROTTLE(2.0, "Online Fruit Estimator: Fruit cloud segment nullptr");
      continue;
    }
    if(fruit_det.cloud_segment->size() < 10)
    {
      ROS_WARN_THROTTLE(2.0, "Online Fruit Estimator: Fruit cloud segment too small");
      continue;
    }
    DetectedFruitWithPeduncle fruit_with_peduncle;
    auto surface_cloud = smoothPointCloud(fruit_det.cloud_segment);
    if(se_detector_->completeFruitShape(surface_cloud, fruit_with_peduncle.fruit))
    {
      Eigen::Vector3d c1, c2, c3;
      tf::pointMsgToEigen(curr_est_interested_fruit_.fruit.fruit_pose.position, c1);
      tf::pointMsgToEigen(fruit_with_peduncle.fruit.fruit_pose.position, c2);
      tf::pointMsgToEigen(orig_est_interested_fruit_.fruit.fruit_pose.position, c3);
      ROS_INFO_STREAM("New Fruit Centre: "<<c1.transpose()<<" Prev Fruit Centre: "<<c2.transpose()<< "Orig Fruit Centre: "<<c3.transpose());
      double centre_dist = utils::computeFruitCentreDistance(curr_est_interested_fruit_.fruit.fruit_pose.position, fruit_with_peduncle.fruit.fruit_pose.position);
      if(centre_dist < 0.02)
      {
        nearest_fruit_id = i;
        nearest_fruit_distance = centre_dist;
        nearest_fruit = fruit_with_peduncle;
        break;
      }
      else if(centre_dist < 0.04 && centre_dist < nearest_fruit_distance)
      {
        nearest_fruit = fruit_with_peduncle;
        nearest_fruit_distance = centre_dist;
        nearest_fruit_id = i;
      }
    }
  }
  if(nearest_fruit_id > -1)
  {
    num_pepper_detections_missed_ = 0;
    pointcloud_roi::SemanticInstanceDetection nearest_pepper_det = detections->peppers[nearest_fruit_id];
    setCurrentEstimateOfPepperDetection(nearest_pepper_det);
    int peduncle_id = findPedunclebyAssociatedFruitID(detections->peduncles, nearest_pepper_det.instance_id);
    auto inst_fruit_pose = nearest_fruit.fruit.fruit_pose;
    nearest_fruit.fruit.fruit_pose = filter(prev_est_interested_fruit_.fruit.fruit_pose, nearest_fruit.fruit.fruit_pose, 0.1);
    nearest_fruit.fruit.fruit_size = orig_est_interested_fruit_.fruit.fruit_size;
    Peduncle selected_peduncle;
    bool use_old_peduncle = false;
    if(peduncle_id > -1)
    {
      num_peduncle_detections_missed_ = 0;
      pointcloud_roi::SemanticInstanceDetection associated_peduncle_det = detections->peduncles[peduncle_id];
      Peduncle associated_peduncle;
      associated_peduncle = convert(associated_peduncle_det);
      setCurrentEstimateOfPeduncleDetection(associated_peduncle_det);
      if(IsNewPeduncleGoodEnough(associated_peduncle, best_peduncle_))
      {
        ROS_INFO_STREAM("HURRAH New Peduncle good enough: "<<associated_peduncle.peduncle_cloud->size());
        selected_peduncle = associated_peduncle;
        transformPedunclePointCloud(selected_peduncle.peduncle_cloud, nearest_fruit.fruit.fruit_pose.position, inst_fruit_pose.position);
        //setBestPeduncle(selected_peduncle);
        selected_peduncle.associated_fruit_pose = nearest_fruit.fruit.fruit_pose;
        use_old_peduncle = false;
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "New Peduncle not good enough");
        if(is_peduncle_available_)
          use_old_peduncle = true;
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Cannot find peduncle in current detection");
      if(is_peduncle_available_)
        use_old_peduncle = true;
    }
    if(use_old_peduncle)
    {
      ROS_INFO_STREAM("Using old peduncle detection");
      selected_peduncle = best_peduncle_;
      //transformPedunclePointCloud(selected_peduncle.peduncle_cloud, nearest_fruit.fruit.fruit_pose.position, best_peduncle_.associated_fruit_pose.position);
      selected_peduncle.associated_fruit_pose = nearest_fruit.fruit.fruit_pose;
    }
    if(is_peduncle_available_)
    {
      ROS_INFO_STREAM("Peduncle available");
      addPeduncleToPepper(nearest_fruit, selected_peduncle);
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "HAS PEDUNCLE false");
      nearest_fruit.has_peduncle = false;
    }
    nearest_fruit.fruit.fruit_id = orig_est_interested_fruit_.fruit.fruit_id;
    setCurrentEstimateOfTheInterestedFruit(nearest_fruit);
    publishDebug();
    
  }
  else
  {
    num_pepper_detections_missed_++;
    ROS_ERROR_STREAM("++++++++++++ Cannot find the interested fruit in current detection");
  }
}

bool OnlineFruitEstimator::isDetectedFruitCurrentInterestedOne(const DetectedFruitWithPeduncle& prev_det, DetectedFruitWithPeduncle& potential_candidate, double center_threshold)
{
  return utils::isFruitCentreSimilar(prev_det.fruit.fruit_pose.position, potential_candidate.fruit.fruit_pose.position, center_threshold);
}

int OnlineFruitEstimator::findPedunclebyAssociatedFruitID(const std::vector<pointcloud_roi::SemanticInstanceDetection>& peduncles, uint32_t fruit_id)
{
  for(int i = 0; i < peduncles.size(); ++i)
  {
    if(peduncles[i].associated_instance == fruit_id)
    {
      if(peduncles[i].cloud_segment !=nullptr && peduncles[i].cloud_segment->size() > 0)
        return i;
    }
  }
  return -1;
}

void OnlineFruitEstimator::addPeduncleToPepper(DetectedFruitWithPeduncle& fruit_with_peduncle, const pointcloud_roi::SemanticInstanceDetection& peduncle)
{
  fruit_with_peduncle.has_peduncle = true;
  fruit_with_peduncle.peduncle.peduncle_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  *fruit_with_peduncle.peduncle.peduncle_cloud = *peduncle.cloud_segment;
  fruit_with_peduncle.peduncle.associated_fruit_id = fruit_with_peduncle.fruit.fruit_id;
  fruit_with_peduncle.peduncle.peduncle_id = peduncle.instance_id;
  fruit_with_peduncle.peduncle.is_estimation = false;
  calculatePeduncleEndpoints(fruit_with_peduncle.peduncle);

}

void OnlineFruitEstimator::addPeduncleToPepper(DetectedFruitWithPeduncle& fruit_with_peduncle, const Peduncle& peduncle)
{
  ROS_INFO_STREAM_THROTTLE(0.1, "Attaching peduncle cloud with: "<<peduncle.peduncle_cloud->size());
  fruit_with_peduncle.has_peduncle = true;
  fruit_with_peduncle.peduncle.peduncle_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  *fruit_with_peduncle.peduncle.peduncle_cloud = *peduncle.peduncle_cloud;
  fruit_with_peduncle.peduncle.associated_fruit_id = fruit_with_peduncle.fruit.fruit_id;
  fruit_with_peduncle.peduncle.peduncle_id = peduncle.peduncle_id;
  fruit_with_peduncle.peduncle.is_estimation = false;
  calculatePeduncleEndpoints(fruit_with_peduncle.peduncle);
  ROS_INFO_STREAM_THROTTLE(2.0, "Fruit centre: "<<fruit_with_peduncle.fruit.fruit_pose.position<<"  Peduncle fruitpoint: "<<fruit_with_peduncle.peduncle.fruitpoint.transpose()<<" stempoint: "<<fruit_with_peduncle.peduncle.stempoint.transpose());

}

void OnlineFruitEstimator::calculatePeduncleEndpoints(Peduncle& peduncle)
{
  std::sort(peduncle.peduncle_cloud->begin(), peduncle.peduncle_cloud->end(), [](const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b) {
        return a.z < b.z;
  });

  peduncle.fruitpoint = peduncle.peduncle_cloud->front().getVector3fMap();
  peduncle.stempoint  = peduncle.peduncle_cloud->back().getVector3fMap();

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr OnlineFruitEstimator::smoothPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double search_radius, int polynomial_order) 
{
  // Set up a search method
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(input_cloud);

  // Create a MovingLeastSquares object
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
  mls.setComputeNormals(false); // Disable normal estimation
  mls.setInputCloud(input_cloud);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(search_radius); // Adjust this value as needed
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder(polynomial_order); // Adjust this value as needed

  // Perform surface reconstruction
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  mls.process(*smoothed_cloud);

  return smoothed_cloud;
}

geometry_msgs::Pose OnlineFruitEstimator::filter(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, double alpha) 
{
  geometry_msgs::Pose filtered_pose;
  double alpha_max = std::min(alpha + num_pepper_detections_missed_*0.01, 0.2);
  // Combine the poses using the weighting factor
  filtered_pose.position.x = alpha_max * curr_pose.position.x + (1 - alpha_max) * prev_pose.position.x;
  filtered_pose.position.y = alpha_max * curr_pose.position.y + (1 - alpha_max) * prev_pose.position.y;
  filtered_pose.position.z = alpha_max * curr_pose.position.z + (1 - alpha_max) * prev_pose.position.z;

  // Assuming quaternion representation for orientation
  filtered_pose.orientation.x = alpha_max * curr_pose.orientation.x + (1 - alpha) * prev_pose.orientation.x;
  filtered_pose.orientation.y = alpha_max * curr_pose.orientation.y + (1 - alpha) * prev_pose.orientation.y;
  filtered_pose.orientation.z = alpha_max * curr_pose.orientation.z + (1 - alpha) * prev_pose.orientation.z;
  filtered_pose.orientation.w = alpha_max * curr_pose.orientation.w + (1 - alpha) * prev_pose.orientation.w;

  return filtered_pose;
}

Eigen::Vector3f OnlineFruitEstimator::filter(const Eigen::Vector3f&& prev_point, const Eigen::Vector3f& curr_point, double alpha) 
{
  Eigen::Vector3f filtered_point;
  // Combine the poses using the weighting factor
  filtered_point = alpha * curr_point + (1 - alpha) * prev_point;
  return filtered_point;
}

void OnlineFruitEstimator::publishDebug()
{
  geometry_msgs::Point peduncle_fruitpoint, peduncle_stempoint, fruit_top;

  fruit_top = curr_est_interested_fruit_.fruit.fruit_pose.position;
  fruit_top.z = fruit_top.z + curr_est_interested_fruit_.fruit.fruit_size.z;

  pub_fruit_top_.publish(fruit_top);

  auto eigen_fruitpoint = curr_est_interested_fruit_.peduncle.fruitpoint;
  peduncle_fruitpoint = utils::fromEigen2GeometryMsgPoint(eigen_fruitpoint);
  pub_peduncle_fruitpoint_.publish(peduncle_fruitpoint);

  auto eigen_stempoint = curr_est_interested_fruit_.peduncle.stempoint;
  peduncle_stempoint = utils::fromEigen2GeometryMsgPoint(eigen_fruitpoint);
  pub_peduncle_stempoint_.publish(peduncle_stempoint);

}

bool OnlineFruitEstimator::IsNewPeduncleGoodEnough(const Peduncle& pnew, const Peduncle& pold, double min_points)
{
  if(is_peduncle_available_ == false)
    return true;
  if(pnew.peduncle_cloud->size() > 0.5* pold.peduncle_cloud->size())
    return true;
  else
    return false;
  // else if(pnew.peduncle_cloud->points.size() > min_points)
  // if(pnew.peduncle_cloud->points.size() > min_points && pnew.peduncle_cloud->points.size() > 0.7* pold.peduncle_cloud->points.size())
  // {
  //   return true;
  // }
  // return false;
}

void OnlineFruitEstimator::transformPedunclePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr peduncle_cloud,
                                 const geometry_msgs::Point& p_new,
                                 const geometry_msgs::Point& p_old) 
{
    // Construct Eigen vectors for translation
    Eigen::Vector3f new_pos(p_new.x, p_new.y, p_old.z);
    Eigen::Vector3f old_pos(p_old.x, p_old.y, p_old.z);
    // Calculate translation vector
    Eigen::Vector3f translation_vector = new_pos - old_pos;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = translation_vector;
    // Apply translation to each point in the peduncle point cloud
    pcl::transformPointCloud(*peduncle_cloud, *peduncle_cloud, transform);
}

Peduncle OnlineFruitEstimator::convert(pointcloud_roi::SemanticInstanceDetection& ped_det)
{
  Peduncle peduncle;
  peduncle.peduncle_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(ped_det.cloud_segment);
  sor.setMeanK(30); // Set the number of neighbors to use for mean distance estimation
  sor.setStddevMulThresh(1.0); // Set the standard deviation multiplier threshold
  sor.filter(*peduncle.peduncle_cloud);
  //*peduncle.peduncle_cloud = *ped_det.cloud_segment;
  peduncle.peduncle_id = ped_det.instance_id;
  peduncle.is_estimation = false;
  return peduncle;
}

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr OnlineFruitEstimator::createSurfaceCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
// {
//   auto smoothed_cloud = smoothPointCloud(input_cloud);
  
//   pcl::Poisson<pcl::PointXYZRGB> poisson;
//   poisson.setInputCloud(smoothed_cloud);

//   pcl::PolygonMesh mesh;
//   poisson.reconstruct(mesh);

//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

//   return cloud;
// }
