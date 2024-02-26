#include "active_perception_planner/online_fruit_estimator.h"
#include "active_perception_planner/common.h"
#include "active_perception_planner/utils.h"

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace active_perception_planner;

OnlineFruitEstimator::OnlineFruitEstimator(ros::NodeHandle& nhp, std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector, std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector, const DetectedFruitWithPeduncle& interested_fruit):nhp_(nhp), roi_detector_(roi_detector), se_detector_(se_detector), orig_est_interested_fruit_(interested_fruit), prev_est_interested_fruit_(interested_fruit), curr_est_interested_fruit_(interested_fruit)
{
  roi_detector_->enableContinuousDetections(true);

  pub_peduncle_fruitpoint_  = nhp_.advertise<geometry_msgs::Point>("peduncle_fruitpoint", 1, false);
  pub_peduncle_stempoint_   = nhp_.advertise<geometry_msgs::Point>("peduncle_stempoint", 1, false);
  pub_fruit_top_            = nhp_.advertise<geometry_msgs::Point>("fruit_top_", 1, false);
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
  
  DetectedFruitWithPeduncle nearest_fruit; 
  int nearest_fruit_id = -1;
  double nearest_fruit_distance = 1000;
  for(int i = 0; i < detections->peppers.size(); ++i)
  {
    auto fruit_det = detections->peppers[i];
    DetectedFruitWithPeduncle fruit_with_peduncle;
    auto surface_cloud = smoothPointCloud(fruit_det.cloud_segment);
    if(se_detector_->completeFruitShape(surface_cloud, fruit_with_peduncle.fruit))
    {
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
    pointcloud_roi::SemanticInstanceDetection nearest_pepper_det = detections->peppers[nearest_fruit_id];
    setCurrentEstimateOfPepperDetection(nearest_pepper_det);
    int peduncle_id = findPedunclebyAssociatedFruitID(detections->peduncles, nearest_pepper_det.instance_id);
    if(peduncle_id > -1)
    {
      pointcloud_roi::SemanticInstanceDetection associated_peduncle = detections->peduncles[peduncle_id];
      addPeduncleToPepper(nearest_fruit, associated_peduncle);
      setCurrentEstimateOfPeduncleDetection(associated_peduncle);

      nearest_fruit.fruit.fruit_pose = filter(prev_est_interested_fruit_.fruit.fruit_pose, nearest_fruit.fruit.fruit_pose, 0.1);
      nearest_fruit.fruit.fruit_id = orig_est_interested_fruit_.fruit.fruit_id;
      setCurrentEstimateOfTheInterestedFruit(nearest_fruit);
      publishDebug();
    }
  }
  else
  {
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
      return i;
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

void OnlineFruitEstimator::calculatePeduncleEndpoints(Peduncle& peduncle)
{
  std::sort(peduncle.peduncle_cloud->begin(), peduncle.peduncle_cloud->end(), [](const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b) {
        return a.z < b.z;
  });

  peduncle.fruitpoint = peduncle.peduncle_cloud->front().getVector3fMap();
  peduncle.stempoint  = peduncle.peduncle_cloud->back().getVector3fMap();

  ROS_INFO_STREAM("Peduncle fruitpoint: "<<peduncle.fruitpoint.transpose()<<" stempoint: "<<peduncle.stempoint.transpose());
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

  // Combine the poses using the weighting factor
  filtered_pose.position.x = alpha * curr_pose.position.x + (1 - alpha) * prev_pose.position.x;
  filtered_pose.position.y = alpha * curr_pose.position.y + (1 - alpha) * prev_pose.position.y;
  filtered_pose.position.z = alpha * curr_pose.position.z + (1 - alpha) * prev_pose.position.z;

  // Assuming quaternion representation for orientation
  filtered_pose.orientation.x = alpha * curr_pose.orientation.x + (1 - alpha) * prev_pose.orientation.x;
  filtered_pose.orientation.y = alpha * curr_pose.orientation.y + (1 - alpha) * prev_pose.orientation.y;
  filtered_pose.orientation.z = alpha * curr_pose.orientation.z + (1 - alpha) * prev_pose.orientation.z;
  filtered_pose.orientation.w = alpha * curr_pose.orientation.w + (1 - alpha) * prev_pose.orientation.w;

  return filtered_pose;
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
