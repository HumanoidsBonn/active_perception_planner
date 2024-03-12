#include "active_perception_planner/online_fruit_estimator.h"
#include "active_perception_planner/common.h"
#include "active_perception_planner/utils_pointcloud_distances.h"



using namespace active_perception_planner;

OnlineFruitEstimator::OnlineFruitEstimator(ros::NodeHandle& nhp, std::shared_ptr<pointcloud_roi::DetectedRoiFilter> roi_detector, std::shared_ptr<superellipsoid::SuperellipsoidDetector> se_detector, const DetectedFruitWithPeduncle& interested_fruit):nhp_(nhp), roi_detector_(roi_detector), se_detector_(se_detector), orig_est_interested_fruit_(interested_fruit), prev_est_interested_fruit_(interested_fruit), curr_est_interested_fruit_(interested_fruit), 
tf_buffer_(ros::Duration(30)),
tf_listener_(tf_buffer_)
{
  is_peduncle_available_ = false;
  if(orig_est_interested_fruit_.has_peduncle)
  {
    if(orig_est_interested_fruit_.peduncle.peduncle_cloud != nullptr && orig_est_interested_fruit_.peduncle.peduncle_cloud->empty() == false)
    {
      best_peduncle_ = orig_est_interested_fruit_.peduncle;
      best_peduncle_.associated_fruit_pose = orig_est_interested_fruit_.fruit.fruit_pose;
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(best_peduncle_.peduncle_cloud);
      sor.setMeanK(30); // Set the number of neighbors to use for mean distance estimation
      sor.setStddevMulThresh(1.0); // Set the standard deviation multiplier threshold
      sor.filter(*best_peduncle_.peduncle_cloud);
      if(best_peduncle_.peduncle_cloud->empty() == false)
      {
        is_peduncle_available_ = true;
        is_old_peduncle_available_ = true;
      }
    }
  }
  if(is_peduncle_available_ == false)
  {
    orig_est_interested_fruit_.peduncle.peduncle_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    orig_est_interested_fruit_.has_peduncle = false;
  }

  roi_detector_->enableContinuousDetections(false);

  timer_tf_   = nhp_.createTimer(ros::Duration(0.4), &OnlineFruitEstimator::processTFTimerCallback, this);
  timer_tf_.start();
  int buffer_size = 5;
  nhp_.param<int>("ofe/peduncle/buffer_size", buffer_size, 3);

  peduncle_buffer_ = std::make_unique<PointCloudBuffer<pcl::PointXYZRGB>>(buffer_size, 0.5);

  pub_peduncle_fruitpoint_  = nhp_.advertise<geometry_msgs::Point>("peduncle_fruitpoint", 1, false);
  pub_peduncle_stempoint_   = nhp_.advertise<geometry_msgs::Point>("peduncle_stempoint", 1, false);
  pub_fruit_top_            = nhp_.advertise<geometry_msgs::Point>("fruit_top_", 1, false);
  pub_peduncle_cloud_       = nhp_.advertise<sensor_msgs::PointCloud2>("raw_peduncle", 1, false);
}

std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> OnlineFruitEstimator::readDetections(double sleep_time, double total_time, bool enable_continuous_detections)
{
  std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> detections = nullptr;
  roi_detector_->enableTrackedDetectionCallback(true);
  if(enable_continuous_detections)
    roi_detector_->enableContinuousDetections(true);
  else
    roi_detector_->enableTrackedDetectionCallback(true);

  auto t_start = ros::Time::now();
  sleep(sleep_time);
  do
  { //tracked_dets_available = roi_detector_->isTrackedDetectionAvailable();
    detections = roi_detector_->readSemanticInstanceDetections();
    sleep(sleep_time);
  }
  while(detections == nullptr && (ros::Time::now()-t_start).toSec() < total_time);
  return detections;
}

bool OnlineFruitEstimator::computeFruitEstimate()
{
  is_peduncle_available_ = false;
  bool success = false;
  
  std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> detections = readDetections(0.02, 0.1, false);
  if(detections == nullptr)
  {
    is_new_estimate_available_ = false;
    return false;
  }
  
  DetectedFruitWithPeduncle nearest_fruit; 
  int nearest_fruit_id = -1;
  double nearest_fruit_distance = 1000;
  double nearest_fruit_z_dist = 1000;
  double nearest_fruit_xy_dist = 1000;
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = filterCloud<pcl::PointXYZRGB>(fruit_det.cloud_segment, orig_est_interested_fruit_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cloud = smoothPointCloud(filtered_cloud);
    if(surface_cloud->empty())
    {
      ROS_WARN_STREAM_THROTTLE(3.0, "PointCloud empty for ID: "<<fruit_det.instance_id);
      continue;
    }
    fruit_with_peduncle.fruit.fruit_pose = orig_est_interested_fruit_.fruit.fruit_pose;
    fruit_with_peduncle.fruit.fruit_size = orig_est_interested_fruit_.fruit.fruit_size;

    if(se_detector_->completeFruitShape(surface_cloud, fruit_with_peduncle.fruit, true)== false)
    {
      ROS_WARN_STREAM_THROTTLE(3.0, "SE fitting failure: "<<fruit_det.instance_id);
      continue;
    }
    if(fruit_with_peduncle.fruit.observation_score < 0.3)
    {
      ROS_WARN_STREAM_THROTTLE(3.0, "SE obs score too low: "<<fruit_with_peduncle.fruit.observation_score);
      continue;
    }

    ROS_INFO_STREAM_THROTTLE(1.0, "Variance New: "<<fruit_with_peduncle.fruit.variance.transpose()<<"\t Variance Orig: "<<orig_est_interested_fruit_.fruit.variance.transpose());
    Eigen::Vector3d c1, c2, c3;
    tf::pointMsgToEigen(curr_est_interested_fruit_.fruit.fruit_pose.position, c1);
    tf::pointMsgToEigen(fruit_with_peduncle.fruit.fruit_pose.position, c2);
    tf::pointMsgToEigen(orig_est_interested_fruit_.fruit.fruit_pose.position, c3);
    ROS_INFO_STREAM_THROTTLE(1.0, "New INST Fruit Centre: "<<c2.transpose()<<"\tCurr Filt Fruit Centre: "<<c1.transpose()<< "\t Orig Fruit Centre: "<<c3.transpose());
    Eigen::Vector3d error_in_centres = utils::computeErrorInCentres(curr_est_interested_fruit_.fruit.fruit_pose.position, fruit_with_peduncle.fruit.fruit_pose.position);
    double center_dist_wrt_orig = (c1-c3).norm();
    ROS_INFO_STREAM_THROTTLE(1.0, "Error in centres: "<<error_in_centres.transpose());
    ROS_INFO_STREAM_THROTTLE(1.0, "Fruit ID: "<<fruit_det.instance_id<< "Confident ID: "<<fruit_id_of_confident_detection_);
    double centre_dist = error_in_centres.norm();
    double x_y_dist = error_in_centres.head(2).norm();
    double z_dist = fabs(error_in_centres.z());
    if(center_dist_wrt_orig > 0.05)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "large error wrt orig estimate: "<<error_in_centres.transpose());
      continue;
    }
    if(centre_dist < 0.01)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Found fruit. Very close to prev centres");
      confident_id_available_ = true;
      nearest_fruit_id = i;
      nearest_fruit_distance = centre_dist;
      nearest_fruit = fruit_with_peduncle;
      fruit_id_of_confident_detection_ = fruit_det.instance_id;
      nearest_fruit_z_dist = z_dist;
      nearest_fruit_xy_dist = x_y_dist;
      break;
    }
    else if(confident_id_available_ && fruit_det.score > 0.75 && fruit_det.instance_id == fruit_id_of_confident_detection_ && centre_dist < 0.02)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Found fruit. ID same");
      nearest_fruit_id = i;
      nearest_fruit_distance = centre_dist;
      nearest_fruit_z_dist = z_dist;
      nearest_fruit = fruit_with_peduncle;
      break;
    }
  }
  if(nearest_fruit_id < 0)
  {
    num_pepper_detections_missed_++;
    ROS_ERROR_STREAM_THROTTLE(2.0, "++++++++++++ Cannot find the interested fruit in current detection");
    return false;
  }
  num_pepper_detections_missed_ = 0;
  pointcloud_roi::SemanticInstanceDetection nearest_pepper_det = detections->peppers[nearest_fruit_id];
  setCurrentEstimateOfPepperDetection(nearest_pepper_det);
  fruit_id_of_confident_detection_ = nearest_pepper_det.instance_id;
  if(confident_id_available_)
  {
    fruit_id_of_confident_detection_ = nearest_pepper_det.instance_id;
  }
  int peduncle_id = findPedunclebyAssociatedFruitID(detections->peduncles, nearest_pepper_det.instance_id);
  auto inst_fruit_pose = nearest_fruit.fruit.fruit_pose;
  nearest_fruit.fruit.fruit_pose = filter(prev_est_interested_fruit_.fruit.fruit_pose, nearest_fruit.fruit.fruit_pose, 0.1);
  Eigen::Vector3d c4;
  tf::pointMsgToEigen(nearest_fruit.fruit.fruit_pose.position, c4);
  ROS_INFO_STREAM("Filtered Fruit Centre: "<<c4.transpose());

  nearest_fruit.fruit.fruit_size = orig_est_interested_fruit_.fruit.fruit_size;
  Peduncle selected_peduncle;
  bool use_old_peduncle = false;
  if(peduncle_id > -1 && detections->peduncles[peduncle_id].cloud_segment != nullptr)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "Peduncle associated with fruit is present");
    num_peduncle_detections_missed_ = 0;
    pointcloud_roi::SemanticInstanceDetection associated_peduncle_det = detections->peduncles[peduncle_id];
    Peduncle associated_peduncle;
    associated_peduncle = convert(associated_peduncle_det);
    setCurrentEstimateOfPeduncleDetection(associated_peduncle_det);
    if(IsNewPeduncleGoodEnough(associated_peduncle, best_peduncle_))
    {
      ROS_INFO_STREAM("HURRAH New Peduncle good enough: "<<associated_peduncle.peduncle_cloud->size());
      selected_peduncle = associated_peduncle;
      //transformPedunclePointCloud(selected_peduncle.peduncle_cloud, nearest_fruit.fruit.fruit_pose.position, inst_fruit_pose.position);
      setBestPeduncle(selected_peduncle);
      selected_peduncle.associated_fruit_pose = nearest_fruit.fruit.fruit_pose;
      use_old_peduncle = false;
      is_peduncle_available_ = true;
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
    //transformPedunclePointCloud(selected_peduncle.peduncle_cloud, nearest_fruit.fruit.fruit_pose.position, selected_peduncle.associated_fruit_pose.position);
    selected_peduncle.associated_fruit_pose = nearest_fruit.fruit.fruit_pose;
    is_peduncle_available_ = true;
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
  return is_new_estimate_available_;    
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
      if(peduncles[i].cloud_segment !=nullptr && peduncles[i].cloud_segment->empty() == false)
        return i;
    }
  }
  return -1;
}

void OnlineFruitEstimator::addPeduncleToPepper(DetectedFruitWithPeduncle& fruit_with_peduncle, const pointcloud_roi::SemanticInstanceDetection& peduncle)
{
  fruit_with_peduncle.peduncle.peduncle_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  if(peduncle.cloud_segment == nullptr)
    return;

  fruit_with_peduncle.has_peduncle = true;
  *fruit_with_peduncle.peduncle.peduncle_cloud = *peduncle.cloud_segment;
  fruit_with_peduncle.peduncle.associated_fruit_id = fruit_with_peduncle.fruit.fruit_id;
  fruit_with_peduncle.peduncle.peduncle_id = peduncle.instance_id;
  fruit_with_peduncle.peduncle.is_estimation = false;
  calculatePeduncleEndpoints(fruit_with_peduncle.peduncle);

}

void OnlineFruitEstimator::addPeduncleToPepper(DetectedFruitWithPeduncle& fruit_with_peduncle, const Peduncle& peduncle)
{
  ROS_INFO_STREAM_THROTTLE(0.1, "Attaching peduncle cloud with: "<<peduncle.peduncle_cloud->size());
  fruit_with_peduncle.peduncle.peduncle_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  if(peduncle.peduncle_cloud == nullptr)
    return;

  fruit_with_peduncle.has_peduncle = true;
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
  if(input_cloud->empty())
  {
    ROS_WARN_THROTTLE(2.0, "Pepper Online Cloud empty. Hence not smoothing");
    return input_cloud;
  }
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

Vector6d OnlineFruitEstimator::filterUsingVariances(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, const Eigen::Vector3d& prev_var, const Eigen::Vector3d& curr_var, double alpha) 
{
  Eigen::Vector3d prev_mean, curr_mean;
  tf2::fromMsg(prev_pose.position, prev_mean);
  tf2::fromMsg(curr_pose.position, curr_mean);

  return utils::computeWeightedAverageUsingVariance(prev_mean, curr_mean, prev_var, curr_var, alpha);
}

bool OnlineFruitEstimator::getPose(geometry_msgs::TransformStamped& transform, const std::string& target_frame, const std::string& original_frame)
{
  try
  {
    transform = tf_buffer_.lookupTransform(target_frame, original_frame, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform from "<<original_frame<<" to "<< target_frame<<" : " << e.what());
    return false;
  }
  //ROS_INFO_STREAM_THROTTLE(1.0, "TF:"<<transform);
  return true;
}
geometry_msgs::Pose OnlineFruitEstimator::updateAccordingToGripperPosition(const geometry_msgs::Pose& prev_pose, const geometry_msgs::Pose& curr_pose, double alpha) 
{
  geometry_msgs::Pose new_pose;
  geometry_msgs::TransformStamped ee_tf;
  Eigen::Vector3d curr_ee_pos;
  std::string ref_frame_id = "trolley_local_link";
  if(getPose(ee_tf, "arm_right_eef_link", ref_frame_id))
  {
    curr_ee_pos.x() = ee_tf.transform.translation.x;
    curr_ee_pos.y() = ee_tf.transform.translation.y;
    curr_ee_pos.z() = ee_tf.transform.translation.z;

    Eigen::Vector3d diff_in_pos = curr_ee_pos- fruit_grasping_position_;
    Eigen::Vector3d instdiff_in_pos = curr_ee_pos - prev_ee_pos_;
    if(instdiff_in_pos.norm() > 0.03)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Large change in ee pos");
      ee_pos_update_available_ = false;
      new_pose = curr_pose;
    }
    else
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Change in EE Position: "<<diff_in_pos.transpose());
      new_pose.position.x = prev_pose.position.x + diff_in_pos.x();
      new_pose.position.y = prev_pose.position.y + diff_in_pos.y();
      new_pose.position.z = prev_pose.position.z; // + diff_in_pos.z();
      new_pose = filter(prev_pose, new_pose, 0.5);
      new_pose.position.z = prev_pose.position.z;
      ee_pos_update_available_ = true;
    }
  }
  else
  {
    ee_pos_update_available_ = false;
    new_pose = curr_pose;
  }
  return new_pose;
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
  if(pnew.peduncle_cloud == nullptr || pnew.peduncle_cloud->empty())
    return false;
  if(pold.peduncle_cloud == nullptr || pold.peduncle_cloud->empty())
    return true;

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
  if(ped_det.cloud_segment == nullptr or ped_det.cloud_segment->empty())
  {
    peduncle.peduncle_cloud.reset();
    return peduncle;
  }
  sor.setInputCloud(ped_det.cloud_segment);
  sor.setMeanK(30); // Set the number of neighbors to use for mean distance estimation
  sor.setStddevMulThresh(1.0); // Set the standard deviation multiplier threshold
  sor.filter(*peduncle.peduncle_cloud);
  //*peduncle.peduncle_cloud = *ped_det.cloud_segment;
  peduncle.peduncle_id = ped_det.instance_id;
  peduncle.is_estimation = false;
  return peduncle;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr OnlineFruitEstimator::createSurfaceCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
  auto smoothed_cloud = smoothPointCloud(input_cloud);
  auto normals = estimateNormals<pcl::PointXYZRGB>(smoothed_cloud, 0.015f);
  
  if(normals == nullptr)
    return smoothed_cloud;
  

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  for(size_t i = 0; i < smoothed_cloud->size(); ++i)
  {
    pcl::PointXYZRGBNormal point_normal;
    point_normal.x = smoothed_cloud->points[i].x;
    point_normal.y = smoothed_cloud->points[i].y;
    point_normal.z = smoothed_cloud->points[i].z;    
    
    point_normal.normal_x = normals->points[i].normal_x;
    point_normal.normal_y = normals->points[i].normal_y;
    point_normal.normal_z = normals->points[i].normal_z;
    cloud_with_normals->push_back(point_normal);
  }
  
  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
  poisson.setInputCloud(cloud_with_normals);

  pcl::PolygonMesh mesh;
  poisson.reconstruct(mesh);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  return cloud;
}

template <typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr OnlineFruitEstimator::estimateNormals(const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud, float search_radius)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals_in;
  normals_in.reset(new pcl::PointCloud<pcl::Normal>());
  if(pcl_cloud->empty())
  {
    normals_in.reset();
    return normals_in;
  }
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod(tree);
  ne.setInputCloud(pcl_cloud);
  ne.setRadiusSearch (search_radius); // Use all neighbors in a sphere of radius 3cm (0.03f) might be a good value
  normals_in->points.reserve(pcl_cloud->points.size());
  ne.compute (*normals_in);
  return normals_in;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr OnlineFruitEstimator::filterCloud(const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud, DetectedFruitWithPeduncle& fruit_with_peduncle)
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  if(pcl_cloud->empty())
    return pcl_cloud;
  sor.setInputCloud(pcl_cloud);
  sor.setMeanK(50);  // Set the number of neighbors to analyze for each point
  sor.setStddevMulThresh(1.0);  // Set the standard deviation multiplier threshold
  // You can adjust these parameters according to your needs

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  sor.filter(*cloud_filtered);

  if(cloud_filtered->empty())
    return cloud_filtered;

  typename pcl::PointCloud<PointT>::Ptr cloud_se_filtered(new pcl::PointCloud<PointT>);
  for (const auto& point : cloud_filtered->points) {
      if (se_detector_->isPointInsideSuperellipsoid<PointT>(point, fruit_with_peduncle.fruit, 1.2)) {
          cloud_se_filtered->points.push_back(point);
      }
  }
  cloud_se_filtered->width = cloud_se_filtered->points.size();
  cloud_se_filtered->height = 1;
  cloud_se_filtered->is_dense = true;
  return cloud_se_filtered;
}

bool OnlineFruitEstimator::computePeduncleEstimate()
{
  num_peduncle_computations_++;
  ROS_INFO_STREAM_THROTTLE(0.1, "computePeduncleEstimate");
  Eigen::Vector3d cutter_ee_pos;
  if(is_peduncle_computed_)
  {
    ROS_INFO_STREAM_THROTTLE(0.1, "Before Cutter");
    cutter_ee_pos = getCutterEEPosition();
    if(isCutterOccludingPeduncle(cutter_ee_pos, computed_peduncle_, 0.2))
    {
      is_new_estimate_available_ = false;
      return false;
    }
  }
  ROS_INFO_STREAM_THROTTLE(0.1, "After Cutter");
  std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> detections = readDetections(0.02, 0.06, true);

  ROS_INFO_STREAM_THROTTLE(0.1, "AFTER READ");
  bool peduncle_found = false;
  int peduncle_det_idx = -1;
  if(detections == nullptr)
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Detections nullptr");
    return false;
  }
  if(detections->peduncles.empty())
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Peduncle detections empty");
    return false;
  }
  ROS_INFO_STREAM_THROTTLE(0.1, "Peduncle detections size: "<<detections->peduncles.size());
  for(int i = 0; i < detections->peduncles.size(); ++i)
  {
    if(confident_id_available_ && detections->peduncles[i].instance_id == fruit_id_of_confident_detection_)
    {
      if(isPeduncleCloudValid(detections->peduncles[i]))
      {
        peduncle_found = true;
        peduncle_det_idx = i;
        break;
      }
    }
    else
    {
      if(isPeduncleCloudValid(detections->peduncles[i]))
      {
        peduncle_found = true;
        peduncle_det_idx = i;
        break;
      }
    }
  }
  if(!peduncle_found)
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "No peduncle found in current detections");
    return false;
  }
  ROS_INFO_STREAM_THROTTLE(0.1, "PEDUNCLE FOUND++++++++++++++++++++++++++++++++");
  Peduncle peduncle = convert(detections->peduncles[peduncle_det_idx]);

  peduncle_buffer_->addFrame(peduncle.peduncle_cloud, ros::Time::now());

  peduncle.peduncle_cloud = peduncle_buffer_->getMergedSmoothedFrame();

  if(peduncle.peduncle_cloud == nullptr || peduncle.peduncle_cloud->empty())
  {
    ROS_INFO_STREAM_THROTTLE(0.1, "FOUND PEDUNCLE EMPTY++++++++++++++++++++++++++++++++");
    return false;
  }
  calculatePeduncleEndpoints(peduncle);
  DetectedFruitWithPeduncle fruit_with_peduncle(curr_est_interested_fruit_.fruit, peduncle);
  fruit_with_peduncle.has_peduncle = true;
  computed_peduncle_ = peduncle;
  is_peduncle_computed_ = true;
  setCurrentEstimateOfTheInterestedFruit(fruit_with_peduncle);
  ROS_INFO_STREAM_THROTTLE(0.1, "UPDATED FRUIT WITH PEDUNCLE++++++++++++++++++++++++++++++++");
  return true;
}

// bool isOnlinePeduncleGoodEnough(const Peduncle& peduncle, bool peduncle_found)
// {
//   if(num_peduncle_computations_ < 3)
//     return true;

//   if()

//   if((peduncle.stempoint- peduncle.fruitpoint).z() < 0.02)
//     return false;
// }

bool OnlineFruitEstimator::isPeduncleCloudValid(const pointcloud_roi::SemanticInstanceDetection& peduncle)
{
  if(peduncle.cloud_segment == nullptr || peduncle.cloud_segment->empty())
  {
    ROS_WARN_THROTTLE(0.1, "isPeduncleCloudValid: Peduncle cloud empty");
    return false;
  }
  sensor_msgs::PointCloud2 ped_msg;
  pcl::toROSMsg(*(peduncle.cloud_segment), ped_msg);  
  ped_msg.header.frame_id = "trolley_local_link";
  ped_msg.header.stamp = ros::Time::now();

  pub_peduncle_cloud_.publish(ped_msg);

  auto centre = utils::fromGeometryMsg2EigenVector3d(orig_est_interested_fruit_.fruit.fruit_pose.position);
  auto dim    = utils::fromGeometryMsg2EigenVector3d(orig_est_interested_fruit_.fruit.fruit_size);

  Eigen::Vector3d bbox_min, bbox_max;

  bbox_min = centre - dim ;
  bbox_min.x() = bbox_min.x() - 0.02;
  bbox_min.y() = bbox_min.y() - 0.02;

  bbox_min.z() = bbox_min.z() + 1.0*dim.z()-0.01;

  bbox_max = centre + dim;
  bbox_max.x() = bbox_max.x() + 0.02;
  bbox_max.y() = bbox_max.y() + 0.03;

  bbox_max.z() = bbox_max.z() + 1.0*dim.z() + 0.02;

  peduncle.cloud_segment = utils::filterPointCloudByBoundingBox(peduncle.cloud_segment, bbox_min, bbox_max);
  if(peduncle.cloud_segment->empty())
  {
    ROS_WARN_THROTTLE(0.00001, "isPeduncleCloudValid: After BBOX filtering Peduncle cloud empty");
    return false;
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*(peduncle.cloud_segment), centroid);

  Eigen::Vector3d ped_centroid = centroid.head(3).cast<double>();
  
  if(ped_centroid.x() < bbox_min.x() || ped_centroid.x() > bbox_max.x() || ped_centroid.y() < bbox_min.y() || ped_centroid.y() > bbox_max.y() || ped_centroid.z() < bbox_min.z() || ped_centroid.z() > bbox_max.z())
  {
    ROS_WARN_STREAM_THROTTLE(0.000001, "Peduncle outside top bbox: centroid: "<<ped_centroid.transpose()<<"\t bbox_min: "<<bbox_min.transpose()<<"\t bbox_max: "<<bbox_max.transpose());
    return false;
  }
  return true;
}

void OnlineFruitEstimator::processTFTimerCallback(const ros::TimerEvent& e)
{
  geometry_msgs::TransformStamped ee_tf;
  std::string ref_frame_id = "trolley_local_link";
  {
    std::scoped_lock lk(mutex_tf_);
    if(getPose(ee_tf, ref_frame_id, "arm_left_eef_link"))
    {
      cutter_ee_pos_.x() = ee_tf.transform.translation.x;
      cutter_ee_pos_.y() = ee_tf.transform.translation.y;
      cutter_ee_pos_.z() = ee_tf.transform.translation.z;
    }
    else
    {
      ROS_WARN_THROTTLE(2.0, "Could not get arm_left_ee_link transform");
    }
  }
  return;
}

Eigen::Vector3d OnlineFruitEstimator::getCutterEEPosition()
{
  Eigen::Vector3d res;
  {
    std::scoped_lock lk(mutex_tf_);
    res = cutter_ee_pos_;
  }
  return res;
}

bool OnlineFruitEstimator::isCutterOccludingPeduncle(const Eigen::Vector3d& cutter_ee_pos, const Peduncle& peduncle, double dist_threshold)
{
  pcl::PointXYZRGB min_point, max_point;
  pcl::getMinMax3D(*peduncle.peduncle_cloud, min_point, max_point);

    // Define the bounding box corners
  Eigen::Vector3f min_coords(min_point.x, min_point.y, min_point.z);
  Eigen::Vector3f max_coords(max_point.x, max_point.y, max_point.z);

  Eigen::Vector3f offset;
  offset<<0.02, 0.02, 0.02;
  min_coords = min_coords - offset;
  max_coords = max_coords + offset; 

  if(cutter_ee_pos.x() < min_coords.x() || cutter_ee_pos.x() > max_coords.x() || cutter_ee_pos.y() < min_coords.y() || cutter_ee_pos.y() > max_coords.y() || cutter_ee_pos.z() < min_coords.z() || cutter_ee_pos.z() > max_coords.z())
  {
    ROS_INFO_STREAM_THROTTLE(0.1, "Cutter pos: "<<cutter_ee_pos.transpose()<<"\t bbox_min: "<<min_coords.transpose()<<"\t bbox_max: "<<max_coords.transpose());
    return false;
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Cutter occluding peduncle: "<<cutter_ee_pos.transpose()<<"\t bbox_min: "<<min_coords.transpose()<<"\t bbox_max: "<<max_coords.transpose());
    return true;
  }
}
