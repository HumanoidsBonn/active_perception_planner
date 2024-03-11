#include "active_perception_planner/stem_estimator.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace active_perception_planner;

StemEstimator::StemEstimator()
{
  is_running_ = true;
  processing_thread_ = std::thread(&StemEstimator::processingThread, this);
}

StemEstimator::~StemEstimator()
{
  is_running_ = false;
  cv_.notify_one();
  try
  {
    processing_thread_.join();
  }
  catch (std::system_error&) {}
}

std::vector<Stem> StemEstimator::getStems()
{
  std::vector<Stem> stems;
  {
    std::scoped_lock(mutex_stems_);
    stems.insert(stems.begin(), stems_.begin(), stems_.end());
  }
  ROS_WARN_STREAM("Stem Estimator: getStems No of stems: "<<stems.size());
  return stems;
}

void StemEstimator::setStems(const std::vector<Stem>& stems)
{
  {
    std::scoped_lock(mutex_stems_);
    stems_.clear();
    stems_.insert(stems_.begin(), stems.begin(), stems.end());
    ROS_WARN_STREAM("END Size of stems: "<<stems.size()<<"\t persistent: "<<stems_.size());
  }
  return;
}

void StemEstimator::semanticInstanceDetectionsCallback(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets)
{
  ROS_INFO_STREAM_THROTTLE(3.0, "StemEstimator: detectionCallback");
  {
    std::lock_guard<std::mutex> lk(mutex_dets_);
    dets_ = dets;
  }
  cv_.notify_one();
}

void StemEstimator::processingThread()
{
  while (is_running_ && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE(3.0, "StemEstimator: Is running");
    std::unique_lock<std::mutex> lk(mutex_dets_);
    cv_.wait(lk, [this]{return this->dets_ != nullptr || !this->is_running_;});
    if (!this->is_running_)
      return;
    
    std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> current_dets = dets_;
    dets_ = nullptr;

    setCurrentStemDetections(current_dets);
    processStemDetections();

    lk.unlock();
  }
}

void StemEstimator::processStemDetections()
{
  auto t_start = ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(3.0, "StemEstimator: processStemDetections 1");
  createStemsFromCurrentDetections(stem_detections_);
  // std::vector<active_perception_planner::Line2D>  lines = computeLinesUsing2DContours(stem_detections_);
  // std::vector<std::vector<active_perception_planner::Line2D>> grouped_segments = groupApproxCollinearSegments(lines);
  // auto current_stems = combine3DSegmentsFromGrouped2DSegments(grouped_segments);
  // ROS_INFO_STREAM_THROTTLE(3.0, "StemEstimator: processStemDetections 2");
  // if(stems_.empty())
  // {
  //   ROS_INFO_STREAM_THROTTLE(3.0, "StemEstimator: processStemDetections 3");
  //   stems_.insert(stems_.begin(), current_stems.begin(), current_stems.end());
  // }
  // else
  // {
  //   ROS_INFO_STREAM_THROTTLE(3.0, "StemEstimator: processStemDetections 4");
  //   mergeNewWithExistingStems(current_stems);
  // }
  ROS_INFO_STREAM_THROTTLE(3.0, "StemEstimator: processStemDetections time taken: "<<(ros::Time::now()-t_start).toSec());
}

void StemEstimator::setCurrentStemDetections(std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets)
{
  rgb_image_ = dets->rgb_image.clone();
  depth_image_ = dets->depth_image.clone();
  stem_detections_.clear();
  for(int i = 0; i < dets->stems.size(); ++i)
  {
    auto stem_det = dets->stems[i];
    if(stem_det.cloud_segment == nullptr || stem_det.cloud_segment->size() < single_stem_min_points_)
      continue;
    
    //ROS_INFO_STREAM("SetCurrent Det: det cloud size: "<<stem_det.cloud_segment->size());
    stem_detections_.push_back(stem_det);
  }
  //stem_detections_.insert(stem_detections_.begin(), dets->stems.begin(), dets->stems.end());
  lines_not_init_ = false;
}

std::vector<Stem> StemEstimator::createStemsFromCurrentDetections(const std::vector<pointcloud_roi::SemanticInstanceDetection>& stem_detections)
{
  std::vector<Stem> stems = getStems();
  //ROS_WARN_STREAM("Size of stems: "<<stems.size()<<"\t persistent: "<<stems_.size());
  for(int i = 0; i < stem_detections.size(); ++i)
  {
    auto stem_det = stem_detections[i];
    if(stem_det.cloud_segment == nullptr or stem_det.cloud_segment->size() < single_stem_min_points_)
    {
      ROS_INFO_STREAM("Stem: det cloud seg either nullptr or too small");
      continue;
    }
    active_perception_planner::Stem stem;
    stem.line_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    *stem.line_cloud +=  *stem_det.cloud_segment;
    stem.stem_id = stem_det.instance_id;
    calculateStemParameters(stem);
    
    ROS_INFO_STREAM_THROTTLE(1.0, "Before Check Stem id: "<<stem.stem_id<<"\t bottom: "<<stem.bottom_point.transpose()<<"\t top: "<<stem.top_point.transpose()<<"\t dir: "<<stem.line_direction.transpose());
    
    if(isStemTooShort(stem.top_point, stem.bottom_point, 0.005f))
      continue;
    if(isStemApproximatelyParallelToZAxis(stem.line_direction, 90.0f) == false)
      continue;
    
    ROS_INFO_STREAM_THROTTLE(1.0, "Stem id: "<<stem.stem_id<<"\t bottom: "<<stem.bottom_point.transpose()<<"\t top: "<<stem.top_point.transpose()<<"\t dir: "<<stem.line_direction.transpose());
    size_t j = 0;
    // Flag to indicate if a match is found
    bool match_found = false;

    while(j < stems.size() && !match_found)
    {
      ROS_INFO_STREAM("Checking for match with stem "<<j<<" out of "<<stems.size());
      bool is_same_stem = checkIfStemsAreSame(stems[j], stem, 0.3, 0.05);
      if(is_same_stem)
      {
        ROS_INFO_STREAM("match found");
        auto existing_stem = stems[j];
        if(existing_stem.line_cloud !=nullptr && stem.line_cloud !=nullptr)
          *(existing_stem.line_cloud) += *(stem.line_cloud);
        stems[j] = existing_stem;
        match_found = true;
      }
      j++;
    }
    if(!match_found)
    {
      ROS_INFO_STREAM("No match found");
      stem.stem_id = stems.size()+1;
      stems.push_back(stem);
    }
  }
  setStems(stems);
  ROS_WARN_STREAM("END Size of stems: "<<stems.size()<<"\t persistent: "<<stems_.size());
  return stems_;
}

void StemEstimator::calculateStemParameters(Stem& stem)
{
  regress3DLine(stem.line_cloud, stem.line_origin, stem.line_direction);
  stem.diameter = calculateApproximateDiameterAverageMetric(stem.line_cloud, stem.line_origin, stem.line_direction);
  findTopAndBottomPoints(stem.line_cloud, stem.top_point, stem.bottom_point);
    
}

void StemEstimator::mergeSegmentsWithinSameDetection(std::vector<Stem>& stems)
{
  for (size_t i = 0; i < stems.size(); ++i) 
  {
    for (size_t j = i + 1; j < stems.size(); ++j)
    {
      bool is_same_stem = checkIfStemsAreSame(stems[i], stems[j], 0.3, 0.05);
      if(is_same_stem == false)
        continue;
      
      if(stems[i].line_cloud !=nullptr && stems[j].line_cloud !=nullptr)
          *(stems[i].line_cloud) += *(stems[i].line_cloud);

    }
  }
}

bool StemEstimator::checkLineParallelToYAxis(double theta)
{
  return (std::abs(theta) > (CV_PI / 2 - CV_PI * vertical_angle_threshold_ / 180.0) &&
            std::abs(theta) < (CV_PI / 2 + CV_PI * vertical_angle_threshold_ / 180.0));
}

bool StemEstimator::areSegmentsApproximatelyCollinear(cv::Point2f segment1[2], cv::Point2f segment2[2]) 
{
  // Calculate slopes and intercepts for the line segments
  double slope1 = (segment1[1].y - segment1[0].y) / (segment1[1].x - segment1[0].x + 1e-6);  // Avoid division by zero
  double theta1 = std::atan2(segment1[1].y - segment1[0].y, segment1[1].x - segment1[0].x + 1e-6);

  double slope2 = (segment2[1].y - segment2[0].y) / (segment2[1].x - segment2[0].x + 1e-6);  // Avoid division by zero
  double theta2 = std::atan2(segment2[1].y - segment2[0].y, segment2[1].x - segment2[0].x + 1e-6);
  double slope21 = (segment2[0].y - segment2[1].y) / (segment2[0].x - segment2[1].x + 1e-6);  // Avoid division by zero

  // Check if slopes and intercepts are approximately equal within the tolerance
  if (checkLineParallelToYAxis(theta1) && checkLineParallelToYAxis(theta2)) {
      double intercept1 = (segment1[1].x + segment1[0].x) / 2;
      double intercept2 = (segment2[1].x + segment2[0].x) / 2;

      if (std::abs(intercept1 - intercept2) <= intercept_tolerance_) {
          return true;
      } else {
          return false;
      }
  } else {
      double intercept1 = segment1[0].y - slope1 * segment1[0].x;
      double intercept2 = segment2[0].y - slope2 * segment2[0].x;

      if (std::abs(slope1 - slope2) <= slope_tolerance_ && std::abs(intercept1 - intercept2) <= intercept_tolerance_) {
          return true;
      } else {
          return false;
      }
  }
}

cv::Vec4i StemEstimator::findLongerLine(const cv::Vec4i& segment1, const cv::Vec4i& segment2) 
{
  cv::Point2i pt1(segment1[0], segment1[1]);
  cv::Point2i pt2(segment1[2], segment1[3]);
  cv::Point2i pt3(segment2[0], segment2[1]);
  cv::Point2i pt4(segment2[2], segment2[3]);

  cv::Point2i pt14 = (pt1 + pt4) / 2;
  cv::Point2i pt23 = (pt2 + pt3) / 2;
  float dist14to23 = static_cast<float>((pt14.x - pt23.x) * (pt14.x - pt23.x) + (pt14.y - pt23.y) * (pt14.y - pt23.y));

  cv::Point2i pt12 = (pt1 + pt2) / 2;
  cv::Point2i pt34 = (pt3 + pt4) / 2;
  float dist12to34 = static_cast<float>((pt12.x - pt34.x) * (pt12.x - pt34.x) + (pt12.y - pt34.y) * (pt12.y - pt34.y));

  return (dist14to23 > dist12to34) ? cv::Vec4i(pt14.x, pt14.y, pt23.x, pt23.y) : cv::Vec4i(pt12.x, pt12.y, pt34.x, pt34.y);
}

std::vector<active_perception_planner::Line2D> StemEstimator::computeLinesUsing2DContours(const std::vector<pointcloud_roi::SemanticInstanceDetection>& stem_detections)
{
  std::vector<active_perception_planner::Line2D> box_lines;
  uint idx = 0; 
  for(const auto& det: stem_detections)
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(det.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2)
                                                   { return cv::contourArea(c1) > cv::contourArea(c2);} );

    std::vector<cv::Point> c = contours[0];
  
    cv::RotatedRect rect = cv::minAreaRect(c);

    if (cv::contourArea(c) > 30) 
    {
      cv::Point2f box[4];
      rect.points(box);
      // std::cout << "Box: ";
      // for (int i = 0; i < 4; ++i) {
      //     std::cout << box[i] << " ";
      // }
      //std::cout << std::endl;

      cv::Point2i pt1 = cv::Point2i(box[0].x, box[0].y);
      cv::Point2i pt2 = cv::Point2i(box[1].x, box[1].y);
      cv::Point2i pt3 = cv::Point2i(box[2].x, box[2].y);
      cv::Point2i pt4 = cv::Point2i(box[3].x, box[3].y);

      active_perception_planner::Line2D line2d;
      line2d.line_points = findLongerLine(cv::Vec4i(pt1.x, pt1.y, pt2.x, pt2.y), cv::Vec4i(pt3.x, pt3.y, pt4.x, pt4.y));
      line2d.vector_idx = idx;
      line2d.instance_id = det.instance_id;
      box_lines.push_back(line2d);
    }
    idx++;
  }
  ROS_INFO_STREAM("Box Lines Size: "<<box_lines.size());
  return box_lines;
}

std::vector<std::vector<active_perception_planner::Line2D>> StemEstimator::groupApproxCollinearSegments(const std::vector<active_perception_planner::Line2D>& lines)
{
  int n = lines.size();
  std::vector<int> indices_of_lines_already_grouped;
  std::vector<std::vector<active_perception_planner::Line2D>> groups;

  for (int i = 0; i < n; ++i) 
  {
    if (std::find(indices_of_lines_already_grouped.begin(), indices_of_lines_already_grouped.end(), i) != indices_of_lines_already_grouped.end()) 
    {
      continue;
    }

    indices_of_lines_already_grouped.push_back(i);

    cv::Point2f segment1[2] = {cv::Point2f(lines[i].line_points[0], lines[i].line_points[1]), cv::Point2f(lines[i].line_points[2], lines[i].line_points[3])};
    std::vector<active_perception_planner::Line2D> group = {lines[i]};

    for (int j = i + 1; j < n; ++j) 
    {
      if (std::find(indices_of_lines_already_grouped.begin(), indices_of_lines_already_grouped.end(), j) != indices_of_lines_already_grouped.end()) 
      {
        continue;
      }

      cv::Point2f segment2[2] = {cv::Point2f(lines[j].line_points[0], lines[j].line_points[1]), cv::Point2f(lines[j].line_points[2], lines[j].line_points[3])};

      if (areSegmentsApproximatelyCollinear(segment1, segment2)) 
      {
        group.push_back(lines[j]);
        indices_of_lines_already_grouped.push_back(j);
      }
    }
    if (!group.empty()) {
        groups.push_back(group);
    }
  }
  ROS_INFO_STREAM("Group Size: "<<groups.size());
  return groups;
}

void StemEstimator::regress3DLine(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& line_origin, Eigen::Vector3f& line_direction)
{
  // Compute centroid of the point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  line_origin = centroid.head<3>();
  // Compute covariance matrix
  // Eigen::Matrix3f covariance;
  // pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
    
  // // Perform PCA and obtain eigenvalues and eigenvectors
  // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  // Eigen::Matrix3f eigenvectors = solver.eigenvectors();
  // line_direction = eigenvectors.col(0);

// RANSAC parameters
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);  // Set model type to line
  seg.setMethodType(pcl::SAC_RANSAC);   // Use RANSAC algorithm
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.02);  // Distance threshold for inliers

  // RANSAC segmentation
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  line_direction.x() = coefficients->values[3];
  line_direction.y() = coefficients->values[4];
  line_direction.z() = coefficients->values[5];

  line_origin.x() = coefficients->values[0];
  line_origin.y() = coefficients->values[1];
  line_origin.z() = coefficients->values[2];
}

std::vector<Stem> StemEstimator::combine3DSegmentsFromGrouped2DSegments(const std::vector<std::vector<active_perception_planner::Line2D>>& grouped_segments)
{
  std::vector<Stem> stems;
  for(unsigned i = 0; i < grouped_segments.size(); ++i)
  {
    auto group = grouped_segments[i];
    active_perception_planner::Stem stem;
    stem.line_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(const auto& seg: group)
    {
      auto det = stem_detections_[seg.vector_idx];
      if(det.cloud_segment == nullptr || stem.line_cloud->size() < single_stem_min_points_)
      {
        ROS_INFO_STREAM("Stem: det cloud seg either nullptr or too small");
        continue;
      }
      else
      {
        ROS_INFO_STREAM("Stem: det cloud size: "<<stem.line_cloud->size());
      }
      if(det.instance_id == seg.instance_id)
      {
        *stem.line_cloud +=  *det.cloud_segment;
      }
    }
    if(stem.line_cloud ==nullptr || stem.line_cloud->size() < merged_stem_min_points_)
    {
      ROS_INFO_STREAM("Merged Stem cloud either nullptr or too small");
      continue;
    }
    regress3DLine(stem.line_cloud, stem.line_origin, stem.line_direction);
    stem.diameter = calculateApproximateDiameterAverageMetric(stem.line_cloud, stem.line_origin, stem.line_direction);
    findTopAndBottomPoints(stem.line_cloud, stem.top_point, stem.bottom_point);
    ROS_INFO_STREAM_THROTTLE(1.0, "Stem id: "<<stem.stem_id<<"\t bottom: "<<stem.bottom_point.transpose()<<"\t top: "<<stem.top_point.transpose()<<"\t dir: "<<stem.line_direction.transpose());
    if(isStemTooShort(stem.top_point, stem.bottom_point, 0.03f))
      continue;
    if(isStemApproximatelyParallelToZAxis(stem.line_direction, 30.0f) == false)
      continue;
    
    ROS_INFO_STREAM_THROTTLE(1.0, "Stem id: "<<stem.stem_id<<"\t bottom: "<<stem.bottom_point.transpose()<<"\t top: "<<stem.top_point.transpose()<<"\t dir: "<<stem.line_direction.transpose());
    stems.push_back(stem);
    
  }
  stems.erase(std::remove_if(stems.begin(), stems.end(), [](const Stem& s) {
        return (s.line_cloud == nullptr || s.line_cloud->empty());
    }), stems.end());

  return stems;
}
float StemEstimator::computeCloudOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) 
{
  if(cloud1 == nullptr || cloud2 == nullptr)
    return 0.0f;
  
  pcl::PointCloud<pcl::PointXYZ> intersection;
  pcl::PointCloud<pcl::PointXYZ> union_cloud;
  pcl::PointCloud<pcl::PointXYZ> difference_cloud;

  // Compute intersection
  pcl::intersection(cloud1, cloud2, intersection);

  // Compute union
  pcl::concatenatePointCloud(cloud1, cloud2, union_cloud);
  pcl::unionCloud(union_cloud, difference_cloud, union_cloud);

  // Compute overlap ratio
  float overlap_ratio = static_cast<float>(intersection.size()) / static_cast<float>(union_cloud.size());
  return overlap_ratio;
}

void StemEstimator::mergeNewWithExistingStems(const std::vector<Stem>& new_stems)
{

  std::unordered_set<int>new_stems_indices_already_merged; 
  for(int i = 0; i < stems_.size(); ++i)
  {
    bool found_similar_stem = false; 
    for(int j = 0; j < new_stems.size(); ++j)
    {
      if(new_stems_indices_already_merged.count(j) > 0 || found_similar_stem)
      {
        continue;
      }
      //ROS_INFO("1");
      bool is_same_stem = checkIfStemsAreSame(stems_[i], new_stems[j], 0.3, 0.05);
      //ROS_INFO("2");
      if(is_same_stem)
      {
        found_similar_stem = true;
        //ROS_INFO("3");
        if(new_stems[j].line_cloud !=nullptr && stems_[i].line_cloud !=nullptr)
          *(stems_[i].line_cloud) += *(new_stems[j].line_cloud);
        else
          continue;
        //ROS_INFO("4");
        
        new_stems_indices_already_merged.insert(j);
        //j = new_stems.size();
      }
    }
  }
  for(int j = 0; j < new_stems.size(); ++j)
  {
    if(new_stems_indices_already_merged.count(j) < 1 && new_stems[j].line_cloud !=nullptr)
    {
      stems_.push_back(new_stems[j]);
    }
  }
  stems_.erase(std::remove_if(stems_.begin(), stems_.end(), [](const Stem& s) {
        return (s.line_cloud == nullptr || s.line_cloud->empty());
    }), stems_.end());

  auto stems_temp = stems_;
  stems_.clear();
  for (int i = 0; i < stems_temp.size(); ++i)
  {
    if(stems_temp[i].line_cloud == nullptr || stems_temp[i].line_cloud->size() < merged_stem_min_points_)
      continue;
      //ROS_INFO("5");
    regress3DLine(stems_temp[i].line_cloud, stems_temp[i].line_origin, stems_temp[i].line_direction);
    //ROS_INFO("6");
    findTopAndBottomPoints(stems_temp[i].line_cloud, stems_[i].top_point, stems_temp[i].bottom_point);

    if(isStemTooShort(stems_temp[i].top_point, stems_temp[i].bottom_point, 0.05))
      continue;
    if(isStemApproximatelyParallelToZAxis(stems_temp[i].line_direction, 15.0f) == false)
      continue;
    
    ROS_INFO_STREAM_THROTTLE(1.0, "Stem id: "<<stems_temp[i].stem_id<<"\t bottom: "<<stems_temp[i].bottom_point.transpose()<<"\t top: "<<stems_temp[i].top_point.transpose()<<"\t dir: "<<stems_temp[i].line_direction.transpose());

    stems_.push_back(stems_temp[i]);
    //ROS_INFO("7")
  }
}

bool StemEstimator::checkIfStemsAreSame(const Stem& st1, const Stem st2, const float& dir_threshold, const float& xy_threshold)
{
  // Calculate the difference in direction
  float dir_diff = std::acos(st1.line_direction.dot(st2.line_direction));

  // Calculate the intersection points of the lines with the xy-plane (z = 0)
  Eigen::Vector2f intersection1 = st1.line_origin.head<2>() - (st1.line_origin.z() / st1.line_direction.z()) * st1.line_direction.head<2>();
  Eigen::Vector2f intersection2 = st2.line_origin.head<2>() - (st2.line_origin.z() / st2.line_direction.z()) * st2.line_direction.head<2>();

  // Calculate the Euclidean distance between the intersection points
  float distance = (intersection1 - intersection2).norm();

  float overlap_ratio = computeCloudOverlap(st1.line_cloud, st2.line_cloud);
  // Check if the difference in direction and distance in XY plane are within thresholds
  return ((dir_diff < dir_threshold && distance < xy_threshold) || overlap_ratio > 0.3);
}

float StemEstimator::calculateApproximateDiameterAverageMetric(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& line_cloud, const Eigen::Vector3f& line_origin, const Eigen::Vector3f& line_direction) 
{
  float total_distance = 0.0;
  for (const auto& point : *line_cloud) {
      Eigen::Vector3f vector2line = point.getVector3fMap() - line_origin;
      float dist2line = vector2line.cross(line_direction).norm();
      total_distance += dist2line;
  }

  // Calculate the average distance
  float average_distance = total_distance / line_cloud->size();

  return average_distance;
}

void StemEstimator::findTopAndBottomPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& top_point, Eigen::Vector3f& bottom_point) 
{
  //ROS_INFO_STREAM("findTopAndBottomPoints 1"<<cloud->points.size());

  std::sort(cloud->begin(), cloud->end(), [](const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b) {
        return a.z < b.z;
  });
  bottom_point = cloud->front().getVector3fMap();
  top_point    = cloud->back().getVector3fMap();
  
  //ROS_INFO("findTopAndBottomPoints 4");
}

bool StemEstimator::isStemTooShort(const Eigen::Vector3f& top_point, const Eigen::Vector3f& bottom_point, float min_height) 
{
  auto stem_height = fabs(top_point.z() - bottom_point.z());
  return (stem_height < min_height);
}

bool StemEstimator::isStemApproximatelyParallelToZAxis(const Eigen::Vector3f& line_direction, float threshold_angle_degrees) 
{
    // Define unit vector along the z-axis
    Eigen::Vector3f z_axis = Eigen::Vector3f::UnitZ();

    // Compute the cosine of the angle between the line direction and the z-axis
    float cosine_angle = line_direction.dot(z_axis) / (line_direction.norm() * z_axis.norm());

    // Compute the angle in degrees
    float angle_degrees = std::acos(cosine_angle) * 180.0f / M_PI;

    // Check if the angle is within the threshold
    return angle_degrees < threshold_angle_degrees;
}


