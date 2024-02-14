#include "active_perception_planner/stem_estimator.h"
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
  std::scoped_lock(mutex_stems_);
  stems = stems_;
  return stems;
}

void StemEstimator::semanticInstanceDetectionsCallback(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets)
{
  ROS_WARN("StemEstimator: detectionCallback");
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
    ROS_WARN_STREAM("StemEstimator: Is running");
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
  std::vector<active_perception_planner::Line2D>  lines = computeLinesUsing2DContours(stem_detections_);
  std::vector<std::vector<active_perception_planner::Line2D>> grouped_segments = groupApproxCollinearSegments(lines);
  auto current_stems = combine3DSegmentsFromGrouped2DSegments(grouped_segments);
  if(stems_.empty())
  {
    stems_.insert(stems_.begin(), current_stems.begin(), current_stems.end());
  }
  else
  {
    mergeNewWithExistingStems(current_stems);
  }
}

void StemEstimator::setCurrentStemDetections(std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets)
{
  rgb_image_ = dets->rgb_image.clone();
  depth_image_ = dets->depth_image.clone();
  stem_detections_.clear();
  stem_detections_.insert(stem_detections_.begin(), dets->stems.begin(), dets->stems.end());
  lines_not_init_ = false;
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

    if (cv::contourArea(c) > 50) 
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

  return groups;
}

void StemEstimator::regress3DLine(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& line_origin, Eigen::Vector3f& line_direction)
{
  // Compute centroid of the point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  line_origin = centroid.head<3>();
  // Compute covariance matrix
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
    
  // Perform PCA and obtain eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  Eigen::Matrix3f eigenvectors = solver.eigenvectors();
  line_direction = eigenvectors.col(0);
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
      if(det.instance_id == seg.instance_id)
      {
        *stem.line_cloud +=  *det.cloud_segment;
      }
    }
    regress3DLine(stem.line_cloud, stem.line_origin, stem.line_direction);
    stem.diameter = calculateApproximateDiameterAverageMetric(stem.line_cloud, stem.line_origin, stem.line_direction);
    findTopAndBottomPoints(stem.line_cloud, stem.top_point, stem.bottom_point);
    stems.push_back(stem);
  }
  return stems;
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
      bool is_same_stem = checkIfStemsAreSame(stems_[i], new_stems[j], 0.1, 0.1);
      if(is_same_stem)
      {
        found_similar_stem = true;
        *(stems_[i].line_cloud) += *(new_stems[i].line_cloud);
        regress3DLine(stems_[i].line_cloud, stems_[i].line_origin, stems_[i].line_direction);
        findTopAndBottomPoints(stems_[i].line_cloud, stems_[i].top_point, stems_[i].bottom_point);
        new_stems_indices_already_merged.insert(j);
        //j = new_stems.size();
      }
    }
  }
  for(int j = 0; j < new_stems.size(); ++j)
  {
    if(new_stems_indices_already_merged.count(j) < 1)
    {
      stems_.push_back(new_stems[j]);
    }
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


  // Check if the difference in direction and distance in XY plane are within thresholds
  return (dir_diff < dir_threshold && distance < xy_threshold);
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
  auto minZ = std::min_element(cloud->begin(), cloud->end(), [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
      return p1.z < p2.z;
  });

  auto maxZ = std::max_element(cloud->begin(), cloud->end(), [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
      return p1.z > p2.z;
  });

  top_point = minZ->getVector3fMap();
  bottom_point = maxZ->getVector3fMap();
}
