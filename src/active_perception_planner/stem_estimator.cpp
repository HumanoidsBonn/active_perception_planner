#include "active_perception_planner/stem_estimator.h"
using namespace active_perception_planner;

StemEstimator::StemEstimator()
{

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

void StemEstimator::combine3DSegmentsFromGrouped2DSegments(const std::vector<std::vector<active_perception_planner::Line2D>>& grouped_segments)
{
  for(const auto& group: grouped_segments)
  {

  }
}
