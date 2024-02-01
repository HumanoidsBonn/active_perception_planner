#include "active_perception_planner/stem_estimator.h"

StemEstimator::StemEstimator()
{

}

void StemEstimator::setCurrentStemDetections(std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets)
{
  rgb_image_ = dets->rgb_image.copy();
  depth_image_ = dets->depth_image.copy();
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

std::vector<cv::Vec4i> StemEstimator::computeLinesUsing2DContours(const std::vector<SemanticInstanceDetection>& stem_detections)
{

}

std::vector<std::vector<cv::Vec4i>> StemEstimator::groupApproxCollinearSegments(const std::vector<cv::Vec4i>& lines)
{
  int n = lines.size();
  std::vector<int> indices_of_lines_already_grouped;
  std::vector<std::vector<cv::Vec4i>> groups;

  for (int i = 0; i < n; ++i) 
  {
    if (std::find(indices_of_lines_already_grouped.begin(), indices_of_lines_already_grouped.end(), i) != indices_of_lines_already_grouped.end()) 
    {
      continue;
    }

    indices_of_lines_already_grouped.push_back(i);

    cv::Point2f segment1[2] = {cv::Point2f(lines[i][0], lines[i][1]), cv::Point2f(lines[i][2], lines[i][3])};
    std::vector<cv::Vec4i> group = {lines[i]};

    for (int j = i + 1; j < n; ++j) 
    {
      if (std::find(indices_of_lines_already_grouped.begin(), indices_of_lines_already_grouped.end(), j) != indices_of_lines_already_grouped.end()) 
      {
        continue;
      }

      cv::Point2f segment2[2] = {cv::Point2f(lines[j][0], lines[j][1]), cv::Point2f(lines[j][2], lines[j][3])};

      if (areSegmentsApproximatelyCollinear(segment1, segment2, slopeTolerance, interceptTolerance)) 
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