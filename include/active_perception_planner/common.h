#ifndef ACTIVE_PERCEPTION_PLANNER_COMMON_H
#define ACTIVE_PERCEPTION_PLANNER_STEM_ESTIMATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pointcloud_roi/detected_roi_filter.h>

namespace active_perception_planner
{
struct Stem
{
  pcl::PointCloud<pcl::PointSurfel>::Ptr line_cloud;
  Eigen::Vector3f line_origin;
  Eigen::Vector3f line_direction;
  Eigen::Vector3f top_point, bottom_point;
  float diameter;
};

struct Crop
{
  std::vector<Stem> stems;
};

} // namespace active_perception_planner



#endif