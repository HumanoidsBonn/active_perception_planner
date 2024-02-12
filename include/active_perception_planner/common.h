#ifndef ACTIVE_PERCEPTION_PLANNER_COMMON_H
#define ACTIVE_PERCEPTION_PLANNER_STEM_ESTIMATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pointcloud_roi/detected_roi_filter.h>

namespace active_perception_planner
{
struct Stem
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_cloud;
  Eigen::Vector3f line_origin;
  Eigen::Vector3f line_direction;
  Eigen::Vector3f top_point, bottom_point;
  float diameter;
};

struct Crop
{
  std::vector<Stem> stems;
};

struct Peduncle
{
  int16_t peduncle_id;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr peduncle_cloud;
  Eigen::Vector3f fruitpoint, stempoint;
  uint32_t associated_fruit_id;
};

} // namespace active_perception_planner



#endif