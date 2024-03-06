#ifndef ACTIVE_PERCEPTION_PLANNER_COMMON_H
#define ACTIVE_PERCEPTION_PLANNER_COMMON_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pointcloud_roi/detected_roi_filter.h>
#include<capsicum_superellipsoid_detector/se_detector.h>
#include <unordered_set>

namespace active_perception_planner
{
struct Stem
{
  int16_t stem_id;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_cloud;
  Eigen::Vector3f line_origin;
  Eigen::Vector3f line_direction;
  Eigen::Vector3f top_point, bottom_point;
  float diameter;
};

struct Peduncle
{
  bool is_estimation = false;
  int16_t peduncle_id;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr peduncle_cloud;
  Eigen::Vector3f fruitpoint, stempoint;
  uint32_t associated_fruit_id;
  geometry_msgs::Pose associated_fruit_pose;
};

struct DetectedFruitWithPeduncle
{
  superellipsoid::DetectedFruit fruit;
  bool has_peduncle = false;
  Peduncle peduncle;
  std::vector<int> nearby_stems;
  int nearest_stem;

  DetectedFruitWithPeduncle()
  {}

  DetectedFruitWithPeduncle(superellipsoid::DetectedFruit _fruit,  Peduncle _peduncle):fruit(_fruit), peduncle(_peduncle)
  {
    //has_peduncle = (peduncle.is_estimation==false);
  }

  DetectedFruitWithPeduncle(superellipsoid::DetectedFruit _fruit):fruit(_fruit)
  {}
};

struct Crop
{
  std::unordered_map<uint32_t, DetectedFruitWithPeduncle> fruit_with_peduncle_map;
  std::vector<Stem> stems;

  Crop(){}

  Crop(const std::unordered_map<uint32_t, DetectedFruitWithPeduncle>& _fruit_with_peduncle_map, const std::vector<Stem>& _stems):fruit_with_peduncle_map(_fruit_with_peduncle_map), stems(_stems)
  {}
};


struct ObservedFruit
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fruit_observed;
  uint32_t fruit_id;
  Eigen::Vector3d fruit_obs_centroid;
};

typedef std::map<uint, superellipsoid::DetectedFruit> FruitMap;
typedef std::map<uint, superellipsoid::DetectedFruit>::iterator FruitMapIterator;

typedef std::unordered_map<uint32_t, DetectedFruitWithPeduncle> FruitWithPeduncleMap;
typedef std::unordered_map<uint32_t, DetectedFruitWithPeduncle>::iterator FruitWithPeduncleMapIterator;


struct InterestedFruit: public superellipsoid::DetectedFruit
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_observation;

  InterestedFruit():superellipsoid::DetectedFruit(), current_observation(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
  }

  InterestedFruit(const superellipsoid::DetectedFruit& det_fruit):current_observation(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    this->valid_shape_completion_available = det_fruit.valid_shape_completion_available;
    this->observed_fruit_shape = det_fruit.observed_fruit_shape;
    this->completed_fruit_shape = det_fruit.completed_fruit_shape;
    this->fruit_pose = det_fruit.fruit_pose;
    this->observed_shape_centroid = det_fruit.observed_shape_centroid;
  }
};

} // namespace active_perception_planner



#endif