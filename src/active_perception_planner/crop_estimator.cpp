#include "active_perception_planner/crop_estimator.h"
#include <pcl/features/moment_of_inertia_estimation.h>


using namespace active_perception_planner;

CropEstimator::CropEstimator()
{

}

Crop CropEstimator::createCrop(const FruitMap& fruit_map, const std::unordered_map<uint32_t, Peduncle>& peduncle_map, const std::vector<Stem>& stems)
{
  //Crop crop;
  ROS_INFO_STREAM("Creating fruit_with_peduncle_map");
  auto fruit_with_peduncle_map = attachDetectedPedunclesToFruits(fruit_map, peduncle_map);
  ROS_INFO_STREAM("Created fruit_with_peduncle_map");
  if(stems.size()> 0)
  {
    ROS_INFO_STREAM("Creating associateStemsWithFruits");
    associateStemsWithFruits(fruit_with_peduncle_map, stems, 0.08f, 0.04f);
    ROS_INFO_STREAM("Created associateStemsWithFruits");
  }
  
  ROS_INFO_STREAM("Creating crop");
  Crop crop(fruit_with_peduncle_map, stems);
  ROS_INFO_STREAM("Created crop");
  return crop;
}

std::unordered_map<uint32_t, DetectedFruitWithPeduncle> CropEstimator::attachDetectedPedunclesToFruits(const FruitMap& fruit_map, const std::unordered_map<uint32_t, Peduncle>& peduncle_map)
{
  std::unordered_map<uint32_t, DetectedFruitWithPeduncle> fruit_with_peduncle_map;
  std::unordered_set<int> assigned_peduncles;

  for(auto itf = fruit_map.begin(); itf != fruit_map.end(); ++itf)
  {
    auto fruit = itf->second;
    bool found_peduncle_fruit_association = false;
    for(auto itp = peduncle_map.begin(); itp != peduncle_map.end(); ++itp)
    {
      auto peduncle = itp->second;
      if(assigned_peduncles.count(peduncle.associated_fruit_id) > 0 || found_peduncle_fruit_association)
      {
        ROS_INFO("Skipping peduncle as already assigned");
        continue;
      }
      calculatePeduncleEndpoints(peduncle);
      bool attached = isPeduncleAtTopCenterOfFruit(fruit, peduncle, 0.02);
      if(attached)
      {
        found_peduncle_fruit_association = true;
        assigned_peduncles.insert(peduncle.associated_fruit_id);
        DetectedFruitWithPeduncle fruit_with_peduncle(fruit, peduncle);
        fruit_with_peduncle_map[fruit.fruit_id] = fruit_with_peduncle;
        fruit_with_peduncle_map[fruit.fruit_id].has_peduncle = true;
        fruit_with_peduncle_map[fruit.fruit_id].peduncle.associated_fruit_id = fruit.fruit_id;
        continue;
      }
    }
    if(found_peduncle_fruit_association == false)
    {
      DetectedFruitWithPeduncle fruit_with_dummy_peduncle(fruit);
      fruit_with_peduncle_map[fruit.fruit_id] = fruit_with_dummy_peduncle;
      fruit_with_peduncle_map[fruit.fruit_id].has_peduncle = false;
    }
  }
  assert(fruit_with_peduncle_map.size() == fruit_map.size());
  
  return fruit_with_peduncle_map;
}

void CropEstimator::associateStemsWithFruits(std::unordered_map<uint32_t, DetectedFruitWithPeduncle>& fruit_with_peduncle_map, const std::vector<Stem>& stems, const float& belonging_threshold, const float& nearness_threshold)
{
  float min_dist = std::numeric_limits<float>::max();
  int nearest_stem_idx = -1;
  for(auto itf = fruit_with_peduncle_map.begin(); itf != fruit_with_peduncle_map.end(); ++itf)
  {
    DetectedFruitWithPeduncle& fruit_with_peduncle = itf->second;
    bool has_belonging_stem = false;
    for(int stem_idx = 0; stem_idx < stems.size(); ++stem_idx)
    {
      auto stem = stems[stem_idx];
      float dist = pointLineDistance(fruit_with_peduncle.fruit.fruit_pose.position, stem.line_origin, stem.line_direction);
      if(dist < nearness_threshold)
      {
        fruit_with_peduncle_map[itf->first].nearby_stems.push_back(stem_idx);
      }
      if(dist <= belonging_threshold)
      {
        has_belonging_stem = true;
        if(dist < min_dist)
        {
          min_dist = dist;
          nearest_stem_idx = stem_idx;
        }
      }
    }
    if(has_belonging_stem)
    {
      fruit_with_peduncle_map[itf->first].nearest_stem = nearest_stem_idx;
      auto nearest_stem = stems[nearest_stem_idx];
      estimatePeduncleEndpointsWithNearestStem(fruit_with_peduncle, nearest_stem);
      fruit_with_peduncle_map[itf->first].peduncle = fruit_with_peduncle.peduncle;
      fruit_with_peduncle_map[itf->first].has_peduncle = true;
    }    
  }
}

Eigen::Vector3f CropEstimator::findIntersection(const Eigen::Vector3f& outside_point, const Eigen::Vector3f& line_origin, const Eigen::Vector3f& line_direction) 
{
    // Vector from line origin to the point outside the line
    Eigen::Vector3f lineToPoint = outside_point - line_origin;

    // Project the line-to-point vector onto the line direction
    float t = line_direction.dot(lineToPoint) / line_direction.squaredNorm();

    // Calculate the intersection point
    return line_origin + t * line_direction;
}

void CropEstimator::calculatePeduncleEndpoints(Peduncle& peduncle)
{
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud(peduncle.peduncle_cloud);
  feature_extractor.compute();

  Eigen::Matrix3f rotational_matrix;
  pcl::PointXYZRGB mass_center, min_point, max_point;

  feature_extractor.getOBB(min_point, max_point, mass_center, rotational_matrix);

  peduncle.fruitpoint = min_point.getVector3fMap();
  peduncle.stempoint  = max_point.getVector3fMap();
}

bool CropEstimator:: isPeduncleAtTopCenterOfFruit(superellipsoid::DetectedFruit& fruit, Peduncle& peduncle, const float& threshold)
{
  Eigen::Vector3f top_center;
  top_center(0) = fruit.fruit_pose.position.x;
  top_center(1) = fruit.fruit_pose.position.y;
  top_center(2) = fruit.fruit_pose.position.z + fruit.fruit_size.z;

  Eigen::Vector3f fruitpoint_gap = peduncle.fruitpoint - top_center;
  Eigen::Vector3f stempoint_gap  = peduncle.stempoint  - top_center;

  float fruitpoint_dist = fruitpoint_gap.norm();
  float stempoint_dist  = stempoint_gap.norm();

  if(fruitpoint_gap(2) < threshold || fruitpoint_dist < 0.04)
  {
    ROS_INFO("Peduncle attached to fruit at fruitpoint");
    peduncle.associated_fruit_id = fruit.fruit_id;
    return true;
  }
  else if(stempoint_gap(2) < threshold || stempoint_dist < 0.04)
  {
    ROS_INFO("Peduncle attached to fruit at original stempoint. Swapping them...");
    peduncle.associated_fruit_id = fruit.fruit_id;
    auto temp = peduncle.fruitpoint;
    peduncle.fruitpoint = peduncle.stempoint;
    peduncle.stempoint = temp;
    return true;
  }
  else
  {
    ROS_WARN_STREAM("Peduncle does not belong to fruit");
    return false;
  }
}

void CropEstimator::estimatePeduncleEndpointsWithNearestStem(DetectedFruitWithPeduncle& fruit_with_peduncle, const Stem& stem)
{
  Eigen::Vector3f top_center;
  top_center(0) = fruit_with_peduncle.fruit.fruit_pose.position.x;
  top_center(1) = fruit_with_peduncle.fruit.fruit_pose.position.y;
  top_center(2) = fruit_with_peduncle.fruit.fruit_pose.position.z + fruit_with_peduncle.fruit.fruit_size.z;

  fruit_with_peduncle.peduncle.stempoint = findIntersection(top_center, stem.line_origin, stem.line_direction);
  fruit_with_peduncle.peduncle.fruitpoint = top_center;
  fruit_with_peduncle.peduncle.is_estimation = true;
  fruit_with_peduncle.has_peduncle = true;
}

float CropEstimator::pointLineDistance(const geometry_msgs::Point& fruit_centre, const Eigen::Vector3f& line_origin, const Eigen::Vector3f& line_direction) 
{
  Eigen::Vector3f point = Eigen::Vector3f(fruit_centre.x, fruit_centre.y, fruit_centre.z);
  return (point - line_origin).cross(line_direction).norm();
}