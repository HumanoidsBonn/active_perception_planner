#ifndef ACTIVE_PERCEPTION_PLANNER_UTILS_POINTCLOUD_DISTANCES_H
#define ACTIVE_PERCEPTION_PLANNER_UTILS_POINTCLOUD_DISTANCES_H

#include <iostream>
#include<fstream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include<fstream>
#include <string>
#include <vector>
#include <filesystem>


using namespace std;


using std::string;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

using PointType = PointXYZRGB;
using Cloud = PointCloud<PointXYZRGB>;

namespace fs = std::filesystem;

namespace utils
{
template<typename TreeT, typename PointT>
float nearestDistance(const TreeT& tree, const PointT& pt)
{
  const int k = 1;
  std::vector<int> indices (k);
  std::vector<float> sqr_distances (k);

  tree.nearestKSearch(pt, k, indices, sqr_distances);

  return sqr_distances[0];
}

// compare cloudB to cloudA
// use threshold for identifying outliers and not considering those for the similarity
// a good value for threshold is 5 * <cloud_resolution>, e.g. 10cm for a cloud with 2cm resolution
template<typename CloudT>
float _similarity(CloudT& cloudA, CloudT& cloudB, float threshold)
{
  // compare B to A
  int num_outlier = 0;
  pcl::search::KdTree<typename CloudT::PointType> tree;
  tree.setInputCloud(cloudA.makeShared());
  auto sum = std::accumulate(cloudB.begin(), cloudB.end(), 0.0f, [&](auto current_sum, const auto& pt) {
    const auto dist = nearestDistance(tree, pt);

    if(dist < threshold)
    {
      return current_sum + dist;
    }
    else
    {
      num_outlier++;
      return current_sum;
    }
  });

  return sum / (cloudB.size() - num_outlier);
}

// comparing the clouds each way, A->B, B->A and taking the average
template<typename CloudT>
float similarity(CloudT& cloudA, CloudT& cloudB, float threshold = std::numeric_limits<float>::max())
{
  TicToc tt;
  tt.tic ();
  // compare B to A
  const auto similarityB2A = _similarity(cloudA, cloudB, threshold);
  // compare A to B
  const auto similarityA2B = _similarity(cloudB, cloudA, threshold);

  float dist = (similarityA2B * 0.5f) + (similarityB2A * 0.5f);

//  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
//  print_info ("A->B: "); print_value ("%f", similarityB2A);
//  print_info (", B->A: "); print_value ("%f", similarityA2B);
//  print_info (", Chamfer Distance: "); print_value ("%f", dist);
//  print_info (" ]\n");
  return dist;
}

typedef struct Point_3D
{
  double x;  //mm world coordinate x
  double y;  //mm world coordinate y
  double z;  //mm world coordinate z
  int r;
  int g;
  int b;

}POINT_WORLD;

using std::string;



struct lin
{
    double a[6];
};

float computeDistanceMin(Cloud &cloud_a, Cloud &cloud_b)
{
  // Estimate
  TicToc tt;
  tt.tic ();

//  print_highlight (stderr, "Computing ");

  // compare A to B
  pcl::search::KdTree<PointType> tree_b;
  tree_b.setInputCloud (cloud_b.makeShared ());
  float max_dist_a = std::numeric_limits<float>::max ();
  for (const auto &point : cloud_a.points)
  {
      pcl::Indices indices (1);
      std::vector<float> sqr_distances (1);

      tree_b.nearestKSearch (point, 1, indices, sqr_distances);
      if (sqr_distances[0] < max_dist_a)
      max_dist_a = sqr_distances[0];
  }

  // compare B to A
  pcl::search::KdTree<PointType> tree_a;
  tree_a.setInputCloud (cloud_a.makeShared ());
  float max_dist_b = std::numeric_limits<float>::max ();
  for (const auto &point : cloud_b.points)
  {
      pcl::Indices indices (1);
      std::vector<float> sqr_distances (1);

      tree_a.nearestKSearch (point, 1, indices, sqr_distances);
      if (sqr_distances[0] < max_dist_b)
      max_dist_b = sqr_distances[0];
  }

  max_dist_a = std::sqrt (max_dist_a);
  max_dist_b = std::sqrt (max_dist_b);

  float dist = std::max (max_dist_a, max_dist_b);

//  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
//  print_info ("A->B: "); print_value ("%f", max_dist_a);
//  print_info (", B->A: "); print_value ("%f", max_dist_b);
//  print_info (", Chamfer Distance 2: "); print_value ("%f", dist);
//  print_info (" ]\n");

  return dist;
}


float computeDistanceChamferFull(float& dist_a_b, Cloud &cloud_a, Cloud &cloud_b)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  // compare A to B
  pcl::search::KdTree<PointType> tree_b;
  tree_b.setInputCloud (cloud_b.makeShared ());
  float sum_a = 0; //std::numeric_limits<float>::max ();
  for (const auto &point : cloud_a.points)
  {
      pcl::Indices indices (1);
      std::vector<float> sqr_distances (1);

      tree_b.nearestKSearch (point, 1, indices, sqr_distances);

      sum_a = sum_a + std::sqrt(sqr_distances[0]);
  }

  // compare B to A
  pcl::search::KdTree<PointType> tree_a;
  tree_a.setInputCloud (cloud_a.makeShared ());
  float sum_b = 0;
  for (const auto &point : cloud_b.points)
  {
      pcl::Indices indices (1);
      std::vector<float> sqr_distances (1);

      tree_a.nearestKSearch (point, 1, indices, sqr_distances);
      sum_b = sum_b + std::sqrt(sqr_distances[0]);
  }

  float max_dist_a = float(sum_a)/cloud_a.points.size();
  float max_dist_b = sum_b/cloud_b.points.size();;

  float dist = max_dist_a + max_dist_b;
  dist_a_b = max_dist_a;

//  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
//  print_info ("A->B: "); print_value ("%f", max_dist_a);
//  print_info (", B->A: "); print_value ("%f", max_dist_b);
//  print_info (", Chamfer Distance Full: "); print_value ("%f", dist);
//  print_info (" ]\n");

  return dist;
}

float computeDistanceHausdorff(Cloud &cloud_a, Cloud &cloud_b)
{
  // Estimate
  TicToc tt;
  tt.tic ();

//  print_highlight (stderr, "Computing ");

  // compare A to B
  pcl::search::KdTree<PointType> tree_b;
  tree_b.setInputCloud (cloud_b.makeShared ());
  float max_dist_a = -std::numeric_limits<float>::max ();
  for (const auto &point : cloud_a.points)
  {
      pcl::Indices indices (1);
      std::vector<float> sqr_distances (1);

      tree_b.nearestKSearch (point, 1, indices, sqr_distances);
      if (sqr_distances[0] > max_dist_a)
      max_dist_a = sqr_distances[0];
  }

  // compare B to A
  pcl::search::KdTree<PointType> tree_a;
  tree_a.setInputCloud (cloud_a.makeShared ());
  float max_dist_b = -std::numeric_limits<float>::max ();
  for (const auto &point : cloud_b.points)
  {
      pcl::Indices indices (1);
      std::vector<float> sqr_distances (1);

      tree_a.nearestKSearch (point, 1, indices, sqr_distances);
      if (sqr_distances[0] > max_dist_b)
      max_dist_b = sqr_distances[0];
  }

  max_dist_a = std::sqrt (max_dist_a);
  max_dist_b = std::sqrt (max_dist_b);

  float dist = std::max (max_dist_a, max_dist_b);

//  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
//  print_info ("A->B: "); print_value ("%f", max_dist_a);
//  print_info (", B->A: "); print_value ("%f", max_dist_b);
//  print_info (", Hausdorff Distance: "); print_value ("%f", dist);
//  print_info (" ]\n");

  return dist;
}

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPointCloudByBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                                                     const Eigen::Vector3d& bbox_min,
                                                                     const Eigen::Vector3d& bbox_max) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Iterate through each point in the cloud
    for (const auto& point : cloud->points) {
        // Extract XYZ coordinates from the point
        double x = point.x;
        double y = point.y;
        double z = point.z;

        // Check if the point lies within the bounding box
        if (x >= bbox_min.x() && x <= bbox_max.x() &&
            y >= bbox_min.y() && y <= bbox_max.y() &&
            z >= bbox_min.z() && z <= bbox_max.z()) {
            // Point lies within the bounding box, add it to the filtered cloud
            filtered_cloud->points.push_back(point);
        }
    }

    // Set width and height of the filtered cloud
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;

    return filtered_cloud;
}


}

#endif // ACTIVE_PERCEPTION_PLANNER_UTILS_POINTCLOUD_DISTANCES_H
