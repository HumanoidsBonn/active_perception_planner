#ifndef ACTIVE_PERCEPTION_PLANNER_UTILS_H
#define ACTIVE_PERCEPTION_PLANNER_UTILS_H

#include<Eigen/Core>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>

namespace utils{

//https://stackoverflow.com/questions/7168484/3d-line-segment-and-plane-intersection

inline bool calcRayPlaneIntersection(Eigen::Vector3d& intersection_point, const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_dir, const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_point)
{
  if(ray_dir.dot(plane_normal) < 0.001f)
    return false;

  double d = plane_normal.dot(plane_point);

  double x = (d - plane_normal.dot(ray_origin))/(plane_normal.dot(ray_dir));

  intersection_point = ray_origin + x * ray_dir;

  return true;
}

inline bool calcLinePlaneIntersection(Eigen::Vector3d& intersection_point, const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_point, const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_point)
{
  Eigen::Vector3d ray_dir = (ray_point - ray_origin);
  ray_dir = ray_dir/ray_dir.norm();
  return calcRayPlaneIntersection(intersection_point, ray_origin, ray_dir, plane_normal, plane_point);
}

inline bool isFruitCentreSimilar(const Eigen::Vector3d& c1, const Eigen::Vector3d& c2, double threshold = 0.01)
{
  if((c1-c2).norm()> threshold)
    return false;

  return true;
}

inline bool isFruitCentreSimilar(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, double threshold = 0.01)
{
  Eigen::Vector3d c1, c2;
  tf::pointMsgToEigen(p1, c1);
  tf::pointMsgToEigen(p2, c2);

  return isFruitCentreSimilar(c1, c2, threshold);
}

inline tf2::Quaternion computeQuaternionfromDirVec(const Eigen::Vector3d& dir_vec, const tf2::Vector3 &view_dir, const tf2::Quaternion& curr_cam_quat)
{
  auto d1= dir_vec.normalized();
  tf2::Vector3 dir_vec_tf = tf2::Vector3(d1(0), d1(1), d1(2));
  tf2::Vector3 rotAx = view_dir.cross(dir_vec_tf);
  double rotAng = view_dir.angle(dir_vec_tf);
  tf2::Quaternion toRot(rotAx, rotAng);
  tf2::Quaternion view_quat = toRot * curr_cam_quat;
  return view_quat;
}

inline geometry_msgs::Pose fromTF2GeometryMsg(const Eigen::Vector3d& point, const tf2::Quaternion& view_quat)
{
  geometry_msgs::Pose view_pose;
  tf::pointEigenToMsg(point, view_pose.position);
  view_pose.orientation = tf2::toMsg(view_quat);
  return view_pose;
}

inline geometry_msgs::Pose fromEigen2GeometryMsg(const Eigen::Vector3d& point, const Eigen::Vector3d& dir_vec, const tf2::Vector3 &view_dir, const tf2::Quaternion& curr_cam_quat)
{
  tf2::Quaternion view_quat = computeQuaternionfromDirVec(dir_vec, view_dir, curr_cam_quat);
  return fromTF2GeometryMsg(point, view_quat);
}


}

#endif // UTILS_H
