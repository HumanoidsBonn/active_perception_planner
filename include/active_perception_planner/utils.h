#ifndef ACTIVE_PERCEPTION_PLANNER_UTILS_H
#define ACTIVE_PERCEPTION_PLANNER_UTILS_H

#include<Eigen/Core>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

inline double computeFruitCentreDistance(const Eigen::Vector3d& c1, const Eigen::Vector3d& c2)
{
  return (c1 -c2).norm();
}

inline double computeFruitCentreDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  Eigen::Vector3d c1, c2;
  tf::pointMsgToEigen(p1, c1);
  tf::pointMsgToEigen(p2, c2);

  return computeFruitCentreDistance(c1, c2);
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

inline geometry_msgs::Point fromEigen2GeometryMsgPoint(const Eigen::Vector3f& point)
{
  geometry_msgs::Point point_msg;
  point_msg.x = point(0);
  point_msg.y = point(1);
  point_msg.z = point(2);

  return point_msg;
}

inline geometry_msgs::Vector3 fromEigen2GeometryMsgVector3(const Eigen::Vector3f& point)
{
  geometry_msgs::Vector3 point_msg;
  point_msg.x = point(0);
  point_msg.y = point(1);
  point_msg.z = point(2);

  return point_msg;
}

inline void computeBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& min_point, Eigen::Vector3f& max_point, Eigen::Vector3f& bounding_box_dimensions)
{
    // Compute the bounding box
    //pcl::getMinMax3D(*cloud, min_point, max_point);

    // Compute the dimensions of the bounding box
    bounding_box_dimensions = max_point - min_point;
}

/**
 * @brief dirVecToQuat computes quaternion closest to camQuat with x-axis aligned to dirVec
 * @param dirVec desired x-axis direction
 * @param camQuat camera orientation
 * @param viewDir camera view direction (x-axis)
 * @return computed quaternion
 */
inline geometry_msgs::Quaternion dirVecToQuat(Eigen::Vector3d &dirVec, const tf2::Quaternion &camQuat, const tf2::Vector3 &viewDir)
{
  dirVec.normalize();    
  Eigen::Vector3d camera_z = Eigen::Vector3d::UnitZ();
  if(dirVec.y() != 0.0)
  {
    camera_z.y() = -dirVec.z()/dirVec.y();
  }

  camera_z.normalize();

  Eigen::Vector3d camera_y = camera_z.cross(dirVec);

  Eigen::Matrix3d camera_rot;

  camera_rot.col(0) = dirVec;
  camera_rot.col(1) = camera_y;
  camera_rot.col(2) = camera_z;

  Eigen::Quaterniond camera_quat_eigen(camera_rot);

  return tf2::toMsg (camera_quat_eigen);
}

inline bool arePosesApproximatelyEqual(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, double position_tolerance = 0.02, double orientation_tolerance = 0.1) {
    // Check position
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    double position_distance = sqrt(dx * dx + dy * dy + dz * dz);

    // Check orientation
    double dot_product = pose1.orientation.x * pose2.orientation.x +
                         pose1.orientation.y * pose2.orientation.y +
                         pose1.orientation.z * pose2.orientation.z +
                         pose1.orientation.w * pose2.orientation.w;
    double orientation_distance = 1 - fabs(dot_product);

    // Check if both position and orientation are within tolerances
    return position_distance <= position_tolerance && orientation_distance <= orientation_tolerance;
}
inline void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
{
  pose.orientation = trans.rotation;
  pose.position.x = trans.translation.x;
  pose.position.y = trans.translation.y;
  pose.position.z = trans.translation.z;
}

inline void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
{
  trans.rotation = pose.orientation;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;
}

inline void convert(const geometry_msgs::TransformStamped& trans, geometry_msgs::PoseStamped& pose)
{
  convert(trans.transform, pose.pose);
  pose.header = trans.header;
}

inline void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
{
  convert(pose.pose, trans.transform);
  trans.header = pose.header;
}
}

#endif // UTILS_H
