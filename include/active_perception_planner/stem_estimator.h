#ifndef ACTIVE_PERCEPTION_PLANNER_STEM_ESTIMATOR_H
#define ACTIVE_PERCEPTION_PLANNER_STEM_ESTIMATOR_H

#include "active_perception_planner/common.h"

#include<vector>
#include<memory>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <opencv2/opencv.hpp>
#include <pointcloud_roi/detected_roi_filter.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace active_perception_planner
{

struct Line2D
{
  uint vector_idx;
  u_int32_t instance_id;
  cv::Vec4i line_points;
};

class StemEstimator
{
    public:
    StemEstimator();

    void setCurrentStemDetections(std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets);

    void processStemDetections();

    std::vector<Stem> getStems();

    private:
    std::vector<pointcloud_roi::SemanticInstanceDetection> stem_detections_;
    std::vector<Stem> stems_;
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    double vertical_angle_threshold_ = 5.0;
    double slope_tolerance_, intercept_tolerance_;

    bool lines_not_init_ = true;

    bool checkLineParallelToYAxis(double theta);

    cv::Vec4i findLongerLine(const cv::Vec4i& segment1, const cv::Vec4i& segment2);

    std::vector<active_perception_planner::Line2D> computeLinesUsing2DContours(const std::vector<pointcloud_roi::SemanticInstanceDetection>& stem_detections);
    
    bool areSegmentsApproximatelyCollinear(cv::Point2f segment1[2], cv::Point2f segment2[2]);

    std::vector<std::vector<active_perception_planner::Line2D>> groupApproxCollinearSegments(const std::vector<active_perception_planner::Line2D>& lines);

    void combine3DSegmentsFromGrouped2DSegments(const std::vector<std::vector<active_perception_planner::Line2D>>& grouped_segments);

    void regress3DLine(const pcl::PointCloud<PointSurfelLabel>::Ptr& cloud, Eigen::Vector3f& line_origin, Eigen::Vector3f& line_direction);
    
    float calculateApproximateDiameterAverageMetric(const pcl::PointCloud<PointSurfelLabel>::Ptr& line_cloud, const Eigen::Vector3f& line_origin, const Eigen::Vector3f& line_direction);
    
    void findTopAndBottomPoints(const pcl::PointCloud<PointSurfelLabel>::Ptr& cloud, Eigen::Vector3f& top_point, Eigen::Vector3f& bottom_point);


};
}

#endif