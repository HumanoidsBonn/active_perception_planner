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
#include <pcl/sample_consensus/sac_model_polynomial.h>
#include <opencv2/opencv.hpp>
#include <pointcloud_roi/detected_roi_filter.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace active_perception_planner
{

class StemEstimator
{
    public:
    StemEstimator();

    void setCurrentStemDetections(std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets);

    void processStemDetections();

    std::vector<Stem> getStems();

    private:
    std::vector<SemanticInstanceDetection> stem_detections_;
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    double vertical_angle_threshold_ = 5.0;
    double slope_tolerance_, intercept_tolerance_;

    bool lines_not_init_ = true;

    bool checkLineParallelToYAxis(double theta);

    bool areSegmentsApproximatelyCollinear(cv::Point2f segment1[2], cv::Point2f segment2[2]);

    std::vector<std::vector<cv::Vec4i>> groupApproxCollinearSegments(const std::vector<cv::Vec4i>& lines);

    std::vector<cv::Vec4i> computeLinesUsing2DContours(const std::vector<SemanticInstanceDetection>& stem_detections);

    void detect3DLineFromSegments(const std::vector<std::vector<cv::Vec4i>>& groupedSegments);

    void regress3DLine(const PointCloudT::Ptr& cloud, Eigen::Vector3f& line_origin, Eigen::Vector3f& line_direction);


};
}

#endif