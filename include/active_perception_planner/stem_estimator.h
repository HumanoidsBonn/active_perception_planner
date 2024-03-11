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

    virtual ~StemEstimator();

    void semanticInstanceDetectionsCallback(const std::shared_ptr<pointcloud_roi::SemanticInstanceDetections>& dets);

    std::vector<Stem> getStems();

    private:
    std::vector<pointcloud_roi::SemanticInstanceDetection> stem_detections_;
    std::vector<Stem> stems_;
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    double vertical_angle_threshold_ = 5.0;
    double slope_tolerance_, intercept_tolerance_;

    bool lines_not_init_ = true;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread processing_thread_;
    std::atomic_bool is_running_;
    std::mutex mutex_dets_;
    std::mutex mutex_stems_;

    std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets_;

    int single_stem_min_points_ = 10;
    int merged_stem_min_points_ = 10;


    void setCurrentStemDetections(std::shared_ptr<pointcloud_roi::SemanticInstanceDetections> dets);

    void processStemDetections();

    void mergeSegmentsWithinSameDetection(std::vector<Stem>& stems);

    void mergeNewWithExistingStems(const std::vector<Stem>& new_stems);

    bool checkIfStemsAreSame(const Stem& st1, const Stem st2, const float& dir_threshold, const float& xy_threshold);

    bool checkLineParallelToYAxis(double theta);

    cv::Vec4i findLongerLine(const cv::Vec4i& segment1, const cv::Vec4i& segment2);

    std::vector<active_perception_planner::Line2D> computeLinesUsing2DContours(const std::vector<pointcloud_roi::SemanticInstanceDetection>& stem_detections);
    
    bool areSegmentsApproximatelyCollinear(cv::Point2f segment1[2], cv::Point2f segment2[2]);

    std::vector<std::vector<active_perception_planner::Line2D>> groupApproxCollinearSegments(const std::vector<active_perception_planner::Line2D>& lines);

    std::vector<Stem> combine3DSegmentsFromGrouped2DSegments(const std::vector<std::vector<active_perception_planner::Line2D>>& grouped_segments);

    void regress3DLine(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& line_origin, Eigen::Vector3f& line_direction);
    
    float calculateApproximateDiameterAverageMetric(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& line_cloud, const Eigen::Vector3f& line_origin, const Eigen::Vector3f& line_direction);
    
    void findTopAndBottomPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& top_point, Eigen::Vector3f& bottom_point);

    void calculateStemParameters(Stem& stem);
    
    bool isStemTooShort(const Eigen::Vector3f& top_point, const Eigen::Vector3f& bottom_point, float min_height = 0.05f);

    void setStems(const std::vector<Stem>& stems);

  float computeCloudOverlap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);

    bool isStemApproximatelyParallelToZAxis(const Eigen::Vector3f& line_direction, float threshold_angle_degrees);

    std::vector<Stem> createStemsFromCurrentDetections(const std::vector<pointcloud_roi::SemanticInstanceDetection>& stem_detections);
    
    void processingThread();


};
}

#endif