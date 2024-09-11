#ifndef ACTIVE_PERCEPTION_PLANNER_EVALUATION_INTERFACE_H
#define ACTIVE_PERCEPTION_PLANNER_EVALUATION_INTERFACE_H

#include "ros/ros.h"
#include "oc2_interface_msgs/DetectFruits.h"
#include "oc2_interface_msgs/StartFollowFruit.h"
#include "oc2_interface_msgs/StopFollowFruit.h"
#include "oc2_interface_msgs/SaveJointState.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include<active_perception_planner/active_perciever.h>

#include <actionlib/server/simple_action_server.h>
#include <oc2_interface_msgs/DetectFruitsAction.h>
#include <oc2_interface_msgs/StartFollowPeduncle.h>

#include <yolact_ros_msgs/SetConfidence.h>
#include <yolact_ros_msgs/SetRegionOfInterest.h>
#include <std_srvs/Empty.h>

#include "active_perception_planner/fruit_shape_estimator.h"
namespace active_perception_planner
{

class EvaluationInterface
{
public:
  EvaluationInterface();

  bool processEvaluateFineServiceCalllback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool processEvaluateCoarseServiceCalllback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool processEvaluateCoarse2FineServiceCalllback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool saveResults(FruitMap& fruit_map);

protected:

  ros::NodeHandle nh_, nhp_;

  std::shared_ptr<active_perception_planner::FruitShapeEstimator> fruit_shape_estimator_;

  ros::ServiceServer srv_evaluate_fine_, srv_evaluate_coarse, srv_evaluate_coarse2fine;

  ros::ServiceClient get_instanceclouds_client_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  std::string config_package_path_;

  std::string folder_path_;

  std::vector<std::string> headers_  = {"fruit_id", "c_x", "c_y", "c_z", "q_w", "q_x", "q_y", "q_z", "volume", "obs_c_x", "obs_c_y", "obs_c_z", "l_x", "l_y", "l_z", "fit_score", "obs_score"};
;

  std::string csv_filename_;

protected:

  bool createCSV(const std::vector<std::string>& headers, FruitMap& fruit_map, std::string filename="results.csv");
  void processAndSaveClouds(FruitMap& fruit_map);


};
}
#endif // ACTIVE_PERCEPTION_PLANNER_EVALUATION_INTERFACE_H
