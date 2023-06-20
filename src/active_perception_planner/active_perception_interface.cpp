#include <active_perception_planner/active_perception_interface.h>

using namespace active_perception_planner;

ActivePercieverInterface::ActivePercieverInterface():nhp_("~")
{
  active_perciever_ = std::make_shared<active_perception_planner::ActivePerciever>(nhp_, std::string("head"), std::string("head_link_base"), std::string("head_link_eef"));
  srv_detect_fruits_ = nhp_.advertiseService("detect_fruits", &ActivePercieverInterface::processDetectFruitsServiceCallback, this);
  srv_start_follow_fruits_ = nhp_.advertiseService("start_follow_fruit", &ActivePercieverInterface::processStartFollowFruitsServiceCallback, this);
  srv_stop_follow_fruits_  = nhp_.advertiseService("stop_follow_fruit", &ActivePercieverInterface::processStopFollowFruitsServiceCallback, this);
  pub_superellipsoids_ = nhp_.advertise<sensor_msgs::PointCloud2>("completed_fruit_shapes_surface", 2, true);
  ROS_INFO_ONCE("ActivePercieverInterface");

}

bool ActivePercieverInterface::processDetectFruitsServiceCallback(oc2_interface_msgs::DetectFruits::Request &req, oc2_interface_msgs::DetectFruits::Response &res)
{
  if(active_perciever_->clearSceneMap() == false)
  {
    ROS_ERROR("Could not clear VPP Scene Map");
    //return false;
  }

  if(active_perciever_->performLocalMappingMotion() == false)
  {
    ROS_ERROR("Could not perform local mapping motion");
    return false;
  }
  vpp_msgs::GetListInstancePointclouds instcloud_srv;
  if(active_perciever_->createSceneMapWithCompletedFruits(instcloud_srv) == false)
  {
    ROS_ERROR("Could not create voxblox plus plus map of instance fruits");
    return false;
  }

  res = getFruitMapAsSrvResponse();
  publishVisualizations();
  return true;

}

bool ActivePercieverInterface::processStartFollowFruitsServiceCallback(oc2_interface_msgs::StartFollowFruit::Request &req, oc2_interface_msgs::StartFollowFruit::Response &res)
{
  active_perciever_->startFollowFruit(req.fruit_id);
  return true;
}

bool ActivePercieverInterface::processStopFollowFruitsServiceCallback(oc2_interface_msgs::StopFollowFruit::Request &req, oc2_interface_msgs::StopFollowFruit::Response &res)
{
  active_perciever_->stopFollowFruit();
  return true;
}

oc2_interface_msgs::DetectFruits::Response ActivePercieverInterface::getFruitMapAsSrvResponse()
{
  oc2_interface_msgs::DetectFruits::Response res;
  res.status_code = int(active_perciever_->getActivePercieverStatus());
  res.success = false;
  if(active_perciever_->getActivePercieverStatus() == FRUIT_MAP_GENERATION_SUCCESS)
  {
    res.success = true;

    res.scene_pointcloud = active_perciever_->getScenePointCloud();
    FruitMap fruit_map   = active_perciever_->getFruitMap();
    FruitMapIterator it;

    for(it = fruit_map.begin(); it != fruit_map.end(); ++it)
    {
      oc2_interface_msgs::DetectedFruit fruit;
      fruit.fruit_id = it->second.fruit_id;
      fruit.fruit_pose.position = it->second.fruit_centre;
      pcl::toROSMsg(*(it->second.observed_fruit_shape), fruit.observed_fruit);
      pcl::toROSMsg(*(it->second.completed_fruit_shape), fruit.predicted_fruit);

      res.detected_fruits.detected_fruits.push_back(fruit);
    }
  }
  return res;
}

void ActivePercieverInterface::publishVisualizations()
{
  FruitMap fruit_map   = active_perciever_->getFruitMap();
  FruitMapIterator it;
  pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 debug_pc_ros;
  for(it = fruit_map.begin(); it != fruit_map.end(); ++it)
  {
    *(debug_pc) += *(it->second.completed_fruit_shape);
  }
  pcl::toROSMsg(*debug_pc, debug_pc_ros);
  debug_pc_ros.header.stamp = ros::Time::now();
  debug_pc_ros.header.frame_id = "world";

  pub_superellipsoids_.publish(debug_pc_ros);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_perciever_node");
  ros::NodeHandle nh; 
  ros::AsyncSpinner spinner(0);
  ActivePercieverInterface act_per_interface;
  ROS_INFO("Ready to Percieve.");
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
