#include <active_perception_planner/active_perception_interface.h>

using namespace active_perception_planner;

ActivePercieverInterface::ActivePercieverInterface():nhp_("~")
{
  srv_detect_fruits_ = nh_.advertiseService("detect_fruits", &ActivePercieverInterface::processDetectFruitsServiceCallback, this);
  srv_detect_fruits_ = nh_.advertiseService("start_follow_fruit", &ActivePercieverInterface::processStartFollowFruitsServiceCallback, this);
  srv_detect_fruits_ = nh_.advertiseService("stop_follow_fruit", &ActivePercieverInterface::processStopFollowFruitsServiceCallback, this);
}

bool ActivePercieverInterface::processDetectFruitsServiceCallback(oc2_interface_msgs::DetectFruits::Request &req, oc2_interface_msgs::DetectFruits::Response &res)
{
  if(active_perciever_->clearSceneMap() == false)
  {
    ROS_ERROR("Could not clear VPP Scene Map");
    return false;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_perciever_node");
  ros::NodeHandle nh;

  ActivePercieverInterface act_per_interface;
  ROS_INFO("Ready to Percieve.");
  ros::spin();

  return 0;
}
