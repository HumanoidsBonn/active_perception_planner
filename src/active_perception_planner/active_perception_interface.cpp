#include <active_perception_planner/active_perception_interface.h>

using namespace active_perception_planner;


bool ActivePercieverInterface::processDetectFruitsServiceCallback(const oc2_interface_msgs::DetectFruits::Request &req, oc2_interface_msgs::DetectFruits::Response &res)
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


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_perciever_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
