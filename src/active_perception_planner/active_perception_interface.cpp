#include <active_perception_planner/active_perception_interface.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include<tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "active_perception_planner/utils.h"
using namespace active_perception_planner;

ActivePercieverInterface::ActivePercieverInterface():nhp_("~"), tfListener_(tfBuffer_)
{
  nhp_.param<bool>("disable_motion", disable_motion_, false);
  nhp_.param<double>("wait_time", wait_time_, 60.0);
  nhp_.param<bool>("disable_d405_processing", disable_d405_processing_, false);

  active_perciever_ = std::make_shared<active_perception_planner::ActivePerciever>(nhp_, std::string("head"), std::string("head_link_base"), std::string("head_link_eef"), disable_motion_);
  srv_detect_fruits_ = nhp_.advertiseService("detect_fruits", &ActivePercieverInterface::processDetectFruitsServiceCallback, this);
  srv_start_follow_fruits_ = nhp_.advertiseService("start_follow_fruit", &ActivePercieverInterface::processStartFollowFruitsServiceCallback, this);
  srv_stop_follow_fruits_  = nhp_.advertiseService("stop_follow_fruit", &ActivePercieverInterface::processStopFollowFruitsServiceCallback, this);
  srv_start_follow_peduncle_  = nhp_.advertiseService("start_follow_peduncle", &ActivePercieverInterface::processStartFollowPeduncleServiceCallback, this);
  srv_save_js_ = nhp_.advertiseService("save_joint_state", &ActivePercieverInterface::processSaveJointStatesCallback, this);


  client_start_depth_seg_    = nhp_.serviceClient<std_srvs::Empty>("/gsm_node/enable_gsm_srv");
  client_stop_depth_seg_     = nhp_.serviceClient<std_srvs::Empty>("/gsm_node/disable_gsm_srv");
  client_start_obj_det_      = nhp_.serviceClient<std_srvs::Empty>("/yolo_segmentation_ros_node/start");
  client_stop_obj_det_       = nhp_.serviceClient<std_srvs::Empty>("/yolo_segmentation_ros_node/stop"); //d405_yolo_segmentation_ros_node
  client_set_confidence_iou_ = nhp_.serviceClient<yolact_ros_msgs::SetConfidence>("/yolo_segmentation_ros_node/set_confidence_iou");

  if(disable_d405_processing_ == false)
  {
    client_start_depth_seg_d405_ = nhp_.serviceClient<std_srvs::Empty>("/d405_depth_segmentation_node/start");
    client_stop_depth_seg_d405_  = nhp_.serviceClient<std_srvs::Empty>("/d405_depth_segmentation_node/stop");
    client_start_obj_det_d405_   = nhp_.serviceClient<std_srvs::Empty>("/d405_yolo_segmentation_ros_node/start");
    client_stop_obj_det_d405_  = nhp_.serviceClient<std_srvs::Empty>("/d405_yolo_segmentation_ros_node/stop");
  } 

  act_detection_server_.reset(new DetectionServer(nhp_, "/detectFruitsAction", false));
	act_detection_server_->registerGoalCallback(boost::bind(&ActivePercieverInterface::executeDetectionServerGoalCallback, this));
  act_detection_server_->registerPreemptCallback(boost::bind(&ActivePercieverInterface::preemptDetectionServerGoalCallback, this));


  pub_superellipsoids_ = nhp_.advertise<sensor_msgs::PointCloud2>("completed_fruit_shapes_surface", 2, true);

  pub_crop_ = nhp_.advertise<oc2_interface_msgs::Crop>("crop", 1, true);

  std::string config_package;
  nhp_.param<std::string>("config_package", config_package, "active_perception_planner");
  config_package_path_ = ros::package::getPath(config_package);

  if(disable_motion_ == false)
    loadJointStatesasRememberedJointValues();

  

  //  if(active_perciever_->moveToInitialMappingPose() == false)
//  {
//   throw std::runtime_error("Cannot move head arm to transport pose");
//  }

  act_detection_server_->start();
  ROS_INFO_ONCE("ActivePercieverInterface");
}

void ActivePercieverInterface::executeDetectionServerGoalCallback()
{
  if(!act_detection_server_->isNewGoalAvailable())
		return;

	const auto& goal = act_detection_server_->acceptNewGoal();

  yolact_ros_msgs::SetConfidence srv_set_conf_iou;
  srv_set_conf_iou.request.confidence_threshold = 0.8;
  srv_set_conf_iou.request.confidence_threshold = 0.5;

  if (!client_set_confidence_iou_.call(srv_set_conf_iou))
  {
    ROS_ERROR("Failed to call service");
  }

  callEmptyService(client_start_obj_det_);
  callEmptyService(client_start_depth_seg_);
  if(disable_d405_processing_ == false)
  {
    //callEmptyService(client_start_depth_seg_d405_);
    callEmptyService(client_start_obj_det_d405_);
  }
  
  if(active_perciever_->clearSceneMap() == false)
  {
    ROS_ERROR("Could not clear VPP Scene Map");
    detection_result_.success = false;
    detection_result_.message = "Could not clear Scene Map";
    act_detection_server_->setAborted(detection_result_);
    return;
  }
  sleep(1.0);
  if(disable_motion_ == false)
  {
    if(active_perciever_->performLocalMappingMotion() == false)
    {
      ROS_ERROR("Could not perform local mapping motion");
      detection_result_.success = false;
      detection_result_.message = "Could not perform local mapping motion";
      act_detection_server_->setAborted(detection_result_);
      return;
    }
  }
  else
  {
    ROS_WARN_STREAM("Local mapping motion disabled. Sleeping for "<<wait_time_<< "seconds");
    sleep(wait_time_);
  }
  vpp_msgs::GetListInstancePointclouds instcloud_srv;
  ROS_WARN_STREAM("Starting Scene Generation with fruits");
  callEmptyService(client_stop_obj_det_);
  callEmptyService(client_stop_depth_seg_);
  if(disable_d405_processing_ == false)
  {
    //callEmptyService(client_stop_depth_seg_d405_);
    callEmptyService(client_stop_obj_det_d405_);
  }

  if(active_perciever_->createSceneMapWithCompletedFruits(instcloud_srv) == false)
  {
    ROS_ERROR("Could not create voxblox plus plus map of instance fruits");
    detection_result_.success = false;
    detection_result_.message = "Could not create voxblox plus plus map of instance fruits";
    act_detection_server_->setAborted(detection_result_);
    return;
  }
  detection_result_.success = true;
  detection_result_ = getCropAsActionResult();
  publishVisualizations();
  act_detection_server_->setSucceeded(detection_result_);
  ROS_INFO("Detect Fruit Action Succeeded");
}

void ActivePercieverInterface::preemptDetectionServerGoalCallback()
{
  ROS_INFO("Detect Fruit Action Preempted");
  act_detection_server_->setPreempted();
}

bool ActivePercieverInterface::processDetectFruitsServiceCallback(oc2_interface_msgs::DetectFruits::Request &req, oc2_interface_msgs::DetectFruits::Response &res)
{
  //callEmptyService(client_start_depth_seg_);
  callEmptyService(client_start_obj_det_);
  if(disable_d405_processing_ == false)
  {
    //callEmptyService(client_start_depth_seg_d405_);
    callEmptyService(client_start_obj_det_d405_);
  }
  
  if(active_perciever_->clearSceneMap() == false)
  {
    ROS_ERROR("Could not clear VPP Scene Map");
    res.success = false;
    return true;
  }
  if(disable_motion_ == false)
  {
    if(active_perciever_->performLocalMappingMotion() == false)
    {
      ROS_ERROR("Could not perform local mapping motion");
      res.success = false;
      return true;
    }
  }
  else
  {
    ROS_WARN_STREAM("Local mapping motion disabled. Sleeping for "<<wait_time_<< "seconds");
    sleep(wait_time_);
  }
  sleep(1.0);
  callEmptyService(client_stop_obj_det_);
  if(disable_d405_processing_ == false)
  {
    //callEmptyService(client_stop_depth_seg_d405_);
    callEmptyService(client_stop_obj_det_d405_);
  }
  vpp_msgs::GetListInstancePointclouds instcloud_srv;
  ROS_WARN_STREAM("Starting Scene Generation with fruits");
  if(active_perciever_->createSceneMapWithCompletedFruits(instcloud_srv) == false)
  {
    ROS_ERROR("Could not create voxblox plus plus map of instance fruits");
    res.success = false;
    return true;
  }
  res.success = true;
  res = getFruitMapAsSrvResponse();
  publishVisualizations();
  //callEmptyService(client_stop_depth_seg_);
  
  return true;

}

bool ActivePercieverInterface::processStartFollowPeduncleServiceCallback(oc2_interface_msgs::StartFollowPeduncle::Request &req, oc2_interface_msgs::StartFollowPeduncle::Response &res)
{
  callEmptyService(client_stop_depth_seg_);
  yolact_ros_msgs::SetConfidence srv_set_conf_iou;
  srv_set_conf_iou.request.confidence_threshold = 0.3;
  srv_set_conf_iou.request.confidence_threshold = 0.3;
  
  if (!client_set_confidence_iou_.call(srv_set_conf_iou))
  {
    ROS_ERROR("Failed to call service");
  }

  res.success = true;
  return true;
}

bool ActivePercieverInterface::processStartFollowFruitsServiceCallback(oc2_interface_msgs::StartFollowFruit::Request &req, oc2_interface_msgs::StartFollowFruit::Response &res)
{
  //callEmptyService(client_start_depth_seg_);
  callEmptyService(client_start_obj_det_);
  if(disable_d405_processing_ == false)
  {
    //callEmptyService(client_start_depth_seg_d405_);
    callEmptyService(client_start_obj_det_d405_);
  }
  res.success = false;
  if(active_perciever_->startFollowFruit(req.fruit_id))
  {
    res.success = true;
  }
  return true;
}

bool ActivePercieverInterface::processStopFollowFruitsServiceCallback(oc2_interface_msgs::StopFollowFruit::Request &req, oc2_interface_msgs::StopFollowFruit::Response &res)
{
  active_perciever_->stopFollowFruit();
  //callEmptyService(client_stop_depth_seg_);
  callEmptyService(client_stop_obj_det_);
  if(disable_d405_processing_ == false)
  {
    //callEmptyService(client_stop_depth_seg_d405_);
    callEmptyService(client_stop_obj_det_d405_);
  }
  res.success = true;
  return true;
}

oc2_interface_msgs::DetectFruits::Response ActivePercieverInterface::getFruitMapAsSrvResponse()
{
  oc2_interface_msgs::DetectFruits::Response res;
  // res.status_code = int(active_perciever_->getActivePercieverStatus());
  // res.success = false;
  // if(active_perciever_->getActivePercieverStatus() == FRUIT_MAP_GENERATION_SUCCESS)
  // {
  //   res.success = true;

  //   res.scene_pointcloud = active_perciever_->getScenePointCloud();
  //   FruitMap fruit_map   = active_perciever_->getFruitMap();
  //   FruitMapIterator it;

  //   for(it = fruit_map.begin(); it != fruit_map.end(); ++it)
  //   {
  //     oc2_interface_msgs::DetectedFruit fruit;
  //     fruit.fruit_id = it->second.fruit_id;
  //     fruit.fruit_pose = it->second.fruit_pose;
  //     fruit.score = it->second.observation_score;
  //     pcl::toROSMsg(*(it->second.observed_fruit_shape), fruit.observed_fruit);
  //     pcl::toROSMsg(*(it->second.completed_fruit_shape), fruit.predicted_fruit);

  //     res.detected_fruits.header.frame_id =  "trolley_local_link";
  //     res.detected_fruits.detected_fruits.push_back(fruit);
  //   }
  // }
  return res;
}

// oc2_interface_msgs::DetectFruitsResult ActivePercieverInterface::getFruitMapAsActionResult()
// {
//   oc2_interface_msgs::DetectFruitsResult res;
//   res.status_code = int(active_perciever_->getActivePercieverStatus());
//   res.success = false;
//   if(active_perciever_->getActivePercieverStatus() == FRUIT_MAP_GENERATION_SUCCESS)
//   {
//     res.success = true;

//     FruitMap fruit_map   = active_perciever_->getFruitMap();
//     FruitMapIterator it;

//     for(it = fruit_map.begin(); it != fruit_map.end(); ++it)
//     {
//       oc2_interface_msgs::DetectedFruit fruit;
//       fruit.fruit_id = it->second.fruit_id;
//       fruit.fruit_pose = it->second.fruit_pose;
//       fruit.score = it->second.observation_score;
//       pcl::toROSMsg(*(it->second.observed_fruit_shape), fruit.observed_fruit);
//       pcl::toROSMsg(*(it->second.completed_fruit_shape), fruit.predicted_fruit);

//       res.detected_fruits.header.frame_id =  "trolley_local_link";
//       res.detected_fruits.detected_fruits.push_back(fruit);
//     }
//   }
//   return res;
// }

oc2_interface_msgs::DetectFruitsResult ActivePercieverInterface::getCropAsActionResult()
{
  oc2_interface_msgs::DetectFruitsResult res;
  res.status_code = int(active_perciever_->getActivePercieverStatus());
  res.success = false;
  res.crop.header.frame_id = "trolley_local_link";

  if(active_perciever_->getActivePercieverStatus() == FRUIT_MAP_GENERATION_SUCCESS)
  {
    res.success = true;
    
    Crop crop = active_perciever_->getCrop();
    FruitWithPeduncleMapIterator it;
    for(it = crop.fruit_with_peduncle_map.begin(); it != crop.fruit_with_peduncle_map.end(); ++it)
    {
      oc2_interface_msgs::DetectedFruit fruit;
      fruit.header.frame_id = "trolley_local_link";
      fruit.fruit_id = it->second.fruit.fruit_id;
      fruit.fruit_pose = it->second.fruit.fruit_pose;
      fruit.fruit_dimensions = it->second.fruit.fruit_size;
      fruit.score = it->second.fruit.observation_score;
      fruit.associated_stem_id = it->second.nearest_stem;
      fruit.has_peduncle = it->second.has_peduncle;

      if(it->second.fruit.observed_fruit_shape != nullptr)
      {
        pcl::toROSMsg(*(it->second.fruit.observed_fruit_shape), fruit.observed_fruit);  
        //pcl::toROSMsg(it->second.fruit.observed_fruit_shape->header, fruit.observed_fruit.header); 
        fruit.observed_fruit.header.frame_id = it->second.fruit.observed_fruit_shape->header.frame_id;
      }

      if(it->second.fruit.completed_fruit_shape != nullptr)
      {
        pcl::toROSMsg(*(it->second.fruit.completed_fruit_shape), fruit.predicted_fruit);  
        fruit.predicted_fruit.header.frame_id = it->second.fruit.completed_fruit_shape->header.frame_id; 
      }

      fruit.peduncle.associated_fruit_id = it->second.peduncle.associated_fruit_id;
      fruit.peduncle.peduncle_id  = it->second.peduncle.peduncle_id;
      fruit.peduncle.stempoint    = utils::fromEigen2GeometryMsgPoint(it->second.peduncle.stempoint);
      fruit.peduncle.fruitpoint   = utils::fromEigen2GeometryMsgPoint(it->second.peduncle.fruitpoint);

      
      fruit.nearby_stems.insert(fruit.nearby_stems.begin(), it->second.nearby_stems.begin(), it->second.nearby_stems.end());

      if(it->second.peduncle.peduncle_cloud != nullptr)
      {
        pcl::toROSMsg(*(it->second.peduncle.peduncle_cloud), fruit.peduncle.cloud);     
        fruit.peduncle.cloud.header.frame_id = it->second.peduncle.peduncle_cloud->header.frame_id;
        //pcl::toROSMsg(it->second.peduncle.peduncle_cloud->header, fruit.peduncle.cloud.header); 
      }
      ROS_INFO_STREAM("ROS Msg Peduncle has "<< (fruit.peduncle.cloud.width*fruit.peduncle.cloud.height));

      res.crop.detected_fruits.push_back(fruit);
    }
    int stem_idx = 0; 
    for(const auto& stem : crop.stems)
    {
      stem_idx++;
      oc2_interface_msgs::Stem stem_msg;
      stem_msg.stem_id = stem.stem_id;
      stem_msg.direction = utils::fromEigen2GeometryMsgVector3(stem.line_direction);
      stem_msg.origin    = utils::fromEigen2GeometryMsgPoint(stem.line_origin);
      stem_msg.bottom    = utils::fromEigen2GeometryMsgPoint(stem.bottom_point);
      stem_msg.origin    = utils::fromEigen2GeometryMsgPoint(stem.top_point);
      stem_msg.diameter  = stem.diameter;

      res.crop.stems.push_back(stem_msg);
    }
  }
  pub_crop_.publish(res.crop);
  return res;
}

void ActivePercieverInterface::publishVisualizations()
{
  FruitMap fruit_map   = active_perciever_->getFruitMap();
  FruitMapIterator it;
  pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 debug_pc_ros, cloud_out;
  for(it = fruit_map.begin(); it != fruit_map.end(); ++it)
  {
    *(debug_pc) += *(it->second.completed_fruit_shape);
  }
  pcl::toROSMsg(*debug_pc, debug_pc_ros);
  debug_pc_ros.header.stamp = ros::Time::now();
  debug_pc_ros.header.frame_id = "trolley_local_link";


  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform("world","trolley_local_link",
            ros::Time(0));
    std::cout << transformStamped << std::endl;
    tf2::doTransform(debug_pc_ros, cloud_out, transformStamped);
    pub_superellipsoids_.publish(cloud_out);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }

}

void ActivePercieverInterface::loadJointStatesasRememberedJointValues()
{
  std::string poses_filepath = config_package_path_ + "/config/" + "head_joint_states.yaml";

  YAML::Node config = YAML::LoadFile(poses_filepath);

  if(config.IsMap())
  {
    for(YAML::const_iterator it=config.begin();it!=config.end();++it)
    {
      std::vector<double> joint_values = it->second.as<std::vector<double>>();
      active_perciever_->getMoveGroupHandle()->rememberJointValues(it->first.as<std::string>(), joint_values);
    }
  }
  else
  {
    ROS_ERROR("YAML File not recongized as a map");
  }

}

bool ActivePercieverInterface::processSaveJointStatesCallback(oc2_interface_msgs::SaveJointState::Request& req, oc2_interface_msgs::SaveJointState::Response& res)
{
  std::string poses_filepath = config_package_path_ + "/config/" + "head_joint_states.yaml";
  std::map< std::string, std::vector< double > > remembered_map = active_perciever_->getMoveGroupHandle()->getRememberedJointValues();
  auto it = remembered_map.find(req.name);
  if(it !=remembered_map.end())
  {
    req.name = req.name + "_" + std::to_string(round(ros::Time::now().toSec()));
  }
  active_perciever_->getMoveGroupHandle()->rememberJointValues(req.name);
  std::map< std::string, std::vector< double > > remembered_map_new = active_perciever_->getMoveGroupHandle()->getRememberedJointValues();


  YAML::Emitter out;
//  for(auto it = remembered_map_new.begin(); it !=remembered_map_new.end(); ++it)
//  {
//    out << YAML::BeginSeq;
//    std::string name = it->first + ":\n";
//    out << YAML::Literal<<name;
//    out << it->second;
//    out <<YAML::EndSeq;
//  }
  out << remembered_map_new;
  assert(out.good());

  std::ofstream file(poses_filepath, std::ios::trunc);
  if (!file.is_open())
  {
    ROS_ERROR("Could not open file");
    res.success = false;
    return true;
  }
  file<<out.c_str();
  file.close();
  res.success = true;
  return true;

}

bool ActivePercieverInterface::callEmptyService(ros::ServiceClient& client)
{
  std_srvs::Empty empty_srv;
  if (!client.call(empty_srv))
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
  return true;
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
