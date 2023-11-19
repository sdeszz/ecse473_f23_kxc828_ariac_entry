#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Proximity.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

#include "std_msgs/String.h"
#include <sstream>

geometry_msgs::PoseStamped part_pose, goal_pose;
osrf_gear::GetMaterialLocations my_get;
std::vector<osrf_gear::Model> type;
std::vector<osrf_gear::LogicalCameraImage> tt;
std::vector<osrf_gear::Order> int_vector;
std::vector<osrf_gear::LogicalCameraImage> image;
std::vector<osrf_gear::LogicalCameraImage> qmage;
std::vector<osrf_gear::LogicalCameraImage> rmage;
tf2_ros::Buffer tfBuffer;
//std::vector<osrf_gear::> servise_geta_succeeded;

geometry_msgs::TransformStamped tfStamped;


int flag = 0;
int oldflag = 0;
int service_call_succeeded,servise_geta_succeeded;


void chatterCallback(const osrf_gear::Order::ConstPtr& msg)
{
  //int_vector.clear();
  int_vector.push_back(*msg);
  ROS_INFO("Order type: %s",int_vector[0].shipments[0].products[0].type.c_str());    
  
}

void cameraaCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  //image.clear();
  //ROS_INFO("Callbcak!");
  //ROS_INFO("%s",msg->models[0].type.c_str());
  //type = *msg->models;
  //tt = *msg;
  flag += 1;
  image.push_back(*msg);
  
}

void camerabCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  
  qmage.push_back(*msg);
  
}

void cameracCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  
  rmage.push_back(*msg);
  
}


int main(int argc, char **argv)
{
  std_srvs::Trigger begin_comp;
  std_srvs::SetBool my_bool_var;
  
  
  ros::init(argc, argv, "node_source");
  
  my_bool_var.request.data = true;
  static bool warn_flag = false;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, chatterCallback);
  ros::Subscriber asub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_1", 1000, cameraaCallback);
  ros::Subscriber bsub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_2", 1000, camerabCallback);
  ros::Subscriber csub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_3", 1000, cameracCallback);
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");  
  ros::ServiceClient begin_anclient = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations"); 
  
  service_call_succeeded = begin_client.call(begin_comp);  
  
  servise_geta_succeeded = begin_anclient.call(my_get);
  
  //ROS_INFO("response unit: %s", type[0].type.c_str());
  
  if(begin_comp.response.success)ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
  
  else{
    ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
    ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  }
  ros::Rate rate(10);
  while(ros::ok()){
    if(!int_vector.empty()){
      ROS_INFO("The first order type: %s",int_vector[0].shipments[0].products[0].type.c_str());
      my_get.request.material_type = int_vector[0].shipments[0].products[0].type;
      servise_geta_succeeded = begin_anclient.call(my_get);
      ROS_INFO("Storage Unit(bins) of the first product in the first shipment of the first order: %s", my_get.response.storage_units[0].unit_id.c_str());
      int_vector.clear();
    }
    
    if(flag != oldflag){
    
    //ROS_INFO("%s",image[flag-1].models[0].type.c_str());
    //ROS_INFO("flag ==%d",flag);
    my_get.request.material_type = image[flag-1].models[0].type;
    servise_geta_succeeded = begin_anclient.call(my_get);
    //ROS_INFO("Another Service called successfully: %s", my_get.response.storage_units[0].unit_id.c_str());
    part_pose.pose = image[flag-1].models[0].pose;
    
    
    try {
    tfStamped = tfBuffer.lookupTransform("arm1_base_link", "bin4_frame", ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
    tfStamped.child_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    } 
    
    part_pose.pose = goal_pose.pose;
    goal_pose.pose.position.z += 0.10;
    goal_pose.pose.orientation.w = 0.707;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.707;
    goal_pose.pose.orientation.z = 0.0;
    
    tf2::doTransform(part_pose, goal_pose, tfStamped);
    if(!warn_flag){
    ROS_WARN("logical_camera frame: x:%f,y:%f,z:%f", part_pose.pose.position.x,part_pose.pose.position.y,part_pose.pose.position.z);
    ROS_WARN("arm1_base_link frame: x:%f,y:%f,z:%f", goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z);
    warn_flag = true;
    }
    oldflag = flag;
    }  
    
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

