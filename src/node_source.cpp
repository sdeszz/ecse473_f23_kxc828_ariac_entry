#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Proximity.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "ik_service/PoseIK.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"
#include <sstream>

trajectory_msgs::JointTrajectory move, traj;
std::vector<sensor_msgs::JointState> join;
geometry_msgs::PoseStamped part_pose, goal_pose;
osrf_gear::GetMaterialLocations my_get;
std::vector<osrf_gear::Model> type;
std::vector<osrf_gear::LogicalCameraImage> tt;
std::vector<osrf_gear::Order> int_vector;
osrf_gear::LogicalCameraImage image;
osrf_gear::LogicalCameraImage qmage;
osrf_gear::LogicalCameraImage rmage;
osrf_gear::LogicalCameraImage smage;
osrf_gear::LogicalCameraImage tmage;
osrf_gear::LogicalCameraImage umage;
tf2_ros::Buffer tfBuffer;
//std::vector<osrf_gear::> servise_geta_succeeded;

geometry_msgs::TransformStamped tfStamped;


int flag = 0;
int oldflag = 0;
int flaga = 0;
int soluN = 0;
int count = 0;
int service_call_succeeded,servise_geta_succeeded;
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];

void chatterCallback(const osrf_gear::Order::ConstPtr& msg){
  //int_vector.clear();
  int_vector.push_back(*msg);
  ROS_INFO("Order type: %s",int_vector[0].shipments[0].products[0].type.c_str());    
  
}

void cameraaCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  //image.clear();
  //ROS_INFO("Callbcak!");
  //ROS_INFO("%s",msg->models[0].type.c_str());
  //type = *msg->models;
  //tt = *msg;
  flag += 1;
  image = *msg;
  
}

void camerabCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
 
  qmage = *msg;
  
}

void cameracCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  
  rmage = *msg;
  
}

void cameradCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  
  smage = *msg;
  
}

void cameraeCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  
  tmage = *msg;
  
}

void camerafCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  
  umage = *msg;
  
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg){
  flaga += 1;
  join.push_back(*msg);
  
}

void infoCallback(const ros::TimerEvent& event) {
    if(!join.empty()){
      
      ROS_INFO("The Joint State:%f",join[flaga-1].position[0]);

    }
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
  ik_service::PoseIK ik_pose;
  geometry_msgs::Pose anotherpart_pose;
  ros::Timer timer = n.createTimer(ros::Duration(10.0), infoCallback);
  ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, chatterCallback);
  ros::Subscriber asub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1", 1000, cameraaCallback);
  ros::Subscriber bsub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2", 1000, camerabCallback);
  ros::Subscriber csub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3", 1000, cameracCallback);
  ros::Subscriber esub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4", 1000, cameradCallback);
  ros::Subscriber fsub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5", 1000, cameraeCallback);
  ros::Subscriber gsub = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 1000, camerafCallback);
  ros::Subscriber dsub = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1000, jointCallback);
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command",10);
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");  
  ros::ServiceClient begin_anclient = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations"); 
  ros::ServiceClient ik_client = n.serviceClient<ik_service::PoseIK>("/pose_ik");
  bool service_available = ros::service::waitForService("/pose_ik",ros::Duration(10.0));
  ROS_INFO("service_available ? %d", service_available);
  
  std::vector<osrf_gear::Model> mod;
  
  
  service_call_succeeded = begin_client.call(begin_comp);  
  
  servise_geta_succeeded = begin_anclient.call(my_get);
  
  //ROS_INFO("response unit: %s", type[0].type.c_str());
  
  if(begin_comp.response.success)ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
  
  else{
    ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
    ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  }
  ros::Rate rate(10);
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  
  while(ros::ok()){
    if(!int_vector.empty()){
      ROS_INFO("The first order type: %s",int_vector[0].shipments[0].products[0].type.c_str());
      my_get.request.material_type = int_vector[0].shipments[0].products[0].type;
      servise_geta_succeeded = begin_anclient.call(my_get);
      ROS_INFO("Storage Unit(bins) of the first product in the first shipment of the first order: %s", my_get.response.storage_units[0].unit_id.c_str());
      int_vector.clear();
    }
    //my_get.response.storage_units[0].unit_id
    //my_get.request.material_type
    
    if(flag != oldflag){
      if(my_get.response.storage_units[0].unit_id == "bin1"){
        mod = image.models;
      }
      else if(my_get.response.storage_units[0].unit_id == "bin2"){
        mod = qmage.models;
      }
      else if(my_get.response.storage_units[0].unit_id == "bin3"){
        mod = rmage.models;
      }
      else if(my_get.response.storage_units[0].unit_id == "bin4"){
        mod = smage.models;
      }
      else if(my_get.response.storage_units[0].unit_id == "bin5"){
       mod = tmage.models;
      }
      else if(my_get.response.storage_units[0].unit_id == "bin6"){
       mod = umage.models;
      }
      for(osrf_gear::Model m : mod){
        if(m.type == my_get.request.material_type){
          part_pose.pose = m.pose;
        }
      }
    
    
    //ROS_INFO("Another Service called successfully: %s", my_get.response.storage_units[0].unit_id.c_str());
    //part_pose.pose = image[flag-1].models[0].pose;
    
    
    try {
    tfStamped = tfBuffer.lookupTransform("arm1_base_link", "bin4_frame", ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
    tfStamped.child_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    } 
    
    q_pose[0] = join[flaga-1].position[1];
    q_pose[1] = join[flaga-1].position[2];
    q_pose[2] = join[flaga-1].position[3];
    q_pose[3] = join[flaga-1].position[4];
    q_pose[4] = join[flaga-1].position[5];
    q_pose[5] = join[flaga-1].position[6];
    
    ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
    
    tf2::doTransform(part_pose, goal_pose, tfStamped);
    //goal_pose.pose = part_pose.pose;
    goal_pose.pose.position.z += 0.10;
    //goal_pose.pose.orientation.w = 0.707;
    //goal_pose.pose.orientation.x = 0.0;
    //goal_pose.pose.orientation.y = 0.707;
    //goal_pose.pose.orientation.z = 0.0;
    
    T_des[0][3] = goal_pose.pose.position.x;
    T_des[1][3] = goal_pose.pose.position.y;
    T_des[2][3] = goal_pose.pose.position.z; // above part
    T_des[3][3] = 1.0;
    // The orientation of the end effector so that the end effector is down.
    T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
    T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
    T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
    T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
    
    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
    anotherpart_pose.position.x = goal_pose.pose.position.x;
    anotherpart_pose.position.y = goal_pose.pose.position.y;
    anotherpart_pose.position.z = goal_pose.pose.position.z;
    ROS_INFO("state velocity x:%f state velocity y:%f state velocity z:%f",join[flaga-1].velocity[6],join[flaga-1].velocity[6],join[flaga-1].velocity[6]);
    //anotherpart_pose.position.x = 0.5;
    anotherpart_pose.orientation.w = 0.707;
    anotherpart_pose.orientation.x = 0.0;
    anotherpart_pose.orientation.y = 0.707;
    anotherpart_pose.orientation.z = 1;
    ik_pose.request.part_pose = anotherpart_pose;
    if (ik_client.call(ik_pose)){
    //ROS_INFO("Call to ik_service returned [%i] solutions", ik_pose.response.num_sols);
      //ROS_INFO("!!:%d!!%d",ik_pose.response.num_sols,num_sols);
      if(ik_pose.response.num_sols > 0 && ik_pose.response.num_sols <= 8){
      //ROS_INFO("Ik_service solution lisr: angle1: %f,angle2: %f,angle3: %f,angle4: %f,angle5: %f,angle6: %f", ik_pose.response.joint_solutions[0].joint_angles[0],ik_pose.response.joint_solutions[0].joint_angles[1],ik_pose.response.joint_solutions[0].joint_angles[2],ik_pose.response.joint_solutions[0].joint_angles[3],ik_pose.response.joint_solutions[0].joint_angles[4],ik_pose.response.joint_solutions[0].joint_angles[5]);
        for(int i = 0; i<8; i++){
          if((ik_pose.response.joint_solutions[i].joint_angles[4]<4.715)&&(ik_pose.response.joint_solutions[i].joint_angles[4]>4.695)){
            soluN = i;
            ROS_INFO("Ik_service solution lisr: angle1: %f,angle2: %f,angle3: %f,angle4: %f,angle5: %f,angle6: %f", ik_pose.response.joint_solutions[i].joint_angles[0],ik_pose.response.joint_solutions[i].joint_angles[1],ik_pose.response.joint_solutions[i].joint_angles[2],ik_pose.response.joint_solutions[i].joint_angles[3],ik_pose.response.joint_solutions[i].joint_angles[4],ik_pose.response.joint_solutions[i].joint_angles[5]);
          
          break;
          }
        }
      }
    }
    
    
    traj.header.seq = count++;
    traj.header.stamp = ros::Time::now(); 
    traj.header.frame_id = "/world";
    traj.joint_names.clear();
    traj.joint_names.push_back("linear_arm_actuator_joint");
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");
    traj.points.resize(2);
    
    traj.points[0].positions.resize(traj.joint_names.size());
    for (int indy = 0; indy < traj.joint_names.size(); indy++) {
      for (int indz = 0; indz < join[flaga-1].name.size(); indz++) {
        if (traj.joint_names[indy] == join[flaga-1].name[indz]) {
          traj.points[0].positions[indy] = join[flaga-1].position[indz];
          break;
          }
        }
    }
    traj.points[0].time_from_start = ros::Duration(0.0); 
    int sol_indx = 3;
    traj.points[1].positions.resize(traj.joint_names.size());
    
    traj.points[1].positions[0] = join[flaga-1].position[1];
    
    for (int indy = 0; indy < 6; indy++) {
      //traj.points[1].positions[indy + 1] = q_des[soluN][indy];
      traj.points[1].positions[indy + 1] = ik_pose.response.joint_solutions[soluN].joint_angles[indy];
    }
    
    traj.points[1].time_from_start = ros::Duration(1.0);
    
    pub.publish(traj);
    
    if(!warn_flag){
    ROS_WARN("logical_camera frame: x:%f,y:%f,z:%f", part_pose.pose.position.x,part_pose.pose.position.y,part_pose.pose.position.z);
    ROS_WARN("arm1_base_link frame: x:%f,y:%f,z:%f", goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z);
    warn_flag = true;
    }
    oldflag = flag;
    }  
    
    //ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

