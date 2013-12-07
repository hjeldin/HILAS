#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "v_repConst.h"

#include <sensor_msgs/JointState.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <motion_control_msgs/JointPositions.h>
#include <motion_control_msgs/JointVelocities.h>
#include <motion_control_msgs/JointEfforts.h>
#include "tf/transform_broadcaster.h"
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include "youbot/YouBotBase.hpp"
#include "generic/ConfigFile.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp"

// Used data structures:
//#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"

// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosSetJointTargetPosition.h"
#include "vrep_common/simRosSetJointTargetVelocity.h"
#include "vrep_common/simRosSetObjectIntParameter.h"
#include "vrep_common/simRosGetObjectPose.h"

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"


// General define:
#define NUM_ARGS 13
#define NUM_ARM_JOINTS 7
#define NUM_BASE_JOINTS 4 // Rolling + caster joints ???
#define NUM_JOINTS 16

// ROS define:
#define TOPIC_ARM_JOINT_POSITION_COMMAND  "/arm_1/arm_controller/position_command"
#define TOPIC_GRIPPER_POSITION_COMMAND   "/arm_1/gripper_controller/position_command"
#define TOPIC_VELOCITY_COMMAND  "/arm_1/arm_controller/velocity_command"
#define TOPIC_TORQUE_COMMAND    "/arm_1/arm_controller/force_command"
#define TOPIC_TWIST_COMMAND   "/cmd_vel"
#define TOPIC_TWIST_READ  "/vrep/twist"
#define TF_ODOM_FRAME_ID  "odom"
#define TF_ODOM_CHILD_FRAME_ID "base_footprint"

#define TOPIC_LASERSCAN_READ "/vrep/RangeFinderData"
/* NUMERI A CASO */
#define output_frame_id_ 1
#define angle_min_ 0
#define angle_max_ M_PI
#define angle_increment_ 0.03
#define scan_time_ 0.01
#define range_min_ 0.05
#define range_max_ 2
#define max_height_ 0.5 
#define min_height_ 0.25
#define range_min_sq_ 1
/* FINE NUMERI A CASO */
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


// Configure path youbot-driver
#define CFG_YOUBOT_BASE "/home/altair/youbot_driver/config/"

// V-REP define:
// - IP -> IntParameter (service Set/GetObjectIntParameter)
// - FP -> FloatParameter (serivce Set/GetObjectFloatParameter)

#define VREP_JOINT_CONTROL_POSITION_IP 2001

// YouBot driver -> base (for kinematic)
//youbot::YouBotBase* youBotBase;
boost::scoped_ptr<youbot::ConfigFile> configfile;
youbot::FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;
youbot::FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
float simulationTime=0.0f;
int all_joints_handle;
int arm_joint_handles[NUM_ARM_JOINTS];
int base_joint_handles[NUM_BASE_JOINTS];
int youbot_handle;

//Ros Publisher for JointStates remapped
ros::Publisher pubJointStates;
ros::Publisher pubOdom;

ros::Publisher laserScan;

ros::ServiceClient client_cmdPos;
ros::ServiceClient client_cmdVel;
ros::ServiceClient client_jointMode;
ros::ServiceClient client_readObjectPose;

// Global service objects
vrep_common::simRosSetJointTargetPosition srv_SetJointTargetPosition;
vrep_common::simRosSetJointTargetVelocity srv_SetJointTargetVelocity;
vrep_common::simRosSetObjectIntParameter srv_SetObjectIntParameter;
vrep_common::simRosGetObjectPose srv_GetObjectPose;


//Odom state 
nav_msgs::Odometry odometry;
geometry_msgs::TransformStamped odometryTransform;

sensor_msgs::LaserScan output;

// Command arm_joint position callback
void cmdPosArmJointCallback(const motion_control_msgs::JointPositions::ConstPtr& msg) 
{
  int index;
  for(unsigned int i=0; i < msg->positions.size() ; i++)
  {
    index = msg->names.at(i).find_last_of('_');
    index = atoi(msg->names.at(i).substr(index+1).c_str()) - 1;

    /** Switch to Control Position **/

    srv_SetObjectIntParameter.request.handle = arm_joint_handles[index];
    srv_SetObjectIntParameter.request.parameter = VREP_JOINT_CONTROL_POSITION_IP;
    srv_SetObjectIntParameter.request.parameterValue = 1;
    client_jointMode.call(srv_SetObjectIntParameter);

    /** Send position value **/

    srv_SetJointTargetPosition.request.handle = arm_joint_handles[index];
    srv_SetJointTargetPosition.request.targetPosition = msg->positions[i];
    client_cmdPos.call(srv_SetJointTargetPosition);  
  }
}

// Command gripper position callback
void cmdPosGripperCallback(const motion_control_msgs::JointPositions::ConstPtr& msg) 
{
  for(unsigned int i=0; i < msg->positions.size(); i++)
  {
    if(msg->names.at(i) == "gripper_finger_joint_l")
    {
      srv_SetJointTargetPosition.request.handle = arm_joint_handles[5];
      srv_SetJointTargetPosition.request.targetPosition = msg->positions[i];
    }

    if(msg->names.at(i) == "gripper_finger_joint_r")
    {
      srv_SetJointTargetPosition.request.handle = arm_joint_handles[6];
      srv_SetJointTargetPosition.request.targetPosition = msg->positions[i];
    }
    client_cmdPos.call(srv_SetJointTargetPosition);  
  }
}

// Command velocity callback
void cmdVelCallback(const motion_control_msgs::JointVelocities::ConstPtr& msg) 
{
  int index;
  for(unsigned int i=0; i < msg->names.size() ; i++)
  {
    index = msg->names.at(i).find_last_of('_');
    index = atoi(msg->names.at(i).substr(index+1).c_str()) - 1;
    
    /** Switch to Control Position **/

    srv_SetObjectIntParameter.request.handle = arm_joint_handles[index];
    srv_SetObjectIntParameter.request.parameter = VREP_JOINT_CONTROL_POSITION_IP;
    srv_SetObjectIntParameter.request.parameterValue = 0;
    client_jointMode.call(srv_SetObjectIntParameter);

    /** Send velocity value **/

    srv_SetJointTargetVelocity.request.handle = arm_joint_handles[index];
    srv_SetJointTargetVelocity.request.targetVelocity = msg->velocities[i];
    client_cmdVel.call(srv_SetJointTargetVelocity);  
  }
}

// Command torque callback
void cmdForceCallback(const motion_control_msgs::JointEfforts::ConstPtr& msg) 
{
  /*
     send torques to VREP
  */
}

// Command twist callback
void cmdTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  std::vector<quantity<angular_velocity> > wheelVelocities;

  quantity<si::velocity> longitudinalVelocity;
  quantity<si::velocity> transversalVelocity;
  quantity<si::angular_velocity> angularVelocity;

  longitudinalVelocity = msg->linear.x * meter_per_second;
  transversalVelocity = msg->linear.y * meter_per_second;
  angularVelocity = msg->angular.z * radian_per_second;

  youBotBaseKinematic.cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, wheelVelocities);

  //std::cout << msg->linear.x << " " << msg->linear.y << " " << msg->angular.z << std::endl;

  // Little hack
  int sign[4] = {1,-1,1,-1};

  for(unsigned int i=0; i < wheelVelocities.size() ; i++)
  {
     srv_SetJointTargetVelocity.request.handle = base_joint_handles[i];
     srv_SetJointTargetVelocity.request.targetVelocity = wheelVelocities[i].value() * sign[i];
     //std::cout << "wheelVelocities[" << i << "]" << " =" << srv_SetJointTargetVelocity.request.targetVelocity << std::endl;
     client_cmdVel.call(srv_SetJointTargetVelocity);
  }  
}

void readTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  // TF Broadcaster
  static tf::TransformBroadcaster br;

  srv_GetObjectPose.request.handle = youbot_handle;
  srv_GetObjectPose.request.relativeToObjectHandle = -1;  //world reference
  client_readObjectPose.call(srv_GetObjectPose);
  if(srv_GetObjectPose.response.result)
  {
    odometry.header.frame_id = "/odom";
    odometry.child_frame_id = "/base_link";
    odometry.pose.pose.position.x = srv_GetObjectPose.response.pose.pose.position.x;
    odometry.pose.pose.position.y = srv_GetObjectPose.response.pose.pose.position.y;
    odometry.pose.pose.position.z = srv_GetObjectPose.response.pose.pose.position.z;
    odometry.pose.pose.orientation.x = srv_GetObjectPose.response.pose.pose.orientation.x;
    odometry.pose.pose.orientation.y = srv_GetObjectPose.response.pose.pose.orientation.y;
    odometry.pose.pose.orientation.z = srv_GetObjectPose.response.pose.pose.orientation.z;
    odometry.pose.pose.orientation.w = srv_GetObjectPose.response.pose.pose.orientation.w;



    /** TEST **/
    // odometry.pose.pose.position.x = srv_GetObjectPose.response.pose.pose.position.y;
    // odometry.pose.pose.position.y = -srv_GetObjectPose.response.pose.pose.position.x;
    // odometry.pose.pose.position.z = srv_GetObjectPose.response.pose.pose.position.z;
    // odometry.pose.pose.orientation.x = srv_GetObjectPose.response.pose.pose.orientation.x;
    // odometry.pose.pose.orientation.y = srv_GetObjectPose.response.pose.pose.orientation.y;
    // odometry.pose.pose.orientation.z = srv_GetObjectPose.response.pose.pose.orientation.z;
    // odometry.pose.pose.orientation.w = srv_GetObjectPose.response.pose.pose.orientation.w;

    KDL::Rotation orient =  KDL::Rotation::Quaternion(srv_GetObjectPose.response.pose.pose.orientation.x, 
                                                    srv_GetObjectPose.response.pose.pose.orientation.y,
                                                    srv_GetObjectPose.response.pose.pose.orientation.z,
                                                    srv_GetObjectPose.response.pose.pose.orientation.w);

    KDL::Rotation rotation = KDL::Rotation::RPY(0,0,M_PI/2);

    orient = orient * rotation;

    double x,y,z,w;
    double roll, pitch, yaw;
    orient.GetQuaternion(x, y, z, w);
    orient.GetRPY(roll, pitch, yaw);
    //std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl; 
    odometry.pose.pose.orientation.x = x;
    odometry.pose.pose.orientation.y = y;
    odometry.pose.pose.orientation.z = z;
    odometry.pose.pose.orientation.w = w;
    // odometry.twist.twist.linear.x = msg->twist.linear.y;
    // odometry.twist.twist.linear.y = -msg->twist.linear.x;
    // odometry.twist.twist.linear.z = msg->twist.linear.z;
    // odometry.twist.twist.angular.x = msg->twist.angular.x;
    // odometry.twist.twist.angular.y = msg->twist.angular.y;
    // odometry.twist.twist.angular.z = msg->twist.angular.z;
    /** **/

    /* Read Twist message from V-REP (only linear/angular velocities) */
    odometry.twist.twist.linear.x = msg->twist.linear.x;
    odometry.twist.twist.linear.y = msg->twist.linear.y;
    odometry.twist.twist.linear.z = msg->twist.linear.z;
    odometry.twist.twist.angular.x = msg->twist.angular.x;
    odometry.twist.twist.angular.y = msg->twist.angular.y;
    odometry.twist.twist.angular.z = msg->twist.angular.z;

    odometryTransform.header.stamp = ros::Time::now();
    odometryTransform.header.frame_id = TF_ODOM_FRAME_ID;
    odometryTransform.child_frame_id = TF_ODOM_CHILD_FRAME_ID;
    odometryTransform.transform.translation.x = odometry.pose.pose.position.x;
    odometryTransform.transform.translation.y = odometry.pose.pose.position.y;
    odometryTransform.transform.translation.z = odometry.pose.pose.position.z;
    odometryTransform.transform.rotation = odometry.pose.pose.orientation;

    pubOdom.publish(odometry);
    br.sendTransform(odometryTransform);   
  }
}

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
  simulationTime=info->simulationTime.data;
  simulationRunning=(info->simulatorState.data&1)!=0;
}

/*void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  sensor_msgs::JointState msg_map;
  msg_map.name.resize(16);
  msg_map.position.resize(16);
  msg_map.velocity.resize(16);
  msg_map.effort.resize(16);
  std::string jn;
  for(int i=0; i < NUM_JOINTS; i++) 
  { 
    switch(i)
    { 
      case 0:
      jn = "wheel_joint_br";
      break;
      case 1:
      jn = "caster_joint_br";
      break;
      case 2:
      jn = "wheel_joint_bl";
      break;
      case 3:
      jn = "caster_joint_bl";
      break;
      case 4:
      jn = "suspension_joint";
      break;
      case 5: 
      jn = "wheel_joint_fl";
      break;
      case 6:
      jn = "caster_joint_fl";
      break;
      case 7:
      jn = "wheel_joint_fr";
      break;
      case 8:
      jn = "caster_joint_fr";
      break;
      case 9:
      jn = "arm_joint_1";
      break;
      case 10:
      jn = "arm_joint_2";
      break;
      case 11:
      jn = "arm_joint_3";
      break;
      case 12:
      jn = "arm_joint_4";
      break;
      case 13:
      jn = "arm_joint_5";
      break;
      case 14:
      jn = "gripper_finger_joint_l";
      break;
      case 15:
      jn = "gripper_finger_joint_r";
      break;
    }
    msg_map.header.stamp = ros::Time::now();
    msg_map.name.at(i) = jn; 
    msg_map.position.at(i) = msg->position.at(i);
    msg_map.velocity.at(i) = msg->velocity.at(i);
    msg_map.effort.at(i) = msg->effort.at(i);
  }
  pubJointStates.publish(msg_map);
}*/

void pointCloud2LaserScanCallback(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
    //sensor_msgs::LaserScan output(new sensor_msgs::LaserScan());
    output.header = cloud->header;
    output.header.frame_id = output_frame_id_; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding mount frame
    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(ranges_size, output.range_max + 1.0);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      const float &x = cloud->points[i].x;
      const float &y = cloud->points[i].y;
      const float &z = cloud->points[i].z;

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
        //NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        continue;
      }

      if (-y > max_height_ || -y < min_height_)
      {
        //NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", x, min_height_, max_height_);
        continue;
      }

      double range_sq = z*z+x*x;
      if (range_sq < range_min_sq_) {
        //NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
        continue;
      }

      double angle = -atan2(x, z);
      if (angle < output.angle_min || angle > output.angle_max)
      {
        //NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
        continue;
      }
      int index = (angle - output.angle_min) / output.angle_increment;


      if (output.ranges[index] * output.ranges[index] > range_sq)
        output.ranges[index] = sqrt(range_sq);
      }

    laserScan.publish(output);
}


int main(int argc,char* argv[])
{

  if(argc>= NUM_ARGS)
  {
    all_joints_handle = atoi(argv[1]);
    for(int i=0; i < NUM_BASE_JOINTS; i++)
      base_joint_handles[i] = atoi(argv[2+i]);

    for(int i=0; i < NUM_ARM_JOINTS; i++)
      arm_joint_handles[i] = atoi(argv[2 + NUM_BASE_JOINTS + i]);

    youbot_handle = atoi(argv[2 + NUM_BASE_JOINTS + NUM_ARM_JOINTS]);

    std::cout << "Youbot Handle " << youbot_handle << std::endl;
  }
  else
  {
    printf("fail...");
    return(0);
  }

  // Create a ROS node. The name has a random component: 
  int _argc = 0;
  char** _argv = NULL;
  struct timeval tv;
  unsigned int timeVal=0;
  if (gettimeofday(&tv,NULL)==0)
    timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
  std::string nodeName("youbot_vrep_sim");
  std::string randId(boost::lexical_cast<std::string>(timeVal+int(999999.0f*(rand()/(float)RAND_MAX))));
  nodeName+=randId;
  ros::init(_argc,_argv,nodeName.c_str());

  if(!ros::master::check())
    return(0);

  ros::NodeHandle node("~");
  printf("youbot_vrep_sim just started with node name %s\n",nodeName.c_str());

  //Youbot Base Kinematic
  //youBotBase = new youbot::YouBotBase("youbot-base", CFG_YOUBOT_BASE);
  configfile.reset(new youbot::ConfigFile("youbot-base.cfg",CFG_YOUBOT_BASE));
  //read the kinematics parameter from a config file
  configfile->readInto(kinematicConfig.rotationRatio, "YouBotKinematic", "RotationRatio");   
  configfile->readInto(kinematicConfig.slideRatio, "YouBotKinematic", "SlideRatio");
  double dummy = 0;
  configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontAndRearWheels_[meter]");
  kinematicConfig.lengthBetweenFrontAndRearWheels = dummy * meter;
  configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontWheels_[meter]");
  kinematicConfig.lengthBetweenFrontWheels = dummy * meter;
  configfile->readInto(dummy, "YouBotKinematic", "WheelRadius_[meter]");
  kinematicConfig.wheelRadius = dummy * meter;
  youBotBaseKinematic.setConfiguration(kinematicConfig);

  // Ros Subscriber VREP-INFO SIM
  ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);

  // Ros Subscriber Command Arm Joint Position 
  ros::Subscriber subCmdPos = node.subscribe<motion_control_msgs::JointPositions>(TOPIC_ARM_JOINT_POSITION_COMMAND, 1, cmdPosArmJointCallback);

  // Ros Subscriber Command Gripper Position
  ros::Subscriber subCmdGripperPos = node.subscribe<motion_control_msgs::JointPositions>(TOPIC_GRIPPER_POSITION_COMMAND, 1, cmdPosGripperCallback);

  // Ros Subscriber Command Arm Velocity
  ros::Subscriber subCmdVel = node.subscribe<motion_control_msgs::JointVelocities>(TOPIC_VELOCITY_COMMAND, 1, cmdVelCallback);

  // Ros Subscriber Command Arm Torque
  ros::Subscriber subCmdForce = node.subscribe<motion_control_msgs::JointEfforts>(TOPIC_TORQUE_COMMAND, 1, cmdForceCallback);

  // Ros Subscriber Command Base Twist
  ros::Subscriber subCmdTwist = node.subscribe<geometry_msgs::Twist>(TOPIC_TWIST_COMMAND, 1, cmdTwistCallback);

  // Ros Subscriber Twist (odom part)
  ros::Subscriber subReadTwist = node.subscribe<geometry_msgs::TwistStamped>(TOPIC_TWIST_READ, 1, readTwistCallback);

  // Ros Subscriber LaserScan (sensor)
  ros::Subscriber laserscanPointCloud = node.subscribe<sensor_msgs::PointCloud>(TOPIC_LASERSCAN_READ,1,pointCloud2LaserScanCallback);

  // Ros Publisher JointStates remapped
  pubJointStates=node.advertise<sensor_msgs::JointState>("/joint_states",10);

  // Ros Publisher Odom
  pubOdom = node.advertise<nav_msgs::Odometry>("/odom",10);

  laserScan = node.advertise<sensor_msgs::LaserScan>("/scan",10);

  client_cmdPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");

  client_cmdVel = node.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");

  client_jointMode = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");

  client_readObjectPose = node.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");

  /*ros::ServiceClient client_enablePublisher=node.serviceClient<vrep_common::simRosEnablePublisher>("/vrep/simRosEnablePublisher");
  vrep_common::simRosEnablePublisher srv_enablePublisher;
  srv_enablePublisher.request.topicName="joint_states"; // the requested topic name
  srv_enablePublisher.request.queueSize=1; // the requested publisher queue size (on V-REP side)
  srv_enablePublisher.request.streamCmd=simros_strmcmd_get_joint_state; // the requested publisher type
  srv_enablePublisher.request.auxInt1=all_joints_handle; // some additional information the publisher needs */

  while(ros::ok() && simulationRunning)
  {
      ros::spinOnce();
      usleep(5000);
  }        
}
