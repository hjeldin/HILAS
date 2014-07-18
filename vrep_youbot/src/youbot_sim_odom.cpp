#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/JointState.h>

#include "youbot/YouBotBase.hpp"
#include "generic/ConfigFile.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#define CFG_YOUBOT_BASE "/home/altair/youbot_driver/config/"

// YouBot driver -> base (for kinematic)
//youbot::YouBotBase* youBotBase;
boost::scoped_ptr<youbot::ConfigFile> configfile;
youbot::FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;
youbot::FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;

quantity<si::length> longitudinalPosition;
quantity<si::length> transversalPosition;
quantity<plane_angle> orientation;

quantity<si::velocity> longitudinalVelocity;
quantity<si::velocity> transversalVelocity;
quantity<si::angular_velocity> angularVelocity;

std::vector<double> jointsBasePosition;

std::vector<quantity<plane_angle> > wheelPositions;
std::vector<quantity<angular_velocity> > wheelVelocities;
quantity<plane_angle> dummy_pos;
quantity<angular_velocity> dummy_vel;

//Odom state 
nav_msgs::Odometry odometry;
geometry_msgs::TransformStamped odometryTransform;
std::vector<double> pos_prev;
ros::Publisher odomPub;

/**
---
header: 
  seq: 11171
  stamp: 
    secs: 1394373451
    nsecs: 253022855
  frame_id: ''
name: ['wheel_joint_br', 'wheel_joint_bl', 'wheel_joint_fl', 'wheel_joint_fr']
position: [-0.059269171208143234, -0.04573903977870941, -0.3583401143550873, -0.10407518595457077]
velocity: [-0.0006161630153656006, -0.0012658536434173584, -0.010037422180175781, -0.004175305366516113]
effort: [-0.012838631868362427, -0.008086711168289185, -0.06191897392272949, 0.06695833802223206]

wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
wheelJoints[1]=simGetObjectHandle('wheel_joint_fl')
wheelJoints[2]=simGetObjectHandle('wheel_joint_fr')
wheelJoints[3]=simGetObjectHandle('wheel_joint_bl')
wheelJoints[4]=simGetObjectHandle('wheel_joint_br')
**/

void baseJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;

	std::vector<double> deltapos;
	std::vector<double> joints;
    deltapos.assign(4,0);
    joints.assign(4,0);

	int sign[4]={1,-1,-1,1};
	for(int i=0; i < 4; i++)
		joints[i] = msg->position[i] * sign[i];

  double vels[4];
  vels[0] = -msg->velocity[1];
  vels[1] = msg->velocity[0];
  vels[2] = -msg->velocity[2];
  vels[3] = msg->velocity[3];

  double long_vel, trans_vel, ang_vel;

  double geom_factor = (kinematicConfig.lengthBetweenFrontAndRearWheels.value() / 2.0) + (kinematicConfig.lengthBetweenFrontWheels.value() / 2.0);

  long_vel = (-vels[0] + vels[1] - vels[2] + vels[3]) * (kinematicConfig.wheelRadius.value() / 4);
  trans_vel = (vels[0] + vels[1] - vels[2] - vels[3]) * (kinematicConfig.wheelRadius.value() / 4);
  ang_vel = (vels[0] + vels[1] + vels[2] + vels[3]) * ((kinematicConfig.wheelRadius.value() / 4) / geom_factor);

    /*deltapos[0] =  msg->position[1] - pos_prev[0];
    deltapos[1] =  msg->position[0] - pos_prev[1];
    deltapos[2] =  msg->position[2] - pos_prev[2];
    deltapos[3] =  msg->position[3] - pos_prev[3];*/

    deltapos[0] =  joints[1] - pos_prev[0];
    deltapos[1] =  joints[0] - pos_prev[1];
    deltapos[2] =  joints[2] - pos_prev[2];
    deltapos[3] =  joints[3] - pos_prev[3];

    for(int i=0; i < 4; ++i)
    {
      if(deltapos[i] > M_PI)
        deltapos[i] += -M_PI*2;
      else if(deltapos[i] < -M_PI)
        deltapos[i] += 2*M_PI;
    }

	// wheelPositions[0] = msg->position[2] * radian;
	// wheelPositions[1] = msg->position[3] * radian;
	// wheelPositions[2] = msg->position[1] * radian;
	// wheelPositions[3] = msg->position[0] * radian;

	for(int i=0; i < 4; i++)
		wheelPositions[i] += deltapos[i] * radian;

	pos_prev[0] = joints[1];
	pos_prev[1] = joints[0];
	pos_prev[2] = joints[2];
	pos_prev[3] = joints[3];

  youBotBaseKinematic.wheelPositionsToCartesianPosition(wheelPositions, longitudinalPosition, transversalPosition, orientation);
  


    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "/odom";
    odometry.child_frame_id = "/base_footprint";
    odometry.pose.pose.position.x = -longitudinalPosition.value();
    odometry.pose.pose.position.y = transversalPosition.value();
    odometry.pose.pose.position.z = 0.095;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(-orientation.value());
    odometry.twist.twist.linear.x = long_vel;
    odometry.twist.twist.linear.y = trans_vel;
    odometry.twist.twist.linear.z = 0;
    odometry.twist.twist.angular.x = 0;
    odometry.twist.twist.angular.y = 0;
    odometry.twist.twist.angular.z = -ang_vel;
    odomPub.publish(odometry);

    odometryTransform.header.stamp = odometry.header.stamp;
    odometryTransform.header.frame_id = "/odom";
    odometryTransform.child_frame_id = "/base_footprint";
    odometryTransform.transform.translation.x = -longitudinalPosition.value();
    odometryTransform.transform.translation.y = transversalPosition.value();
    odometryTransform.transform.translation.z = 0.095;
    odometryTransform.transform.rotation = tf::createQuaternionMsgFromYaw(-orientation.value());
 	br.sendTransform(odometryTransform);
}

int main(int argc,char* argv[])
{
  ros::init(argc,argv,"odom_sim");

  if(!ros::master::check())
    return(0);

  ros::NodeHandle node("~");

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

  wheelPositions.assign(4, dummy_pos);
  wheelVelocities.assign(4, dummy_vel);
  pos_prev.assign(4,0.0);

  ros::Subscriber subJointStates = node.subscribe<sensor_msgs::JointState>("/vrep/base/joint_states",
																			 10, baseJointStatesCallback);

  odomPub = node.advertise<nav_msgs::Odometry>("/odom", 1);
  ros::spin();
}
