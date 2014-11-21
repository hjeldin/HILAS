#pragma once

#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <std_msgs/typekit/Types.h>
#include <ocl/Component.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

namespace Hilas
{

using namespace RTT;
using namespace std;

class IRobotStateRepublisher: public RTT::TaskContext
{

public:

	IRobotStateRepublisher(std::string const& name, std::string robot_name, int SIZE_JOINT_NAME_ARRAY, const std::string* JOINT_NAME_ARRAY);
 	~IRobotStateRepublisher();

protected:

 	bool startHook();
 	bool configureHook();
 	virtual void updateHook() = 0;
	void stopHook();
	void cleanupHook();
	
	int arm_count;
	int base_count;

	vector< InputPort<sensor_msgs::JointState>* > arm_state_in;
 	vector< InputPort<sensor_msgs::JointState>* > base_state_in;
 	InputPort<nav_msgs::Odometry> odometry_state_in;

 	sensor_msgs::JointState m_arm_state;
 	sensor_msgs::JointState m_base_state;

 	std::string m_events;

 	OutputPort<sensor_msgs::JointState> robot_state_out;
 	OutputPort<nav_msgs::Odometry> odometry_state_out;
 	OutputPort<std::string> events_out;

 	nav_msgs::Odometry m_odometry_state;
 	sensor_msgs::JointState m_robot_state;
};

}