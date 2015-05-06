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

/** Generic robot state republisher. 
 * Keep in mind that you can't generate an orocos component out of this class since it's update hook is pure virtual and should be extended with your robot joints configuration.
 * Check HILAS_HOME/robots/youbot/youbot_republisher.cpp for an example.
 */
class IRobotStateRepublisher: public RTT::TaskContext
{

public:

	//* Default constructor for 
	IRobotStateRepublisher(std::string const& name, std::string robot_name, int SIZE_JOINT_NAME_ARRAY, const std::string* JOINT_NAME_ARRAY);
 	~IRobotStateRepublisher();

protected:

 	bool startHook();
 	bool configureHook();

 	/**
 	 * Overridable update hook for specification in your robot state publisher.
 	 * Necessary since URDF may specify useless joints (such as youbot's caster joint) which you don't want to pass to high level, and to publish ROS messages regarding robot state.
 	 */

 	virtual void updateHook() = 0;
	void stopHook();
	void cleanupHook();
	
	int arm_count;															///< arm count obtained from constructor 
	int base_count;															///< base count obtained from constructor

	vector< InputPort<sensor_msgs::JointState>* > arm_state_in;				///< orocos input port vector [arm1_joint_state, arm2_joint_state] expressed in joint position
 	vector< InputPort<sensor_msgs::JointState>* > base_state_in;			///< orocos input port vector [base1_state, base2_state] expressed in joint position
 	InputPort<nav_msgs::Odometry> odometry_state_in;						///< orocos input port for odometry

 	sensor_msgs::JointState m_arm_state;									///< temporary message for single arm 
 	sensor_msgs::JointState m_base_state;									///< temporary message for single base

 	std::string m_events;													///< unknown variable for unknown task

 	OutputPort<sensor_msgs::JointState> robot_state_out;					///< output orocos port for joint and base stats.isDirectory
 	OutputPort<nav_msgs::Odometry> odometry_state_out;						///< output orocos port for odometry
 	OutputPort<std::string> events_out;										///< unknown variable for unknown task

 	nav_msgs::Odometry m_odometry_state;									///< output message for odometry
 	sensor_msgs::JointState m_robot_state;									///< output message for robot state (joint and base)
};

}