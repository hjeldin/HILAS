#pragma once

#include <rtt/RTT.hpp>

#include <motion_control_msgs/typekit/Types.h>
#include <ocl/Component.hpp>

#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

namespace Hilas
{

using namespace RTT;
using namespace std;

class IRobotCmdDemux: public RTT::TaskContext
{

public:

	IRobotCmdDemux(std::string const& name, std::string robot_name, const string* ARM_JOINT_NAME_ARRAY[], const string* BASE_JOINT_NAME_ARRAY[]);
 	~IRobotCmdDemux();

protected:

 	bool startHook();
 	bool configureHook();

 	virtual void updateHook() = 0;
	void stopHook();
	void cleanupHook();
	
	int arm_count;
	int base_count;

	vector<int> arm_joint_count;
	vector<char> base_joint_count;

	InputPort<motion_control_msgs::JointVelocities> joint_velocities_in;

	vector< OutputPort<motion_control_msgs::JointVelocities>* > arm_velocity_command_out;
 	vector< OutputPort<motion_control_msgs::JointVelocities>* > base_velocity_command_out;

 	vector<motion_control_msgs::JointVelocities> m_arm_velocity_cmd;
 	vector<motion_control_msgs::JointVelocities> m_base_velocity_cmd;

	motion_control_msgs::JointVelocities m_joint_velocity_cmd;
};

}