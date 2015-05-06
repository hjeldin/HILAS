#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <std_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

#include <cassert>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>

#include <hilas.hpp>
#include <IRobot.hpp>

namespace Hilas
{

using namespace RTT;
using namespace RTT::types;
using namespace std;
using namespace boost;
using namespace KDL;

class IRobotBaseService: public Service
{
public:

	IRobotBaseService(const string& name, TaskContext* parent, int base_slave_count, int robot_slave_count, const std::string base_slave_name[], long m_clientID);
	virtual ~IRobotBaseService();

	virtual void setControlModesAll(int mode);
	void setControlModes(vector<ctrl_modes>& all);
	void getControlModes(vector<ctrl_modes>& all);
	virtual void displayMotorStatuses();
	virtual void clearControllerTimeouts();
    void setsim_mode(int mode);

protected:

	InputPort<motion_control_msgs::JointVelocities> joint_velocity_command;
	InputPort<motion_control_msgs::JointPositions> joint_position_command;
	InputPort<motion_control_msgs::JointEfforts> joint_effort_command;
	InputPort<geometry_msgs::Twist> cmd_twist;

    InputPort<sensor_msgs::JointState> in_joint_state;
    InputPort<nav_msgs::Odometry> in_odometry_state;

	OutputPort<sensor_msgs::JointState> joint_state;
	OutputPort<nav_msgs::Odometry> odometry_state;
	OutputPort<std::string> events;

	void setupComponentInterface();

	bool start();
	void update();
	virtual void cleanup();
	virtual void stop();
	virtual bool calibrate();

	virtual void readJointStates() = 0;
	virtual void readOdometry() = 0;
	virtual void setJointSetpoints() = 0;
	virtual void setTwistSetpoints() = 0;
	virtual void checkMotorStatuses();

	motion_control_msgs::JointVelocities m_joint_velocity_command;
	motion_control_msgs::JointPositions m_joint_position_command;
	motion_control_msgs::JointEfforts m_joint_effort_command;
	geometry_msgs::Twist m_cmd_twist;
	sensor_msgs::JointState m_joint_state;
	nav_msgs::Odometry m_odometry_state;
	std::string m_events;

	vector<ctrl_modes> m_joint_ctrl_modes;

	int NR_OF_BASE_SLAVES;
    long clientID;
    bool is_in_visualization_mode;	
	bool m_calibrated;
};

}