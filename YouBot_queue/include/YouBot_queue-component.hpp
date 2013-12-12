#ifndef OROCOS_YOUBOT_QUEUE_COMPONENT_HPP
#define OROCOS_YOUBOT_QUEUE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <deque>

#include "ros/ros.h"
#include "ros/time.h"
#include <motion_control_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

using namespace RTT;
using namespace std;

struct queue_item
{
	motion_control_msgs::JointPositions arm_pos;
	motion_control_msgs::JointVelocities arm_vel;
	motion_control_msgs::JointEfforts arm_eff;

	motion_control_msgs::JointPositions gripper_pos;	
	geometry_msgs::Twist base_twist;

	int mode_index;
	uint64_t timestamp;

	queue_item(
		motion_control_msgs::JointPositions ap ,motion_control_msgs::JointVelocities av,
		motion_control_msgs::JointEfforts ae, motion_control_msgs::JointPositions gp,
		geometry_msgs::Twist bt, int m, uint64_t t): arm_pos(ap), arm_vel(av), arm_eff(ae), gripper_pos(gp), base_twist(bt), mode_index(m), timestamp(t){}

	queue_item(motion_control_msgs::JointPositions ap, uint64_t t): arm_pos(ap), mode_index(0), timestamp(t){}
	queue_item(motion_control_msgs::JointVelocities av, uint64_t t): arm_vel(av), mode_index(1), timestamp(t){}
	queue_item(motion_control_msgs::JointEfforts ae, uint64_t t): arm_eff(ae), mode_index(2), timestamp(t){}	
	queue_item(motion_control_msgs::JointPositions gp, int m, uint64_t t): gripper_pos(gp), mode_index(m), timestamp(t){}
	queue_item(geometry_msgs::Twist bt, uint64_t t): base_twist(bt), mode_index(4), timestamp(t){}
};

class YouBot_queue : public RTT::TaskContext{
  public:
    YouBot_queue(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setQueueMode(bool mode);
  
  protected:
    InputPort<motion_control_msgs::JointPositions> ros_arm_joint_position_command;
    InputPort<motion_control_msgs::JointVelocities> ros_arm_joint_velocity_command;
    InputPort<motion_control_msgs::JointEfforts> ros_arm_joint_effort_command;  	

    InputPort<geometry_msgs::Twist> ros_base_cmd_twist;

    InputPort<motion_control_msgs::JointPositions> ros_gripper_joint_position_command;

    InputPort<motion_control_msgs::JointPositions> orocos_arm_joint_position_command;
    InputPort<motion_control_msgs::JointVelocities> orocos_arm_joint_velocity_command;
    InputPort<motion_control_msgs::JointEfforts> orocos_arm_joint_effort_command;  	

    InputPort<geometry_msgs::Twist> orocos_base_cmd_twist;

    InputPort<motion_control_msgs::JointPositions> orocos_gripper_joint_position_command;

    OutputPort<motion_control_msgs::JointPositions> out_arm_joint_position_command;
    OutputPort<motion_control_msgs::JointVelocities> out_arm_joint_velocity_command;
    OutputPort<motion_control_msgs::JointEfforts> out_arm_joint_effort_command;  	

    OutputPort<geometry_msgs::Twist> out_base_cmd_twist;

    OutputPort<motion_control_msgs::JointPositions> out_gripper_joint_position_command;

	motion_control_msgs::JointPositions ros_arm_joint_position_command_data;
	motion_control_msgs::JointVelocities ros_arm_joint_velocity_command_data;
	motion_control_msgs::JointEfforts ros_arm_joint_effort_command_data;
	geometry_msgs::Twist ros_base_cmd_twist_data;
	motion_control_msgs::JointPositions ros_gripper_joint_position_command_data;

	motion_control_msgs::JointPositions orocos_arm_joint_position_command_data;
	motion_control_msgs::JointVelocities orocos_arm_joint_velocity_command_data;
	motion_control_msgs::JointEfforts orocos_arm_joint_effort_command_data;
	geometry_msgs::Twist orocos_base_cmd_twist_data;
	motion_control_msgs::JointPositions orocos_gripper_joint_position_command_data;

  private:
  	bool isinloading;
  	std::deque<queue_item> queue;
};
#endif
