#ifndef OROCOS_YOUBOT_QUEUE_COMPONENT_HPP
#define OROCOS_YOUBOT_QUEUE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <deque>
#include <cmath>

#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include "ros/ros.h"
#include "ros/time.h"
#include <motion_control_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

#define POS_DISTANCE 0.5
#define ORIENT_OFFSET 0.5

using namespace RTT;
using namespace std;

enum command_type{ARM_POS=0,ARM_VEL=1,ARM_EFF=2,GRIP_POS=3,TWIST=4,PLANNER=5,CARTESIAN=6};

struct queue_item
{
	motion_control_msgs::JointPositions arm_pos;
	motion_control_msgs::JointVelocities arm_vel;
	motion_control_msgs::JointEfforts arm_eff;

	motion_control_msgs::JointPositions gripper_pos;	
	geometry_msgs::Twist base_twist;
    geometry_msgs::PoseStamped planner;
    geometry_msgs::Pose cartesian;

	int mode_index;
	uint64_t timestamp;

    queue_item(){}
	queue_item(
		motion_control_msgs::JointPositions ap ,motion_control_msgs::JointVelocities av,
		motion_control_msgs::JointEfforts ae, motion_control_msgs::JointPositions gp,
		geometry_msgs::Twist bt, geometry_msgs::PoseStamped pl, geometry_msgs::Pose ct,
        int m, uint64_t t): arm_pos(ap), arm_vel(av), arm_eff(ae), gripper_pos(gp), base_twist(bt), planner(pl), cartesian(ct), mode_index(m), timestamp(t){}

	queue_item(motion_control_msgs::JointPositions ap, uint64_t t): arm_pos(ap), mode_index(static_cast<command_type>(ARM_POS)), timestamp(t){}
	queue_item(motion_control_msgs::JointVelocities av, uint64_t t): arm_vel(av), mode_index(static_cast<command_type>(ARM_VEL)), timestamp(t){}
	queue_item(motion_control_msgs::JointEfforts ae, uint64_t t): arm_eff(ae), mode_index(static_cast<command_type>(ARM_EFF)), timestamp(t){}	
	queue_item(motion_control_msgs::JointPositions gp, int m, uint64_t t): gripper_pos(gp), mode_index(static_cast<command_type>(GRIP_POS)), timestamp(t){}
	queue_item(geometry_msgs::Twist bt, uint64_t t): base_twist(bt), mode_index(static_cast<command_type>(TWIST)), timestamp(t){}
    queue_item(geometry_msgs::PoseStamped pl, uint64_t t): planner(pl), mode_index(static_cast<command_type>(PLANNER)), timestamp(t){}
    queue_item(geometry_msgs::Pose ct, uint64_t t): cartesian(ct), mode_index(static_cast<command_type>(CARTESIAN)), timestamp(t){}        
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

    /* Information from hw */
    //InputPort<nav_msgs::Odometry> from_hw_odom;
    //InputPort<sensor_msgs::JointState> from_hw_arm_joints;
    InputPort<geometry_msgs::Pose> from_cartesian_status;
    InputPort<bool> from_planner_goal;

    //nav_msgs::Odometry from_hw_odom_data;
    //sensor_msgs::JointState from_hw_arm_joints_data;
    geometry_msgs::Pose from_cartesian_status_data;
    bool from_planner_goal_data;

    /* Property */
    bool isEmpty;

    /* Command from ROS */
    InputPort<motion_control_msgs::JointPositions> ros_arm_joint_position_command;
    InputPort<motion_control_msgs::JointVelocities> ros_arm_joint_velocity_command;
    InputPort<motion_control_msgs::JointEfforts> ros_arm_joint_effort_command;
    InputPort<geometry_msgs::Twist> ros_base_cmd_twist;
    InputPort<motion_control_msgs::JointPositions> ros_gripper_joint_position_command;
    InputPort<geometry_msgs::PoseStamped> ros_planner_command;
    InputPort<geometry_msgs::Pose> ros_cartesian_command;

    motion_control_msgs::JointPositions ros_arm_joint_position_command_data;
    motion_control_msgs::JointVelocities ros_arm_joint_velocity_command_data;
    motion_control_msgs::JointEfforts ros_arm_joint_effort_command_data;
    geometry_msgs::Twist ros_base_cmd_twist_data;
    motion_control_msgs::JointPositions ros_gripper_joint_position_command_data;
    geometry_msgs::PoseStamped ros_planner_command_data;
    geometry_msgs::Pose ros_cartesian_command_data;

    /* Command from Orocos */
    InputPort<motion_control_msgs::JointPositions> orocos_arm_joint_position_command;
    InputPort<motion_control_msgs::JointVelocities> orocos_arm_joint_velocity_command;
    InputPort<motion_control_msgs::JointEfforts> orocos_arm_joint_effort_command;
    InputPort<geometry_msgs::Twist> orocos_base_cmd_twist;
    InputPort<motion_control_msgs::JointPositions> orocos_gripper_joint_position_command;

    motion_control_msgs::JointPositions orocos_arm_joint_position_command_data;
    motion_control_msgs::JointVelocities orocos_arm_joint_velocity_command_data;
    motion_control_msgs::JointEfforts orocos_arm_joint_effort_command_data;
    geometry_msgs::Twist orocos_base_cmd_twist_data;
    motion_control_msgs::JointPositions orocos_gripper_joint_position_command_data;

    /* Command to hw */
    OutputPort<motion_control_msgs::JointPositions> out_arm_joint_position_command;
    OutputPort<motion_control_msgs::JointVelocities> out_arm_joint_velocity_command;
    OutputPort<motion_control_msgs::JointEfforts> out_arm_joint_effort_command;
    OutputPort<geometry_msgs::Twist> out_base_cmd_twist;
    OutputPort<motion_control_msgs::JointPositions> out_gripper_joint_position_command;
    OutputPort<geometry_msgs::PoseStamped> out_ros_planner_command;
    OutputPort<geometry_msgs::Pose> out_ros_cartesian_command;

  private:
    void my_push_front(const queue_item& q);
    bool lastCartesianPoseIsReached();

  	bool isinloading;
  	std::deque<queue_item> queue;

    uint64_t my_time;
    uint64_t time_of_the_last;
    struct queue_item last_command_submitted;
};
#endif