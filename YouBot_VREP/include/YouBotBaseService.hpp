#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <std_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

#include "YouBotVREP.hpp"
#include "YouBotTypes.hpp"

namespace YouBot
{
using namespace RTT;
using namespace std;

class YouBotBaseService: public Service
{

  public:
    YouBotBaseService(const string& name, TaskContext* parent,
        unsigned int min_slave_nr);
    virtual ~YouBotBaseService();

    void setControlModes(vector<ctrl_modes>& all);
    void getControlModes(vector<ctrl_modes>& all);

    void displayMotorStatuses();

    void clearControllerTimeouts();

  protected:
    OutputPort<sensor_msgs::JointState> joint_state;
    OutputPort<nav_msgs::Odometry> odometry_state;

    InputPort<motion_control_msgs::JointVelocities> joint_velocity_command;
    InputPort<motion_control_msgs::JointPositions> joint_position_command;
    InputPort<motion_control_msgs::JointEfforts> joint_effort_command;

    InputPort<geometry_msgs::Twist> cmd_twist;

    OutputPort<std::string> events;

  private:
    void setupComponentInterface();

    bool calibrate();
    bool start();
    void update();
    void cleanup();
    void stop();

    void readJointStates();
    void readOdometry();

    void setJointSetpoints();
    void setTwistSetpoints();

    void checkMotorStatuses();

    motion_control_msgs::JointVelocities m_joint_velocity_command;
    motion_control_msgs::JointPositions m_joint_position_command;
    motion_control_msgs::JointEfforts m_joint_effort_command;
    geometry_msgs::Twist m_cmd_twist;

    sensor_msgs::JointState m_joint_state;
    nav_msgs::Odometry m_odometry_state;

    std::string m_events;

    vector<ctrl_modes> m_joint_ctrl_modes;

    //JointAngleSetpoint m_tmp_joint_position_command;
    //JointVelocitySetpoint m_tmp_joint_velocity_command;
    //JointTorqueSetpoint m_tmp_joint_effort_command;

    //YouBotBase* m_base;
    //YouBotJoint* m_joints[NR_OF_BASE_SLAVES];

    bool m_overcurrent[NR_OF_BASE_SLAVES];
    bool m_undervoltage[NR_OF_BASE_SLAVES];
    bool m_overvoltage[NR_OF_BASE_SLAVES];
    bool m_overtemperature[NR_OF_BASE_SLAVES];
    bool m_connectionlost[NR_OF_BASE_SLAVES];
    bool m_i2texceeded[NR_OF_BASE_SLAVES];
    bool m_timeout[NR_OF_BASE_SLAVES];

    bool m_calibrated;

    const unsigned int m_min_slave_nr;

    YouBotVREP* m_VREP;
};

}
