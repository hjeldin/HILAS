#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <stdio.h>
#include <cassert>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

#include <hilas.hpp>
#include <IRobot.hpp>

namespace Hilas
{

using namespace RTT::types;
using namespace RTT;
using namespace std;

class IRobotArmService: public Service
{

  public:

    IRobotArmService(const string& name, TaskContext* parent, int arm_slave_count, int robot_slave_count, const std::string arm_slave_name[], long m_clientID);
    virtual ~IRobotArmService();

    virtual void setControlModesAll(int mode);
    void setControlModes(vector<ctrl_modes>& all);
    void getControlModes(vector<ctrl_modes>& all);
    virtual void displayMotorStatuses();
    virtual void clearControllerTimeouts();
    void setsim_mode(int mode);

  protected:

    InputPort<motion_control_msgs::JointPositions> joint_position_command;
    InputPort<motion_control_msgs::JointVelocities> joint_velocity_command;
    InputPort<motion_control_msgs::JointEfforts> joint_effort_command;
    InputPort<sensor_msgs::JointState> in_joint_state;

    OutputPort<sensor_msgs::JointState> joint_state;
    OutputPort<std::string> events;

    void setupComponentInterface();

    bool start();
    void update();
    virtual void cleanup();
    virtual void stop();
    virtual bool calibrate();

    virtual void readJointStates() = 0;
    virtual void updateJointSetpoints() = 0;
    virtual void checkMotorStatuses();

    motion_control_msgs::JointPositions m_joint_position_command;
    motion_control_msgs::JointVelocities m_joint_velocity_command;
    motion_control_msgs::JointEfforts m_joint_effort_command;
    sensor_msgs::JointState m_joint_state;
    std::string m_events;

    vector<JointLimits> m_joint_limits;
    vector<ctrl_modes> m_joint_ctrl_modes;

    int NR_OF_ARM_SLAVES;
    long clientID;
    bool is_in_visualization_mode;    
    bool m_calibrated;
};

}
