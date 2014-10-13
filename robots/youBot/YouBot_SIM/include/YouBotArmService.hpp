#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotManipulator.hpp>
#include <youbot/YouBotJoint.hpp>

#include "YouBotSIM.hpp"
#include "YouBotTypes.hpp"

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

#define VREP_JOINT_CONTROL_POSITION_IP 2001

namespace YouBot
{
using namespace RTT;
using namespace std;
using namespace youbot;
using namespace boost::units;
using namespace boost::units::si;

struct _JointLimits;
typedef struct _JointLimits JointLimits;

class YouBotArmService: public Service
{

  public:
    YouBotArmService(const string& name, TaskContext* parent,
        unsigned int min_slave_nr, long i_clientID);
    virtual ~YouBotArmService();

    void setControlModesAll(int mode);
    void setControlModes(vector<ctrl_modes>& all);
    void getControlModes(vector<ctrl_modes>& all);
    void sim_mode_ops(int mode);
    void displayMotorStatuses();

    void clearControllerTimeouts();

  protected:
    OutputPort<sensor_msgs::JointState> joint_state;

    InputPort<motion_control_msgs::JointPositions> joint_position_command;
    InputPort<motion_control_msgs::JointVelocities> joint_velocity_command;
    InputPort<motion_control_msgs::JointEfforts> joint_effort_command;

    OutputPort<std::string> events;

    /* Acquire VREP Data
       HW -> read from YuoBot_OODL (visualization mode)
       SIM -> read from VREP
    */

    InputPort<sensor_msgs::JointState> in_joint_state;
    //@todo catch events from VREP (ex. motor brake, ...)
    //InputPort<std::string> in_events;

    /* Port for send data to VREP */
    OutputPort<motion_control_msgs::JointPositions> out_joint_position_command;
    OutputPort<motion_control_msgs::JointVelocities> out_joint_velocity_command;
    OutputPort<motion_control_msgs::JointEfforts> out_joint_effort_command;

  private:
    void setupComponentInterface();

    bool calibrate();
    bool start();
    void update();
    void cleanup();
    void stop();

    void readJointStates();
    void updateJointSetpoints();
    void checkMotorStatuses();

    long m_clientID;
    vector<simxInt> vrep_joint_handle;
    bool is_in_visualization_mode;

    motion_control_msgs::JointPositions m_joint_position_command;
    motion_control_msgs::JointVelocities m_joint_velocity_command;
    motion_control_msgs::JointEfforts m_joint_effort_command;

    /* Dummy */
    motion_control_msgs::JointPositions m_out_joint_position_command;
    motion_control_msgs::JointVelocities m_out_joint_velocity_command;
    motion_control_msgs::JointEfforts m_out_joint_effort_command;

    sensor_msgs::JointState m_joint_state;

    std::string m_events;

    vector<JointLimits> m_joint_limits;

    vector<ctrl_modes> m_joint_ctrl_modes;

    //JointAngleSetpoint m_tmp_joint_position_command;
    //JointVelocitySetpoint m_tmp_joint_cmd_velocity;
    //JointTorqueSetpoint m_tmp_joint_cmd_torque;

    bool m_overcurrent[NR_OF_ARM_SLAVES];
    bool m_undervoltage[NR_OF_ARM_SLAVES];
    bool m_overvoltage[NR_OF_ARM_SLAVES];
    bool m_overtemperature[NR_OF_ARM_SLAVES];
    bool m_connectionlost[NR_OF_ARM_SLAVES];
    bool m_i2texceeded[NR_OF_ARM_SLAVES];
    bool m_timeout[NR_OF_ARM_SLAVES];

    //YouBotManipulator* m_manipulator;
    //YouBotJoint* m_joints[NR_OF_ARM_SLAVES];

    bool m_calibrated;

    const unsigned int m_min_slave_nr;

    YouBotSIM* m_VREP;
};

/**
 * @brief Part of the workaround to prevent the generation of Exceptions in OODL.
 */
struct _JointLimits
{
    quantity<plane_angle> min_angle;
    quantity<plane_angle> max_angle;

    _JointLimits() :
        min_angle(0 * radian), max_angle(0 * radian) //, min_velocity(0), max_velocity(0), min_torque(0), max_torque(0)
    {
    }
};

}
