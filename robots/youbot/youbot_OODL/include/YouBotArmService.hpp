#pragma once

#include <IRobotArmService.hpp>

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <youbot/YouBotManipulator.hpp>
#include <youbot/YouBotJoint.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <stdio.h>
#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <youBot.hpp>

namespace YouBot
{

using namespace RTT;
using namespace RTT::types;
using namespace std;
using namespace youbot;
using namespace boost::units;
using namespace boost::units::si;

class YouBotArmService: public Hilas::IRobotArmService
{

  public:
    
    YouBotArmService(const string& name, TaskContext* parent);
    ~YouBotArmService();

    void displayMotorStatuses();
    void clearControllerTimeouts();

  private:
    
    bool calibrate();
    void cleanup();
    void stop();

    void readJointStates();
    void updateJointSetpoints();
    void checkMotorStatuses();

    JointAngleSetpoint m_tmp_joint_position_command;
    JointVelocitySetpoint m_tmp_joint_cmd_velocity;
    JointTorqueSetpoint m_tmp_joint_cmd_torque;

    YouBotManipulator* m_manipulator;
    YouBotJoint* m_joints[YouBot::NR_OF_ARM_SLAVES];

    bool m_overcurrent[YouBot::NR_OF_ARM_SLAVES];
    bool m_undervoltage[YouBot::NR_OF_ARM_SLAVES];
    bool m_overvoltage[YouBot::NR_OF_ARM_SLAVES];
    bool m_overtemperature[YouBot::NR_OF_ARM_SLAVES];
    bool m_connectionlost[YouBot::NR_OF_ARM_SLAVES];
    bool m_i2texceeded[YouBot::NR_OF_ARM_SLAVES];
    bool m_timeout[YouBot::NR_OF_ARM_SLAVES];

    const unsigned int m_min_slave_nr;
};

}
