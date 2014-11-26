#pragma once

#include "IRobotBaseService.hpp"

#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotJoint.hpp>
#include <youbot/ProtocolDefinitions.hpp>
#include <base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp>

#include <youBot.hpp>

namespace YouBot
{

using namespace RTT;
using namespace RTT::types;
using namespace std;
using namespace youbot;

class YouBotBaseService: public Hilas::IRobotBaseService
{

  public:

    YouBotBaseService(const string& name, TaskContext* parent);
    ~YouBotBaseService();

    void displayMotorStatuses();
    void clearControllerTimeouts();

  private:

    bool calibrate();
    void cleanup();
    void stop();

    void readJointStates();
    void readOdometry();
    void setJointSetpoints();
    void setTwistSetpoints();
    void checkMotorStatuses();

    JointAngleSetpoint m_tmp_joint_position_command;
    JointVelocitySetpoint m_tmp_joint_velocity_command;
    JointTorqueSetpoint m_tmp_joint_effort_command;

    YouBotBase* m_base;
    YouBotJoint* m_joints[YouBot::NR_OF_BASE_SLAVES];

    bool m_overcurrent[YouBot::NR_OF_BASE_SLAVES];
    bool m_undervoltage[YouBot::NR_OF_BASE_SLAVES];
    bool m_overvoltage[YouBot::NR_OF_BASE_SLAVES];
    bool m_overtemperature[YouBot::NR_OF_BASE_SLAVES];
    bool m_connectionlost[YouBot::NR_OF_BASE_SLAVES];
    bool m_i2texceeded[YouBot::NR_OF_BASE_SLAVES];
    bool m_timeout[YouBot::NR_OF_BASE_SLAVES];

    const unsigned int m_min_slave_nr;
};

}