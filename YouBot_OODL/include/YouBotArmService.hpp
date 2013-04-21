/**********************************************************************
 *
 * Copyright (c) 2010-2013
 * All rights reserved.
 *
 * Robotics and Mechatronics (RaM) group
 * Faculty of Electrical Engineering, Mathematics and Computer Science
 * University of Twente
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author(s):
 * Robert Wilterdink, Yury Brodskiy 
 *
 * Supervised by: 
 * Jan F. Broenink
 * 
 * The research leading to these results has received funding from the 
 * European Community's Seventh Framework Programme (FP7/2007-2013) 
 * under grant agreement no. FP7-ICT-231940-BRICS (Best Practice in 
 * Robotics).
 * 
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This software is published under a dual-license: GNU Lesser General 
 * Public License LGPL 2.1 and BSD license. The dual-license implies 
 * that users of this code may choose which terms they prefer.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above 
 *       copyright notice, this list of conditions and the following 
 *       disclaimer in the documentation and/or other materials 
 *       provided with the distribution.
 *     * Neither the name of the University of Twente nor the names of 
 *       its contributors may be used to endorse or promote products 
 *       derived from this software without specific prior written 
 *       permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more 
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 **********************************************************************/

#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotManipulator.hpp>
#include <youbot/YouBotJoint.hpp>

#include "YouBotOODL.hpp"
#include "YouBotTypes.hpp"

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

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
        unsigned int min_slave_nr);
    virtual ~YouBotArmService();

    void setControlModes(vector<ctrl_modes>& all);
    void getControlModes(vector<ctrl_modes>& all);

    void displayMotorStatuses();

    void clearControllerTimeouts();

  protected:
    OutputPort<sensor_msgs::JointState> joint_state;

    InputPort<motion_control_msgs::JointPositions> joint_position_command;
    InputPort<motion_control_msgs::JointVelocities> joint_velocity_command;
    InputPort<motion_control_msgs::JointEfforts> joint_effort_command;

    OutputPort<std::string> events;

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

    motion_control_msgs::JointPositions m_joint_position_command;
    motion_control_msgs::JointVelocities m_joint_velocity_command;
    motion_control_msgs::JointEfforts m_joint_effort_command;

    sensor_msgs::JointState m_joint_state;

    std::string m_events;

    vector<JointLimits> m_joint_limits;

    vector<ctrl_modes> m_joint_ctrl_modes;

    JointAngleSetpoint m_tmp_joint_position_command;
    JointVelocitySetpoint m_tmp_joint_cmd_velocity;
    JointTorqueSetpoint m_tmp_joint_cmd_torque;

    bool m_overcurrent[NR_OF_ARM_SLAVES];
    bool m_undervoltage[NR_OF_ARM_SLAVES];
    bool m_overvoltage[NR_OF_ARM_SLAVES];
    bool m_overtemperature[NR_OF_ARM_SLAVES];
    bool m_connectionlost[NR_OF_ARM_SLAVES];
    bool m_i2texceeded[NR_OF_ARM_SLAVES];
    bool m_timeout[NR_OF_ARM_SLAVES];

    YouBotManipulator* m_manipulator;
    YouBotJoint* m_joints[NR_OF_ARM_SLAVES];

    bool m_calibrated;

    const unsigned int m_min_slave_nr;

    YouBotOODL* m_OODL;
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
