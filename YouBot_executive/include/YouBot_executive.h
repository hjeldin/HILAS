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

/*
 * YouBot_executive.h
 *
 *  Created on: Dec 15, 2011
 *      Author: Yury Brodskiy
 */
#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <motion_control_msgs/JointPositions.h>

#include <list>

#include <YouBotTypes.hpp>

#include "ExecutiveTypes.hpp"
#include "ConnectionMapping.hpp"

namespace YouBot
{

  using namespace RTT;
  using namespace std;

  class YouBot_executive: public TaskContext
  {
  public:
    YouBot_executive(const string& name);
    virtual ~YouBot_executive();

    virtual bool configureHook();
    virtual void updateHook();

    void setupGravityMode();
    void setupJointControl();
    void setupCartesianControl();
    void setupDualControl();
    void setupNavigationControl();

    // predefined settings
    void unfoldArmPosition();
    void foldArmPosition();
    // Specify the DOF to use
    void useBaseOnly();
    void useArmOnly();
    void useFullRobot();

    // Apply
    void execute();

    // predefined actions
    void openGripper();
    void closeGripper();

    // Cartesian space actions
    void setHvp0(vector<double> position_c);
    void setHtipCC(vector<double> position_c);
    void setCartesianStiffness(vector<double> stiffness_c);
    void setCartesianDamping(vector<double> damping);

    //Joint Space actions
    void setArmJointAngles(vector<double> position_j);
    void setHBase0(vector<double> position);

    //readouts
    void getArmJointStates(vector<double>& sample);
    void getTip_xyzypr(vector<double>& sample);
    void getHtip0(vector<double>& sample_H);
    void getHBase0(vector<double>& sample_H);

    void doneEvent();

    // hacks
    void sleep(double seconds);
    void quaternionToH(vector<double>& quat, vector<double>& H);

    // Ports and their variables
    RTT::OutputPort<flat_matrix_t> ArmJointAnglesSetpoint;
    RTT::OutputPort<flat_matrix_t> HBase0Setpoint;

    RTT::OutputPort<flat_matrix_t> ArmJointActive;
    RTT::OutputPort<flat_matrix_t> BaseJointActive;

    RTT::OutputPort<flat_matrix_t> CartSpaceSetpoint;
    RTT::OutputPort<flat_matrix_t> CartSpaceStiffness;
    RTT::OutputPort<flat_matrix_t> HtipCC;
    RTT::OutputPort<flat_matrix_t> CartSpaceDamping;

    RTT::InputPort<flat_matrix_t> Htip0;
    RTT::InputPort<flat_matrix_t> ArmJointStates;
    RTT::InputPort<flat_matrix_t> Wtip0;
    RTT::InputPort<flat_matrix_t> H_base_0;

    RTT::InputPort<std_msgs::Float64MultiArray> stiffness_slider;

    RTT::InputPort<std_msgs::Bool> open_gripper;

    RTT::OutputPort<motion_control_msgs::JointPositions> gripper_cmd;

    RTT::OutputPort<std::string> events;

  protected:
    void setupComponentInterface();
    void init();

    void calculateCartStiffness();

    void readAll();

    void stateTransition(state_t new_state);

    void clearControlModes(); // Go to gravityMode settings -> always safe!

    // Variables for ports
    flat_matrix_t m_ArmJointAnglesSetpoint;
    flat_matrix_t m_HBase0Setpoint;

    flat_matrix_t m_ArmJointActive;
    flat_matrix_t m_BaseJointActive;

    flat_matrix_t m_Hvp0;
    flat_matrix_t m_CartSpaceStiffness;
    flat_matrix_t m_CartSpaceStiffness_orig;
    flat_matrix_t m_HtipCC;
    flat_matrix_t m_CartSpaceDamping;

    flat_matrix_t m_Htip0;
    flat_matrix_t m_ArmJointState;
    flat_matrix_t m_Wtip0;
    flat_matrix_t m_H_base_0;

    std_msgs::Float64MultiArray m_stiffness_slider;

    std_msgs::Bool m_open_gripper;

    motion_control_msgs::JointPositions m_gripper_cmd;

    OperationCaller<void(vector<ctrl_modes>)> base_setControlModes;
    OperationCaller<void(vector<ctrl_modes>)> arm1_setControlModes;
    vector<ctrl_modes> m_base_ctrl_modes;
    vector<ctrl_modes> m_arm1_ctrl_modes;

    state_t m_state;

    bool use_stiffness_slider;

    std::string m_events;
  };

}
