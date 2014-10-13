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

#include "YouBotArmService.hpp"

#include <stdio.h>
#include <cassert>

#include "YouBotHelpers.hpp"
#include <youbot/ProtocolDefinitions.hpp>

namespace YouBot
{
  using namespace RTT;
  using namespace RTT::types;
  using namespace std;

  extern unsigned int non_errors;

  YouBotArmService::YouBotArmService(const string& name, TaskContext* parent,
      unsigned int min_slave_nr, long i_clientID) :
      Service(name, parent), m_joint_limits(NR_OF_ARM_SLAVES), m_joint_ctrl_modes(
          NR_OF_ARM_SLAVES, MOTOR_STOP),

      // Set the commands to zero depending on the number of joints
      m_calibrated(false), m_min_slave_nr(min_slave_nr),
      m_clientID(i_clientID)
  {
    m_VREP = (YouBotSIM*) parent;

    vrep_joint_handle.assign(NR_OF_ARM_SLAVES, 0);

    simxGetObjectHandle(m_clientID, "arm_joint_1", &vrep_joint_handle[0], simx_opmode_oneshot_wait);
    simxGetObjectHandle(m_clientID, "arm_joint_2", &vrep_joint_handle[1], simx_opmode_oneshot_wait);
    simxGetObjectHandle(m_clientID, "arm_joint_3", &vrep_joint_handle[2], simx_opmode_oneshot_wait);        
    simxGetObjectHandle(m_clientID, "arm_joint_4", &vrep_joint_handle[3], simx_opmode_oneshot_wait);
    simxGetObjectHandle(m_clientID, "arm_joint_5", &vrep_joint_handle[4], simx_opmode_oneshot_wait);
    simxGetObjectHandle(m_clientID, "arm_joint_6", &vrep_joint_handle[5], simx_opmode_oneshot_wait);        

    m_joint_state.name.push_back("arm_joint_1");
    m_joint_state.name.push_back("arm_joint_2");
    m_joint_state.name.push_back("arm_joint_3");
    m_joint_state.name.push_back("arm_joint_4");
    m_joint_state.name.push_back("arm_joint_5");

    m_joint_state.position.assign(NR_OF_ARM_SLAVES, 0);
    m_joint_state.velocity.assign(NR_OF_ARM_SLAVES, 0);
    m_joint_state.effort.assign(NR_OF_ARM_SLAVES, 0);

    m_joint_position_command.positions.assign(NR_OF_ARM_SLAVES, 0);
    m_joint_velocity_command.velocities.assign(NR_OF_ARM_SLAVES, 0);
    m_joint_effort_command.efforts.assign(NR_OF_ARM_SLAVES, 0);

    // Pre-allocate port memory for outputs
    joint_state.setDataSample(m_joint_state);

    // set to false
    memset(m_overcurrent, 0, NR_OF_ARM_SLAVES);
    memset(m_undervoltage, 0, NR_OF_ARM_SLAVES);
    memset(m_overvoltage, 0, NR_OF_ARM_SLAVES);
    memset(m_overtemperature, 0, NR_OF_ARM_SLAVES);
    memset(m_connectionlost, 0, NR_OF_ARM_SLAVES);
    memset(m_i2texceeded, 0, NR_OF_ARM_SLAVES);
    memset(m_timeout, 0, NR_OF_ARM_SLAVES);

    setupComponentInterface();
  }

  YouBotArmService::~YouBotArmService()
  {
    //delete m_manipulator;
  }

  void YouBotArmService::setupComponentInterface()
  {
    this->addPort("joint_state", joint_state).doc("Joint states");

    this->addPort("joint_position_command", joint_position_command).doc(
        "Command joint angles");
    this->addPort("joint_velocity_command", joint_velocity_command).doc(
        "Command joint velocities");
    this->addPort("joint_effort_command", joint_effort_command).doc(
        "Command joint torques");

	this->addPort("in_joint_state",in_joint_state).doc("Input joint states");

    this->addPort("out_joint_position_command", out_joint_position_command ).doc("Arm positions to simulated robot");
    this->addPort("out_joint_velocity_command", out_joint_velocity_command).doc("Arm velocities to simulated robot");
    this->addPort("out_joint_effort_command", out_joint_effort_command).doc("Arm torques to simulated robot");

    // Events - Pre-allocate port memory for outputs
    m_events.reserve(max_event_length);
    events.setDataSample(m_events);
    this->addPort("events", events).doc("Joint events");

    this->addOperation("start", &YouBotArmService::start, this);
    this->addOperation("update", &YouBotArmService::update, this);
    this->addOperation("calibrate", &YouBotArmService::calibrate, this);
    this->addOperation("stop", &YouBotArmService::stop, this);
    this->addOperation("cleanup", &YouBotArmService::cleanup, this);
    this->addOperation("sim_mode_ops", &YouBotArmService::sim_mode_ops, this);

	this->addOperation("setControlModesAll", &YouBotArmService::setControlModesAll,
		this, OwnThread).doc("Control modes can be set individually.");
    this->addOperation("setControlModes", &YouBotArmService::setControlModes,
        this, OwnThread).doc("Control modes can be set individually.");
    this->addOperation("getControlModes", &YouBotArmService::getControlModes,
        this, OwnThread).doc("Control modes are individual.");
    this->addOperation("displayMotorStatuses",
        &YouBotArmService::displayMotorStatuses, this, OwnThread);
    this->addOperation("clearControllerTimeouts",
        &YouBotArmService::clearControllerTimeouts, this, OwnThread);
  }

  void YouBotArmService::sim_mode_ops(int mode)
  {
    switch(mode)
    {
      case 1:
        is_in_visualization_mode = true;
      break;
      
      case 2:
        is_in_visualization_mode = false;
      break;
      
      default: break;
    }
  }

  void YouBotArmService::setControlModes(vector<ctrl_modes>& all)
  {
    m_joint_ctrl_modes = all;
  }

  void YouBotArmService::setControlModesAll(int mode)
  {
	for(int i=0;i<NR_OF_ARM_SLAVES;++i)
	{
		m_joint_ctrl_modes[i] = static_cast<ctrl_modes>(mode);

      switch (m_joint_ctrl_modes[i])
      {
      case (PLANE_ANGLE):
      {
        simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,1,simx_opmode_oneshot);
        break;
      }
      case (ANGULAR_VELOCITY):
      {
        simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);
        break;
      }
      case (TORQUE):
      {
        simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);
        break;
      }
      case (MOTOR_STOP):
      {
        simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,1,simx_opmode_oneshot);
        break;
      }
      case (TWIST):
      {
        log(Error) << "Case twist unable on arm." << endlog();
        this->getOwner()->error();
        break;
      }
      default:
      {
        log(Error) << "Case not recognized." << endlog();
        this->getOwner()->error();
        break;
      }
      }     
	}
  }

  void YouBotArmService::getControlModes(vector<ctrl_modes>& all)
  {
    all = m_joint_ctrl_modes;
  }

  void YouBotArmService::displayMotorStatuses(){}

  bool YouBotArmService::start()
  {
    if (m_calibrated)
    {
      clearControllerTimeouts();
      return true;
    }
    else
    {
      return false;
    }
  }

  void YouBotArmService::clearControllerTimeouts(){}

  void YouBotArmService::readJointStates()
  {
    //in_joint_state.read(m_joint_state);
    float p,v,e;

    for(int i = 0; i < NR_OF_ARM_SLAVES; ++i)
    {                
      simxGetJointPosition(m_clientID, vrep_joint_handle[i], &p, simx_opmode_buffer);
      simxGetObjectFloatParameter(m_clientID, vrep_joint_handle[i], 2012, &v, simx_opmode_buffer);
      simxGetJointForce(m_clientID, vrep_joint_handle[i], &e, simx_opmode_buffer);                  
 
      m_joint_state.position[i] = p;
      m_joint_state.velocity[i] = v;
      m_joint_state.effort[i] = e;
    }

    joint_state.write(m_joint_state);
  }

  void YouBotArmService::updateJointSetpoints()
  {
    m_out_joint_position_command.names.resize(0);
    m_out_joint_position_command.positions.resize(0);

    m_out_joint_velocity_command.names.resize(0);
    m_out_joint_velocity_command.velocities.resize(0);

    m_out_joint_effort_command.names.resize(0);
    m_out_joint_effort_command.efforts.resize(0);

    // InputPort -> YouBot
    FlowStatus f = joint_position_command.read(m_joint_position_command);
    FlowStatus f1 = joint_velocity_command.read(m_joint_velocity_command);
    FlowStatus f2 = joint_effort_command.read(m_joint_effort_command);

    // Update joint setpoints
    simxPauseCommunication(m_clientID,1);
    for (unsigned int joint_nr = 0; joint_nr < NR_OF_ARM_SLAVES; ++joint_nr)
    {
      assert(joint_nr < NR_OF_ARM_SLAVES);

      switch (m_joint_ctrl_modes[joint_nr])
      {
      case (PLANE_ANGLE):
      {
		    if(f != NewData) break;

        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[joint_nr],m_joint_position_command.positions[joint_nr], simx_opmode_streaming);
        break;
      }
      case (ANGULAR_VELOCITY):
      {
		    if(f1 != NewData) break;

        simxSetJointTargetVelocity(m_clientID,vrep_joint_handle[joint_nr],m_joint_velocity_command.velocities[joint_nr], simx_opmode_streaming);
        break;
      }
      case (TORQUE):
      {
		    if(f2 != NewData) break;

        simxSetJointForce(m_clientID,vrep_joint_handle[joint_nr],m_joint_effort_command.efforts[joint_nr], simx_opmode_streaming);
        break;
      }
      case (MOTOR_STOP):
      {
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[0],m_joint_state.position[0], simx_opmode_streaming);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[1],m_joint_state.position[1], simx_opmode_streaming);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[2],m_joint_state.position[2], simx_opmode_streaming);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[3],m_joint_state.position[3], simx_opmode_streaming);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[4],m_joint_state.position[4], simx_opmode_streaming);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[5],m_joint_state.position[5], simx_opmode_streaming);                 
        break;
      }
      default:
      {
        log(Error) << "ctrl_mode not recognized." << endlog();
        break;
      }
      }
    }
    simxPauseCommunication(m_clientID,0);

	if(f == NewData)
		out_joint_position_command.write(m_out_joint_position_command);
	if(f1 == NewData)
		out_joint_velocity_command.write(m_out_joint_velocity_command);
	if(f2 == NewData)
		out_joint_effort_command.write(m_out_joint_effort_command);
  }

  void YouBotArmService::checkMotorStatuses(){}

  void YouBotArmService::update()
  {
    if(is_in_visualization_mode)
    {
      in_joint_state.read(m_joint_state);
      simxSetJointPosition(m_clientID,vrep_joint_handle[0],m_joint_state.position[0], simx_opmode_streaming);
      simxSetJointPosition(m_clientID,vrep_joint_handle[1],m_joint_state.position[1], simx_opmode_streaming);
      simxSetJointPosition(m_clientID,vrep_joint_handle[2],m_joint_state.position[2], simx_opmode_streaming);
      simxSetJointPosition(m_clientID,vrep_joint_handle[3],m_joint_state.position[3], simx_opmode_streaming);
      simxSetJointPosition(m_clientID,vrep_joint_handle[4],m_joint_state.position[4], simx_opmode_streaming);
      simxSetJointPosition(m_clientID,vrep_joint_handle[5],m_joint_state.position[5], simx_opmode_streaming);
      return;
    }

    readJointStates();
    updateJointSetpoints();
    checkMotorStatuses();
  }

  bool YouBotArmService::calibrate()
  {
    log(Info) << "Calibrating YouBotArmService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void YouBotArmService::stop(){}

  void YouBotArmService::cleanup()
  {
    m_calibrated = false;
  }

}
