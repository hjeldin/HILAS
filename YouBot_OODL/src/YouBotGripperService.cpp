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

#include "YouBotGripperService.hpp"

#include <stdio.h>
#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>

#include "YouBotHelpers.hpp"

namespace YouBot
{
  using namespace RTT;
  using namespace RTT::types;
  using namespace std;

  YouBotGripperService::YouBotGripperService(const string& name,
      TaskContext* parent) :
      Service(name, parent),
      // Set the commands to zero depending on the number of joints
      m_calibrated(false)
  {

  //		m_gripper_state.name.assign(1, "");
  //		m_gripper_state.name[0] = "gripper";

  //		m_gripper_state.position.assign(1, 0);
  //		m_gripper_state.velocity.assign(0);
  //		m_gripper_state.effort.assign(0);

    m_gripper_cmd_position.positions.resize(1, 0);

    this->addPort("gripper_cmd_position", gripper_cmd_position).doc(
        "Command the gripper position");

    this->addOperation("start", &YouBotGripperService::start, this);
    this->addOperation("update", &YouBotGripperService::update, this);
    this->addOperation("calibrate", &YouBotGripperService::calibrate, this);
    this->addOperation("stop", &YouBotGripperService::stop, this);
    this->addOperation("cleanup", &YouBotGripperService::cleanup, this);

  //        this->addOperation("displayGripperStatus",&YouBotGripperService::displayGripperStatus,this, OwnThread);

  // Pre-allocate port memory for outputs
  //        gripper_state.setDataSample(m_gripper_state);
  }

  YouBotGripperService::~YouBotGripperService()
  {
    delete m_manipulator;
  }

  void YouBotGripperService::displayGripperStatus()
  {
    log(Warning) << "Not implemented." << endlog();
  }

  bool YouBotGripperService::start()
  {
    return m_calibrated;
  }

  void YouBotGripperService::update()
  {
  //		ros::Time time = ros::Time::now();

  // The OODL gripper does not support this.
  //		m_gripper->getData(m_tmp_gripper_state);

  //		m_gripper_state.header.stamp = m_joint_states.header.stamp;
  //		m_gripper_state.position[0] = m_tmp_gripper_state.barSpacing.value();

  //		gripper_state.write(m_gripper_state);

  // Update gripper setpoint
    if (gripper_cmd_position.read(m_gripper_cmd_position) == NewData) //setData has SLEEP_MILLISECOND :-(
    {
      m_tmp_gripper_cmd_position.barSpacing = m_gripper_cmd_position.positions[0]
          * si::meter;
  //			// check limits to prevent exceptions
  //			if( m_tmp_gripper_cmd_position.barSpacing < m_gripper_limits.min_position )
  //			{
  //				m_tmp_gripper_cmd_position.barSpacing = m_gripper_limits.min_position;
  //			}
  //			//above limits:
  //			else if(m_tmp_gripper_cmd_position.barSpacing > m_gripper_limits.max_position)
  //			{
  //				m_tmp_gripper_cmd_position.barSpacing = m_gripper_limits.max_position;
  //			}

      m_gripper->setData(m_tmp_gripper_cmd_position);
    }

    // Check for errors:
  //		checkForErrors();
  }

  bool YouBotGripperService::calibrate()
  {
    log(Info) << "Calibrating YouBotGripperService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "Already calibrated." << endlog();
      return m_calibrated;
    }

    try
    {
      m_manipulator = new YouBotManipulator("/youbot-manipulator",
          OODL_YOUBOT_CONFIG_DIR);
      if (m_manipulator == NULL)
      {
        log(Error) << "Could not create the YouBotManipulator." << endlog();
        return false;
      }

      // Gripper
      m_manipulator->calibrateGripper();
      m_gripper = &(m_manipulator->getArmGripper());

  //			// Determine gripper limits to prevent exceptions
  //			MaxTravelDistance _max_distance;
  //			BarSpacingOffset _spacing;
  //			quantity<length> max_distance;
  //			quantity<length> spacing;
  //			m_gripper->getConfigurationParameter(_max_distance, BAR_ONE);
  //			m_gripper->getConfigurationParameter(_spacing, BAR_ONE);
  //			_max_distance.getParameter(max_distance);
  //			_spacing.getParameter(spacing);
  //			log(Info) << "Spacing: " << spacing << " max_distance: " << max_distance << endlog();
  //
  //			m_gripper->getConfigurationParameter(_max_distance, BAR_TWO);
  //			m_gripper->getConfigurationParameter(_spacing, BAR_TWO);
  //			_max_distance.getParameter(max_distance);
  //			_spacing.getParameter(spacing);
  //			log(Info) << "Spacing: " << spacing << " max_distance: " << max_distance << endlog();
  //
  //			m_gripper_limits.min_position = spacing;
  //			m_gripper_limits.max_position = max_distance + spacing;

      log(Info) << "Gripper calibration min_position: "
          << m_gripper_limits.min_position << " max_position: "
          << m_gripper_limits.max_position << endlog();
    } catch (std::exception& e)
    {
      log(Error) << e.what() << endlog();
      return false;
    }

    log(Info) << "Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void YouBotGripperService::checkForErrors()
  {
    log(Warning) << "checkForErrors - Not implemented" << endlog();
  }

  void YouBotGripperService::stop()
  {
  }

  void YouBotGripperService::cleanup()
  {
    m_calibrated = false;
  }

}
