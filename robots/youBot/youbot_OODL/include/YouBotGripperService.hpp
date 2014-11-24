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

#include "YouBotOODL.hpp"

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
  using namespace RTT;
  using namespace std;
  using namespace youbot;
  using namespace boost::units;
  using namespace boost::units::si;

  /**
   * @brief Part of the workaround to prevent the generation of Exceptions in OODL.
   */
  struct _GripperLimits
  {
      quantity<si::length> min_position;
      quantity<si::length> max_position;

      _GripperLimits() :
          min_position(0 * meter), max_position(0 * meter)
      {
      }
  };
  typedef struct _GripperLimits GripperLimits;

  class YouBotGripperService: public Service
  {

    public:
      YouBotGripperService(const string& name, TaskContext* parent);
      virtual ~YouBotGripperService();

      void displayGripperStatus();

    protected:
      // Gripper
      InputPort<motion_control_msgs::JointPositions> gripper_cmd_position;
  //			OutputPort<sensor_msgs::JointState> gripper_state;

    private:
      bool calibrate();
      bool start();
      void update();
      void cleanup();
      void stop();

      void checkForErrors();

      // Gripper
      motion_control_msgs::JointPositions m_gripper_cmd_position;
  //	        sensor_msgs::JointState m_gripper_state;
      GripperBarSpacingSetPoint m_tmp_gripper_cmd_position;
  //	        GripperBarSpacingSetPoint m_tmp_gripper_state;
      GripperLimits m_gripper_limits;

      YouBotManipulator* m_manipulator;
      YouBotGripper* m_gripper;

      bool m_calibrated;
  };

}
