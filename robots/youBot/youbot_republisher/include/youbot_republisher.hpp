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
 * Yury Brodskiy, Robert Wilterdink
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

#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <std_msgs/typekit/Types.h>
#include <ocl/Component.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>


namespace YouBot
{
  using namespace RTT;
  using namespace std;

  static const string JOINT_NAME_ARRAY[] =
  //{"j0","j1","j2","j3","j4","w1","w2","w3","w4","f1","f2"};
  { "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5",
      "wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br",
      "caster_joint_fl","caster_joint_fr", "caster_joint_bl", "caster_joint_br",
      "gripper_finger_joint_l", "gripper_finger_joint_r"};

  static const unsigned int SIZE_JOINT_NAME_ARRAY=15;
  static const double MIN_ENERGY =0.0;

  class YouBotStateRepublisher: public RTT::TaskContext
  {
  public:
    YouBotStateRepublisher(std::string const& name);
    ~YouBotStateRepublisher();

    bool startHook();
    void updateHook();
    bool configureHook();

  private:

    InputPort<sensor_msgs::JointState> arm_state_in;
    InputPort<sensor_msgs::JointState> base_state_in;
    
    sensor_msgs::JointState m_arm_state;
    sensor_msgs::JointState m_base_state;

    std_msgs::Float64MultiArray m_energy_tank;
    double m_arm_energy_tank;
    double m_base_energy_tank;
    double m_kinematics_energy_tank;
    std::string m_events;
  //	InputPort<flat_matrix_t> odometry;

    OutputPort<sensor_msgs::JointState> robot_state_out;
    
    // 15 10 2013
    OutputPort<nav_msgs::Odometry> odometry_state_out;
    InputPort<nav_msgs::Odometry> odometry_state_in;
    
    nav_msgs::Odometry m_odometry_state;
            
    sensor_msgs::JointState m_youbot_state;
    // Republishing energy states to make an event when energy in the tanks drop below zeros
    InputPort<std_msgs::Float64MultiArray> arm_energy_tank_in;
    InputPort<std_msgs::Float64MultiArray> base_energy_tank_in;
    InputPort<std_msgs::Float64MultiArray> kinematics_energy_tank_in;
    OutputPort<std::string> events_out;

    unsigned int m_dimension;
    double wheel;
  };

}