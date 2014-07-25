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

#include "YouBotStateRepublisher.hpp"

/*
 * @brief Adapter to connect 20Sim to the YouBot OODL.
 */
namespace YouBot
{
using namespace RTT;
const size_t max_event_length = 255;
std::string& make_edge_event(std::string& s, const std::string& event, bool status)
 {
     char tmpstr[max_event_length];
     if(s.capacity() < max_event_length)
         log(Error) << "make_event: event string capacity < max_event_length." << endlog();

     if(status == true)
       snprintf(tmpstr, max_event_length, "%s,true", event.c_str());
     else
       snprintf(tmpstr, max_event_length, "%s,false", event.c_str());

     s.insert(0, tmpstr, max_event_length);
     return s;
 }
YouBotStateRepublisher::YouBotStateRepublisher(std::string const& name) :
				TaskContext(name, PreOperational), m_dimension(0), wheel(0)
{
	this->addPort("arm_state", arm_state);
	this->addPort("base_state", base_state);
	this->addPort("youbot_state", youbot_state);
	//Energy state republishing
	this->addPort("arm_energy_tank",arm_energy_tank);
	this->addPort("base_energy_tank",base_energy_tank);
	this->addPort("kinematics_energy_tank",kinematics_energy_tank);
	this->addPort("events",events);
  
  // 15 10 2013
  this->addPort("odometry_state",odometry_state);
  this->addPort("oodl_odometry_state",oodl_odometry_state);

	m_youbot_state.position.resize(SIZE_JOINT_NAME_ARRAY, 0.0);
	m_youbot_state.name.assign(JOINT_NAME_ARRAY,JOINT_NAME_ARRAY+SIZE_JOINT_NAME_ARRAY);
	youbot_state.setDataSample(m_youbot_state);
}

YouBotStateRepublisher::~YouBotStateRepublisher()
{

}

bool YouBotStateRepublisher::configureHook()
{

	if (!arm_state.connected())
	{
		log(Warning) << "The port arm_state is not connected" << endlog();
	}
	else if(arm_state.read(m_arm_state) != NoData && m_arm_state.position.size() != 5)
	{
	  log(Warning) << "The port arm_state does not have the right dimension." << endlog();
	}

	if (!base_state.connected())
	{
		log(Warning) << "The port base_state is not connected" << endlog();
	}
	else if(base_state.read(m_base_state) != NoData && m_base_state.position.size() != 4)
	{
		log(Warning) << "The port base_state does not have the right dimension." << endlog();
	}

	if (!arm_energy_tank.connected())
	{
		log(Warning) << "The port arm_energy_tank is not connected" << endlog();
	}

	if (!base_energy_tank.connected())
	{
		log(Warning) << "The port base_energy_tank is not connected" << endlog();
	}

	if (!kinematics_energy_tank.connected())
	{
		log(Warning) << "The port kinematics_energy_tank is not connected" << endlog();
	}

	if (!events.connected())
	{
		log(Warning) << "The port events is not connected" << endlog();
	}

	return TaskContext::configureHook();
}
bool YouBotStateRepublisher::startHook()
{
	return TaskContext::startHook();
}

void YouBotStateRepublisher::updateHook()
{
  m_youbot_state.header.stamp = ros::Time::now();
  if (arm_state.read(m_arm_state) == NewData)
  {
    m_youbot_state.position[0] = m_arm_state.position[0];
    m_youbot_state.position[1] = m_arm_state.position[1];
    m_youbot_state.position[2] = m_arm_state.position[2];
    m_youbot_state.position[3] = m_arm_state.position[3];
    m_youbot_state.position[4] = m_arm_state.position[4];
  }
  if (base_state.read(m_base_state) == NewData)
  {
    m_youbot_state.position[5] = m_base_state.position[0];
    m_youbot_state.position[6] = m_base_state.position[1];
    m_youbot_state.position[7] = m_base_state.position[2];
    m_youbot_state.position[8] = m_base_state.position[3];
  }
  
  // 15 10 2013
  m_odometry_state.header.stamp = ros::Time::now();  
  m_odometry_state.header.frame_id = "odom";
  m_odometry_state.child_frame_id = "/base_link";
  
  if (oodl_odometry_state.read(m_oodl_odometry_state) == NewData)
  {
    m_odometry_state.pose.pose.position.x =           m_oodl_odometry_state.pose.pose.position.x;
    m_odometry_state.pose.pose.position.y =           m_oodl_odometry_state.pose.pose.position.y;
    m_odometry_state.pose.pose.position.z =           m_oodl_odometry_state.pose.pose.position.z;
    m_odometry_state.pose.pose.orientation.x =        m_oodl_odometry_state.pose.pose.orientation.x;
    m_odometry_state.pose.pose.orientation.y =        m_oodl_odometry_state.pose.pose.orientation.y;
    m_odometry_state.pose.pose.orientation.z =        m_oodl_odometry_state.pose.pose.orientation.z;
    m_odometry_state.pose.pose.orientation.w =        m_oodl_odometry_state.pose.pose.orientation.w;
    
    memset(&m_odometry_state.pose.covariance, 0, sizeof(m_odometry_state.pose.covariance));
        
    m_odometry_state.twist.twist.linear.x =     m_oodl_odometry_state.twist.twist.linear.x;
    m_odometry_state.twist.twist.linear.y =     m_oodl_odometry_state.twist.twist.linear.y;
    m_odometry_state.twist.twist.linear.z =     m_oodl_odometry_state.twist.twist.linear.z;
    m_odometry_state.twist.twist.angular.x =    m_oodl_odometry_state.twist.twist.angular.x;
    m_odometry_state.twist.twist.angular.y =    m_oodl_odometry_state.twist.twist.angular.y;
    m_odometry_state.twist.twist.angular.z =    m_oodl_odometry_state.twist.twist.angular.z;
    memset(&m_odometry_state.twist.covariance, 0, sizeof(m_odometry_state.pose.covariance));
  }

  // casters remain 0

  // Gripper cannot be read in real-time (yet)
  m_youbot_state.position[13] = 0.001;
  m_youbot_state.position[14] = 0.001;

  youbot_state.write(m_youbot_state);
  odometry_state.write(m_odometry_state);

  if(arm_energy_tank.read(m_energy_tank) == NewData)
  {
	  if (m_energy_tank.data[0]<MIN_ENERGY && m_arm_energy_tank > MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",true));
	  }
	  if (m_energy_tank.data[0]>MIN_ENERGY && m_arm_energy_tank < MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",false));
	  }
	  m_arm_energy_tank=m_energy_tank.data[0];
  }
  if(base_energy_tank.read(m_energy_tank) == NewData)
  {
	  if (m_energy_tank.data[0]<MIN_ENERGY && m_base_energy_tank > MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",true));
	  }
	  if (m_energy_tank.data[0]>MIN_ENERGY && m_base_energy_tank < MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",false));
	  }
	  m_base_energy_tank=m_energy_tank.data[0];
  }
  if(kinematics_energy_tank.read(m_energy_tank) == NewData)
  {
	  if (m_energy_tank.data[0]<MIN_ENERGY && m_kinematics_energy_tank > MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",true));
	  }
	  if (m_energy_tank.data[0]>MIN_ENERGY && m_kinematics_energy_tank < MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",false));
	  }
	  m_kinematics_energy_tank=m_energy_tank.data[0];
  }
  
  
	TaskContext::updateHook();
}

}


ORO_CREATE_COMPONENT( YouBot::YouBotStateRepublisher)
