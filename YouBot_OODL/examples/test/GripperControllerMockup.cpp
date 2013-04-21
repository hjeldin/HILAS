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

#include "GripperControllerMockup.h"
#include "YouBotOODL.hpp"

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

#include <boost/units/systems/si.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace boost::units;
	using namespace boost::units::si;

	GripperControllerMockup::GripperControllerMockup(const string& name) :
			TaskContext(name, PreOperational)
	{
//		m_joint_states.position.assign(NR_OF_ARM_SLAVES,0);
//		m_joint_states.velocity.assign(NR_OF_ARM_SLAVES,0);
//		m_joint_states.effort.assign(NR_OF_ARM_SLAVES,0);

		m_gripper_cmd_position.positions.assign(1,0);

//		this->addPort("joint_states" , joint_states).doc("Input joint states from the driver");
		this->addPort("gripper_cmd_position", gripper_cmd_position).doc("Output gripper position.");

		gripper_cmd_position.setDataSample(m_gripper_cmd_position);

		this->addOperation("openGripper",&GripperControllerMockup::openGripper,this, OwnThread);
		this->addOperation("closeGripper",&GripperControllerMockup::closeGripper,this, OwnThread);
	}

	GripperControllerMockup::~GripperControllerMockup() {}

	void GripperControllerMockup::openGripper()
	{
		m_gripper_cmd_position.positions[0] = 0.01;
		gripper_cmd_position.write(m_gripper_cmd_position);
	}

	void GripperControllerMockup::closeGripper()
	{
		m_gripper_cmd_position.positions[0] = 0.0;
		gripper_cmd_position.write(m_gripper_cmd_position);
	}

	bool GripperControllerMockup::configureHook()
	{
		return TaskContext::configureHook();
	}

	bool GripperControllerMockup::startHook()
	{
		if(!gripper_cmd_position.connected())
		{
			log(Error) << "Ports not connected." << endlog();
			return false;
		}

		return TaskContext::startHook();
	}

	void GripperControllerMockup::updateHook()
	{
        TaskContext::updateHook();
	}

	void GripperControllerMockup::stopHook()
	{
        TaskContext::stopHook();
	}

	void GripperControllerMockup::cleanupHook()
	{
        TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::GripperControllerMockup )
