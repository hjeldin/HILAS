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

#include "ArmControllerMockup.h"
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

	const double unfoldedPosition[] = { 169 * M_PI / 180.0, 65 * M_PI / 180.0,
			-146 * M_PI / 180.0, 102.5 * M_PI / 180.0, 167.5 * M_PI / 180.0 }; //radian!

	const double foldedPosition[] = { 0.115385 * M_PI / 180.0, 0.0980769 * M_PI / 180.0,
			-0.0936 * M_PI / 180.0, 0.14831 * M_PI / 180.0, 4.7193 * M_PI / 180.0 }; //radian

	ArmControllerMockup::ArmControllerMockup(const string& name) :
			TaskContext(name, PreOperational),
//			m_joint_velocities(NR_OF_ARM_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_torques(NR_OF_ARM_SLAVES, quantity<si::torque>(0*newton_meter)),
//			m_joint_statuses(NR_OF_ARM_SLAVES, 0),
			m_modes(NR_OF_ARM_SLAVES, MOTOR_STOP)
			// Set the commands to zero depending on the number of joints
//			m_joint_cmd_velocities(NR_OF_ARM_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_cmd_torques(NR_OF_ARM_SLAVES, quantity<si::torque>(0*newton_meter)),
	{
		m_joint_states.position.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.velocity.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.effort.assign(NR_OF_ARM_SLAVES,0);

		m_joint_cmd_angles.positions.assign(NR_OF_ARM_SLAVES,0);

		this->addPort("joint_states" , joint_states).doc("Input joint states from the driver");
		this->addPort("joint_cmd_angles", joint_cmd_angles).doc("Output joint angle commands to the driver");

		joint_cmd_angles.setDataSample(m_joint_cmd_angles);

		this->addOperation("setJointAngles",&ArmControllerMockup::setJointAngles,this, OwnThread).doc("In degrees");
		this->addOperation("getJointAngles",&ArmControllerMockup::getJointAngles,this, OwnThread).doc("In degrees");
		this->addOperation("unfoldManipulator",&ArmControllerMockup::unfoldManipulator,this, OwnThread);
		this->addOperation("foldManipulator",&ArmControllerMockup::foldManipulator,this, OwnThread);
	}

	ArmControllerMockup::~ArmControllerMockup() {}

	void ArmControllerMockup::setJointAngles(vector< double >& angles, double epsilon)
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, PLANE_ANGLE);

		op_setControlModes(m_modes);

		// convert degrees to radian
		for(unsigned int  i = 0; i < angles.size(); ++i)
		{
			m_joint_cmd_angles.positions[i] = angles[i] * M_PI / 180; //to radian
		}

        joint_cmd_angles.write(m_joint_cmd_angles);

		bool done = false;
		while(!done)
		{
			joint_states.read(m_joint_states);

			done = true;
			for(unsigned int i = 0; i < angles.size(); ++i)
			{
				if( abs(m_joint_states.position[i] * 180 / M_PI - angles[i]) > epsilon) //in degrees
				{
					done = false;
					break;
				}
			}
		}
	}

	bool ArmControllerMockup::getJointAngles(vector< double >& angles)
	{
		if(angles.size() == m_joint_states.position.size())
		{
			// convert degrees to radian
			for(unsigned int  i = 0; i < angles.size(); ++i)
			{
				angles[i] = m_joint_states.position[i] * 180 / M_PI;
			}
			return true;
		}
		else
		{
			log(Error) << "Array size mismatch." << endlog();
			return false;
		}
	}

	void ArmControllerMockup::unfoldManipulator()
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, PLANE_ANGLE);

		op_setControlModes(m_modes);

		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joint_cmd_angles.positions[i] = unfoldedPosition[i];
		}
	}

	void ArmControllerMockup::foldManipulator()
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, PLANE_ANGLE);

		op_setControlModes(m_modes);

		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joint_cmd_angles.positions[i] = foldedPosition[i];
		}
	}

	bool ArmControllerMockup::configureHook()
	{
		return TaskContext::configureHook();
	}

	bool ArmControllerMockup::startHook()
	{
		if(!joint_states.connected() || !joint_cmd_angles.connected())
		{
			log(Error) << "Ports not connected." << endlog();
			return false;
		}

		TaskContext* task_ptr = getPeer("youbot");
		if(task_ptr == NULL)
		{
			log(Error) << "Could not find peer YouBot_OODL" << endlog();
			return false;
		}
		op_setControlModes = task_ptr->provides("Arm1")->getOperation("setControlModes");

		if(!op_setControlModes.ready())
		{
			log(Error) << "Could not connect to Arm1.setControllerModes" << endlog();
			return false;
		}

		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, MOTOR_STOP);
		op_setControlModes(m_modes);

		return TaskContext::startHook();
	}

	void ArmControllerMockup::updateHook()
	{
    TaskContext::updateHook();

    joint_states.read(m_joint_states);

    joint_cmd_angles.write(m_joint_cmd_angles);

	}

	void ArmControllerMockup::stopHook()
	{
		m_modes = vector<ctrl_modes>(NR_OF_ARM_SLAVES, MOTOR_STOP);
		op_setControlModes(m_modes);

    TaskContext::stopHook();
	}

	void ArmControllerMockup::cleanupHook()
	{
    TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::ArmControllerMockup )
