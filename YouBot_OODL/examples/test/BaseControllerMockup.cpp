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

#include "BaseControllerMockup.h"
#include "YouBotOODL.hpp"

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace boost::units;
	using namespace boost::units::si;

	BaseControllerMockup::BaseControllerMockup(const string& name) :
			TaskContext(name, PreOperational)
//			m_joint_velocities(NR_OF_BASE_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_torques(NR_OF_BASE_SLAVES, quantity<si::torque>(0*newton_meter)),
//			m_joint_statuses(NR_OF_BASE_SLAVES, 0)
//			m_modes(NR_OF_BASE_SLAVES, PLANE_ANGLE),
			// Set the commands to zero depending on the number of joints
//			m_joint_cmd_velocities(NR_OF_BASE_SLAVES, quantity<si::angular_velocity>(0*radian_per_second)),
//			m_joint_cmd_torques(NR_OF_BASE_SLAVES, quantity<si::torque>(0*newton_meter)),
	{
		m_joint_states.position.assign(NR_OF_BASE_SLAVES,0);
		m_joint_states.velocity.assign(NR_OF_BASE_SLAVES,0);
		m_joint_states.effort.assign(NR_OF_BASE_SLAVES,0);

		m_joint_cmd_angles.positions.assign(NR_OF_BASE_SLAVES,0);

		this->addPort("joint_states" , joint_states).doc("Input joint states from the driver");
		this->addPort("joint_cmd_angles", joint_cmd_angles).doc("Output joint angle commands to the driver");

		joint_cmd_angles.setDataSample(m_joint_cmd_angles);

		this->addOperation("setJointAngles",&BaseControllerMockup::setJointAngles,this, OwnThread);

//		this->addOperation("getBasePosition",&BaseControllerMockup::getBasePosition,this, OwnThread);
//		this->addOperation("setBasePosition",&BaseControllerMockup::setBasePosition,this, OwnThread);
//
//		this->addOperation("setBaseVelocity",&BaseControllerMockup::setBaseVelocity,this, OwnThread);
	}

	BaseControllerMockup::~BaseControllerMockup() {}

	void BaseControllerMockup::setJointAngles(vector< double >& angles, double epsilon)
	{
		m_modes = vector<ctrl_modes>(NR_OF_BASE_SLAVES, PLANE_ANGLE);
		op_setControlModes(m_modes);

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

//	void BaseControllerMockup::getBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation)
//	{
//		quantity<length> _longitudinalPosition;
//		quantity<length> _transversalPosition;
//		quantity<plane_angle> _orientation;
//
//		op_getBasePosition(_longitudinalPosition, _transversalPosition, _orientation);
//		longitudinalPosition = _longitudinalPosition.value();
//		transversalPosition = _transversalPosition.value();
//		orientation = _orientation.value() * 180 / M_PI;
//	}
//
//	void BaseControllerMockup::setBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation)
//	{
//		m_modes = vector<ctrl_modes>(NR_OF_BASE_SLAVES, PLANE_ANGLE);
//
//		quantity<length> _longitudinalPosition = longitudinalPosition * si::meter;
//		quantity<length> _transversalPosition = transversalPosition * si::meter;
//		quantity<plane_angle> _orientation = orientation * M_PI / 180 * si::radian;
//		op_setBasePosition(_longitudinalPosition, _transversalPosition, _orientation);
//	}
//
//	void BaseControllerMockup::getBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity)
//	{
//		log(Error) << "Not implemented." << endlog();
//	}
//
//	void BaseControllerMockup::setBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity)
//	{
//		m_modes = vector<ctrl_modes>(NR_OF_BASE_SLAVES, ANGULAR_VELOCITY);
//		op_setControlModes(m_modes);
//
//		quantity<velocity> _longitudinalVelocity = longitudinalVelocity * si::meter_per_second;
//		quantity<velocity> _transversalVelocity = transversalVelocity * si::meter_per_second;
//		quantity<angular_velocity> _angularVelocity = angularVelocity * M_PI / 180 * si::radian_per_second;
//		op_setBaseVelocity(_longitudinalVelocity, _transversalVelocity, _angularVelocity);
//	}

	bool BaseControllerMockup::configureHook()
	{
		return TaskContext::configureHook();
	}

	bool BaseControllerMockup::startHook()
	{
		if(!joint_states.connected() || !joint_cmd_angles.connected() )
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
		op_setControlModes = task_ptr->provides("Base")->getOperation("setControlModes");

		if(!op_setControlModes.ready())
		{
			log(Error) << "Could not connect to Base services" << endlog();
			return false;
		}

		m_modes = vector<ctrl_modes>(NR_OF_BASE_SLAVES, MOTOR_STOP);
		op_setControlModes(m_modes);

		return TaskContext::startHook();
	}

	void BaseControllerMockup::updateHook()
	{
    TaskContext::updateHook();

    joint_states.read(m_joint_states);

    joint_cmd_angles.write(m_joint_cmd_angles);

	}

	void BaseControllerMockup::stopHook()
	{
		m_modes = vector<ctrl_modes>(NR_OF_BASE_SLAVES, MOTOR_STOP);
		op_setControlModes(m_modes);

    TaskContext::stopHook();
	}

	void BaseControllerMockup::cleanupHook()
	{
    TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT( YouBot::BaseControllerMockup )
