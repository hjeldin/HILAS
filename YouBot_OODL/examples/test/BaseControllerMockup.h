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

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>

#include "YouBotOODL.hpp"
#include <boost/units/systems/si.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;
	using namespace boost::units;
	using namespace boost::units::si;

	class BaseControllerMockup: public TaskContext
	{
		public:
		BaseControllerMockup(const string& name);
		virtual ~BaseControllerMockup();

//		void getBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation);
//		void setBasePosition(double& longitudinalPosition, double& transversalPosition, double& orientation);
//
//		void getBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity);
//		void setBaseVelocity(double& longitudinalVelocity, double& transversalVelocity, double& angularVelocity);

		void setJointAngles(vector< double >& angles, double epsilon);

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

		private:
			// Ports to and from OODL
			InputPort< sensor_msgs::JointState > joint_states;
			OutputPort< motion_control_msgs::JointPositions > joint_cmd_angles;

			// Ports to a setpoint generating device (e.g. joystick)
			InputPort< vector<double> > setpoint;

			sensor_msgs::JointState m_joint_states;
			vector<ctrl_modes> m_modes;

			motion_control_msgs::JointPositions m_joint_cmd_angles;
//			vector<quantity<si::angular_velocity> > m_joint_cmd_velocities;
//			vector<quantity<si::torque> > m_joint_cmd_torques;

			OperationCaller<void(vector<ctrl_modes>) > op_setControlModes;

//			OperationCaller<void(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation)> op_getBasePosition;
//			OperationCaller<void(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation)> op_setBasePosition;
//
//			OperationCaller<void(quantity<velocity>& longitudinalPosition, quantity<velocity>& transversalPosition, quantity<angular_velocity>& orientation)> op_setBaseVelocity;

	};
}
