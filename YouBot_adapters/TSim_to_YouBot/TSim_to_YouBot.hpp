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

#include <rtt/RTT.hpp>

#include <YouBotTypes.hpp>
#include <motion_control_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;

	class TSim_to_YouBot : public RTT::TaskContext
	{
		public:
			TSim_to_YouBot(std::string const& name);
			~TSim_to_YouBot();

			void initialize(ctrl_modes ctrl_mode, unsigned int dimension = 6);

			bool startHook() ;
			void updateHook() ;

		private:
			InputPort<std_msgs::Float64MultiArray > input_cmd_signal;

			OutputPort<motion_control_msgs::JointPositions> output_cmd_angles;
			OutputPort<motion_control_msgs::JointVelocities> output_cmd_velocities;
			OutputPort<motion_control_msgs::JointEfforts> output_cmd_torques;
			OutputPort<geometry_msgs::Twist> output_cmd_twist;

			std_msgs::Float64MultiArray m_input_cmd_signal;

			motion_control_msgs::JointVelocities m_output_cmd_velocities;
			motion_control_msgs::JointPositions m_output_cmd_angles;
			motion_control_msgs::JointEfforts m_output_cmd_torques;
			geometry_msgs::Twist m_output_cmd_twist;

			ctrl_modes m_ctrl_mode;

			unsigned int m_dimension;
	};
}

