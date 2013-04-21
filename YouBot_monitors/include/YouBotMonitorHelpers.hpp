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

#include "YouBotMonitorTypes.hpp"

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <tf/tf.h>

#include <boost/function.hpp>
#include <vector>
#include <cassert>

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	bool compare(const vector<unsigned int>& indices, const vector<double>& setp, const vector<double>& state, const vector<compare_type>& ct, const double epsilon)
	{
		assert(setp.size() == ct.size() && ct.size() == indices.size());

		unsigned int index = 0;
		unsigned int size = indices.size();

		for(unsigned int i = 0; i < size; ++i)
		{
			index = indices[i];
			double diff = state[index] - setp[i];

//			log(Info) << compare_type_tostring(ct) << " diff (" << cur[index] << " - " << (*setp)[index] << ") : " << diff << " epsilon: " << epsilon << endlog();

			if( ct[i] == LESS && diff >= 0 )
			{
				return false;
			}
			else if(ct[i] == LESS_EQUAL && diff > 0 )
			{
				return false;
			}
			else if(ct[i] == EQUAL)
			{
//				double relativeError = 0.0;
//				if(setp[i] == state[index])
//					continue; // true
//				else if(abs(setp[i]) > abs(state[index]))
//					relativeError = abs( (state[index] - setp[i]) / setp[i]);
//				else
//					relativeError =  / state[index]);

				if(abs(state[index] - setp[i]) > epsilon)
				{
//					log(Info) << "__false__" << endlog();
					return false;
				}
			}
			else if(ct[i] == GREATER_EQUAL && diff < 0 )
			{
				return false;
			}
			else if(ct[i] == GREATER && diff <= 0 )
			{
				return false;
			}
		}

		return (size > 0) ? true : false;
	}

	void homogeneous_to_xyzypr(const homogeneous_matrix_t& H, xyzypr_t& xyzypr)
	{
		assert(H.data.size() == 16 && xyzypr.size() == 6);

		xyzypr[0] = H.data[3];
		xyzypr[1] = H.data[7];
		xyzypr[2] = H.data[11];

		btMatrix3x3 rotMatrix(H.data[0], H.data[1], H.data[2], H.data[4], H.data[5], H.data[6], H.data[8], H.data[9], H.data[10]);

		btScalar y, p, r;
		rotMatrix.getEulerYPR(y, p, r);
		xyzypr[3] = y;
		xyzypr[4] = p;
		xyzypr[5] = r;
	}

	std::string control_space_tostring(const control_space& space)
	{
		if(space == CARTESIAN)
			return "CARTESIAN";
		else if(space == JOINT)
			return "JOINT";
		else
			return "error";
	}

	std::string control_space_toeventstring(const control_space& space)
	{
		if(space == CARTESIAN)
			return "cart";
		else if(space == JOINT)
			return "jnt";
		else
			return "error";
	}

	std::string physical_quantity_tostring(const physical_quantity& quantity)
	{
		if(quantity == MONITOR_POSITION)
			return "MONITOR_POSITION";
		else if(quantity == MONITOR_VELOCITY)
			return "MONITOR_VELOCITY";
		else if(quantity == MONITOR_EFFORT)
			return "MONITOR_EFFORT";
		else if(quantity == MONITOR_TIME)
			return "MONITOR_TIME";
		else
			return "error";
	}

	std::string physical_quantity_toeventstring(const physical_quantity& quantity)
	{
		if(quantity == MONITOR_POSITION)
			return "pos";
		else if(quantity == MONITOR_VELOCITY)
			return "vel";
		else if(quantity == MONITOR_EFFORT)
			return "eff";
		else if(quantity == MONITOR_TIME)
			return "time";
		else
			return "error";
	}

	std::string physical_part_tostring(const physical_part& part)
	{
		if(part == ARM)
			return "ARM";
		else if(part == BASE)
			return "BASE";
		else
			return "error";
	}

	std::string event_type_tostring(const event_type& e_type)
	{
		if(e_type == EDGE)
			return "EDGE";
		else if(e_type == LEVEL)
			return "LEVEL";
		else
			return "error";
	}

	std::string compare_type_tostring(const compare_type& c_type)
	{
		if(c_type == LESS)
			return "LESS";
		else if(c_type == LESS_EQUAL)
			return "LESS_EQUAL";
		else if(c_type == EQUAL)
			return "EQUAL";
		else if(c_type == GREATER)
			return "GREATER";
		else if(c_type == GREATER_EQUAL)
			return "GREATER_EQUAL";
		else
			return "error";
	}

}
