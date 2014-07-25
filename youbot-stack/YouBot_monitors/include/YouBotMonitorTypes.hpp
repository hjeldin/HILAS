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

#include <boost/function.hpp>
#include <vector>

#include <ros/time.h>

#include <std_msgs/typekit/Types.h>

namespace YouBot
{
	typedef std_msgs::Float64MultiArray std_cart_t;
	typedef std_cart_t cart_efforts_t; // 6x1 vector (omega, trans)
	typedef std_cart_t homogeneous_matrix_t; // 4x4 matrix
	typedef std::vector<double> xyzypr_t; //internal

	const size_t max_event_length = 255;

	enum control_space 		{JOINT = 1, CARTESIAN = 2};
	enum physical_part 		{ARM = 1, BASE = 2}; //, BOTH = 3 - don't use BOTH yet
	enum physical_quantity 	{MONITOR_POSITION = 1, MONITOR_VELOCITY = 2, MONITOR_EFFORT = 3, MONITOR_TIME=4};
	enum event_type 		{EDGE = 1, LEVEL = 2};
	enum compare_type		{LESS = 1, LESS_EQUAL = 2, EQUAL = 3, GREATER = 4, GREATER_EQUAL = 5};

	typedef boost::function<bool() > monitor_fp;

	typedef struct _monitor
	{
		bool active;
		std::string descriptive_name;

		physical_part part;
		control_space space;
		physical_quantity quantity;
		event_type e_type;
		std::vector<compare_type> c_type;

		std::string id; // determined beforehand to speed up runtime execution.
		std::string msg;

		bool state; //for EDGE

		std::vector<bool> timer_state;
		std::vector<ros::Time> timer_expires;

		double epsilon;
		std::vector<unsigned int> indices;
		std::vector<double> values;

		monitor_fp check;

		_monitor() : active(false), part(ARM), space(JOINT), quantity(MONITOR_POSITION), e_type(EDGE), c_type(1, EQUAL),
				id(""), msg(""),
				state(false),
				timer_state(0),
				timer_expires(0),
				epsilon(5),
				indices(6),
				values(6)
		{}

		_monitor(const _monitor& copy) :
			active(false), part(copy.part), space(copy.space), quantity(copy.quantity), e_type(copy.e_type), c_type(copy.c_type),
				id(copy.id), msg(copy.msg),
				state(false),
				timer_state(0),
				timer_expires(0),
				epsilon(copy.epsilon),
				indices(copy.indices),
				values(copy.values)
		{}
	} monitor;

}
