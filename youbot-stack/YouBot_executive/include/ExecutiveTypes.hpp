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

#include <std_msgs/Float64MultiArray.h>

namespace YouBot
{
	typedef std_msgs::Float64MultiArray flat_matrix_t;
	enum state_t {JOINT_CONTROL, CARTESIAN_CONTROL, GRAVITY_MODE, DUAL_CONTROL, NAVIGATION};

	static const unsigned int SIZE_CART_STIFFNESS=9;
	static const unsigned int SIZE_H=16;
	static const double GRIPPER_OPENING=0.022;

  static const unsigned int SIZE_ARM_JOINTS_ARRAY = 5;
  static const unsigned int SIZE_BASE_JOINTS_ARRAY = 3;
  static const unsigned int SIZE_CART_SPACE=6;//

	static const double UNFOLD_JOINT_POSE[]={0,0,0,0,0};
	static const double FOLD_JOINT_POSE[]={-2.8,-1.1,2.5,-1.76,-2.9};

	static const double UNFOLD_CART_POSE[]={0,0,1,0,0,0};
//	static const double BASIC_JOINT_STIFFNESS[]={0,0,0,10,5,5,5,5};
	static const double BASIC_CART_STIFFNESS[]={200,200,200,10,10,10,0,0,0};
	static const double DEFAULT_CART_DAMPING = 1.0;
	static const double EYE4[]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

//	static const double RETRACT_STIFFNESS_C[]={500,500,500,0,0,0,0,0,0};
	static const double GRIPPER_SIZE=-0.2;
	static const int X_H=3;
	static const int Y_H=7;
	static const int Z_H=11;
}
