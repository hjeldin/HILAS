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
 * Yury Brodskiy 
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
#include <ocl/Component.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

namespace YouBot
{
        typedef std_msgs::Float64MultiArray flat_matrix_t;

	class Master_executive: public RTT::TaskContext
	{
	public:
		Master_executive(const std::string& name);
		virtual ~Master_executive(void);

		void submitEnergyQuanta(double joules);
		double getEnergyState1();

		void setCartesianStiffness(std::vector<double> stiffness_c);

//		bool configureHook ();
		bool startHook ();
		void updateHook ();
		void stopHook ();

	protected:
		void setCartSpaceStiffness();

		RTT::OutputPort<flat_matrix_t> EnergyQuanta;
		RTT::InputPort<flat_matrix_t > EnergyState1;

		RTT::InputPort<flat_matrix_t > stiffness_slider;
		RTT::OutputPort<flat_matrix_t> CartSpaceStiffness;

		RTT::InputPort<std_msgs::Bool> submit_quota;

	private:
		flat_matrix_t m_EnergyQuanta;
		flat_matrix_t m_EnergyState1;

		flat_matrix_t m_stiffness_slider;
		flat_matrix_t m_CartSpaceStiffness;
		flat_matrix_t m_CartSpaceStiffness_orig;

		std_msgs::Bool m_submit_quota;
		std_msgs::Bool m_submit_quota_prev;
		double m_quota;
	};

}

