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

#pragma once

/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  include\CartesianController.hpp
 *  subm:  CartesianController
 *  model: motion_stack
 *  expmt: motion_stack
 *  date:  October 17, 2012
 *  time:  12:46:40 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.4.1
 *
 **********************************************************/

/* This file describes the model functions
 that are supplied for computation.

 The model itself is the CartesianControllerModel.cpp file
 */

#include "CartesianControllerModel.hpp"

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>

#include "Adapter20Sim.h"

#define COMPUTATION_TIME_MEASUREMENT 1

#ifdef COMPUTATION_TIME_MEASUREMENT
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#endif

namespace motion_stack
{
	using namespace common20sim;

	class CartesianController: public CartesianControllerModel , public RTT::TaskContext
	{
	public:
		//enum stateflags_CartesianController {initialrun, mainrun, finished};

		/**
		 * CartesianController constructor
		 */
		CartesianController(std::string name = "CartesianController");

		/**
		 * CartesianController destructor
		 */
		virtual ~CartesianController(void);

		/**
		 * CartesianController configuration code and returns false if startup fails
		 */
		bool configureHook ();

		/**
		 * CartesianController startUp code and returns false if startup fails
		 */
		bool startHook ();

		/**
		 * CartesianController Calculation executed in this Hook.
		 */
		void updateHook ();

		/**
		 * CartesianController Terminate
		 */
		void stopHook ();

		double getTime(void);

		virtual bool setPeriod(RTT::Seconds s);

	protected:
    virtual void CopyInputsToVariables();

    virtual void CopyVariablesToOutputs();

    void setupComponentInterface();

		/**
		 * OROCOS Ports for input and ouput
		 */
		std::vector< Adapter20Sim<RTT::InputPort<flat_matrix_t > > > inputPorts;
		std::vector< Adapter20Sim<RTT::OutputPort<flat_matrix_t > > > outputPorts;
		std::vector< Adapter20Sim<RTT::Property<RTT::types::carray<double> > > > propertyPorts;

	private:
		RTT::PropertyBag* createPropertyBags(std::string name, RTT::PropertyBag* head);
		void cleanupPropertyBags(RTT::PropertyBag* p);

		std::string m_config_file;

#ifdef COMPUTATION_TIME_MEASUREMENT
	  RTT::Seconds m_cum_avg;
	  unsigned int m_cum_avg_counter;
#endif

	};

}

