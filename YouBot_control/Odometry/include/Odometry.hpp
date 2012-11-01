#pragma once

/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  include\Odometry.hpp
 *  subm:  Odometry
 *  model: motion_stack
 *  expmt: motion_stack
 *  date:  November 1, 2012
 *  time:  4:08:55 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.4.1
 *
 **********************************************************/

/* This file describes the model functions
 that are supplied for computation.

 The model itself is the OdometryModel.cpp file
 */

#include "OdometryModel.hpp"

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

	class Odometry: public OdometryModel , public RTT::TaskContext
	{
	public:
		//enum stateflags_Odometry {initialrun, mainrun, finished};

		/**
		 * Odometry constructor
		 */
		Odometry(std::string name = "Odometry");

		/**
		 * Odometry destructor
		 */
		virtual ~Odometry(void);

		/**
		 * Odometry configuration code and returns false if startup fails
		 */
		bool configureHook ();

		/**
		 * Odometry startUp code and returns false if startup fails
		 */
		bool startHook ();

		/**
		 * Odometry Calculation executed in this Hook.
		 */
		void updateHook ();

		/**
		 * Odometry Terminate
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

