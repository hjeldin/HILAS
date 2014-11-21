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

#include "YouBotOODL.hpp"
#include "YouBotBaseService.hpp"
#include "YouBotArmService.hpp"
#include "YouBotHelpers.hpp"
#include "YouBotGripperService.hpp"

#include <youbot/ProtocolDefinitions.hpp>
#include <youbot/EthercatMaster.hpp>

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

#include <generic/Logger.hpp>
#include <rtt/Logger.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

//	unsigned int non_errors = ::MOTOR_HALTED | ::PWM_MODE_ACTIVE | ::VELOCITY_MODE | ::POSITION_MODE | ::TORQUE_MODE | ::POSITION_REACHED | ::INITIALIZED;

	YouBotOODL::YouBotOODL(const string& name) :
	    TaskContext(name, PreOperational), m_communication_errors(0)
	{
		youbot::Logger::logginLevel = youbot::fatal;
		RTT::Logger* ins = RTT::Logger::Instance();
		ins->setLogLevel(RTT::Logger::Info);

		m_max_communication_errors = 100;
	}

	YouBotOODL::~YouBotOODL() {}

	bool YouBotOODL::configureHook()
	{
		// MUST BE THE FIRST ONE TO CALL getInstance!!!
		unsigned int nr_slaves = 0;

		try
		{
			m_ec_master = &EthercatMaster::getInstance("/youbot-ethercat.cfg", OODL_YOUBOT_CONFIG_DIR, false);

      if(m_ec_master == NULL || (m_ec_master != NULL && !m_ec_master->isEtherCATConnectionEstablished()))
      {
        log(Error) << "No EtherCat connection." << endlog();
        this->error();
        return false;
      }

      if(m_ec_master->isThreadActive())
      {
        log(Error) << "EtherCat thread detected, programming error." << endlog();
        this->error();
        return false;
      }

			nr_slaves = m_ec_master->getNumberOfSlaves();

			if(nr_slaves != (NR_OF_BASE_SLAVES) && nr_slaves != (NR_OF_BASE_SLAVES + NR_OF_ARM_SLAVES) && nr_slaves != (NR_OF_BASE_SLAVES + 2*NR_OF_ARM_SLAVES))
			{
				log(Error) << "Not a proper amount of Ethercat slaves, got:" << nr_slaves << endlog();
		    this->error();
				return false;
			}
		}
		catch (std::exception& e)
		{
			log(Error) << e.what() << endlog();
			this->exception();
			return false;
		}

		// Base
		this->provides()->addService(Service::shared_ptr( new YouBotBaseService("Base",this, 1) ) );
		update_ops.push_back(this->provides("Base")->getOperation("update"));
		calibrate_ops.push_back(this->provides("Base")->getOperation("calibrate"));
		start_ops.push_back(this->provides("Base")->getOperation("start"));
		stop_ops.push_back(this->provides("Base")->getOperation("stop"));
		cleanup_ops.push_back(this->provides("Base")->getOperation("cleanup"));
		log(Info) << "Detected youbot base, loading Base service" << endlog();

		if(nr_slaves == NR_OF_BASE_SLAVES + NR_OF_ARM_SLAVES) // Arm 2
		{
			// Arm 1
			this->provides()->addService(Service::shared_ptr( new YouBotArmService("Arm1",this, 1) ) );
			update_ops.push_back(this->provides("Arm1")->getOperation("update"));
			calibrate_ops.push_back(this->provides("Arm1")->getOperation("calibrate"));
			start_ops.push_back(this->provides("Arm1")->getOperation("start"));
			stop_ops.push_back(this->provides("Arm1")->getOperation("stop"));
			cleanup_ops.push_back(this->provides("Arm1")->getOperation("cleanup"));
			log(Info) << "Detected youbot arm, loading Arm1 service" << endlog();

			// Gripper 1
			this->provides()->addService(Service::shared_ptr( new YouBotGripperService("Gripper1",this) ) );
			update_ops.push_back(this->provides("Gripper1")->getOperation("update"));
			calibrate_ops.push_back(this->provides("Gripper1")->getOperation("calibrate"));
			start_ops.push_back(this->provides("Gripper1")->getOperation("start"));
			stop_ops.push_back(this->provides("Gripper1")->getOperation("stop"));
			cleanup_ops.push_back(this->provides("Gripper1")->getOperation("cleanup"));
			log(Info) << "Detected youbot gripper, loading Gripper1 service" << endlog();
		}

		if(nr_slaves == NR_OF_BASE_SLAVES + 2*NR_OF_ARM_SLAVES) // Arm 2
		{
			this->provides()->addService(Service::shared_ptr( new YouBotArmService("Arm2",this, 1 + NR_OF_ARM_SLAVES) ) );
			update_ops.push_back(this->provides("Arm2")->getOperation("update"));
			calibrate_ops.push_back(this->provides("Arm2")->getOperation("calibrate"));
			start_ops.push_back(this->provides("Arm2")->getOperation("start"));
			stop_ops.push_back(this->provides("Arm2")->getOperation("stop"));
			cleanup_ops.push_back(this->provides("Arm2")->getOperation("cleanup"));
			log(Info) << "Detected youbot arm, loading Arm2 service" << endlog();
		}

		Seconds period = this->getPeriod();
		if(period < 0.001)
		{
			log(Error) << "The EthercatMaster thread needs at least 1kHz frequency to operate properly." << endlog();
			return false;
		}

		log(Info) << "EthercatMaster thread period: " << period << endlog();

		// invoke all calibration operations
		bool proper_calibration(true);
		for(unsigned int i=0; i<calibrate_ops.size(); ++i)
		{
			if(calibrate_ops[i].ready())
			{
				if(!calibrate_ops[i]())
				{
					proper_calibration = false;
					log(Warning) << "Calibration failed on one Service." << endlog();
					break;
				}
			}
		}

		// Break off when a part of the robot couldn't calibrate.
		if(!proper_calibration)
		{
			cleanupHook();
			update_ops.clear();
			calibrate_ops.clear();
			start_ops.clear();
			stop_ops.clear();
			cleanup_ops.clear();
      this->error();
			return false;
		}
		else
		{
			return TaskContext::configureHook();
		}
	}

	bool YouBotOODL::startHook()
	{
		// invoke all starts
		bool fully_started(true);

    for(unsigned int i=0;i<start_ops.size();++i)
    {
        if(start_ops[i].ready())
        {
          if(!start_ops[i]())
          {
            fully_started = false;
            log(Warning) << "Start failed on one Service." << endlog();
            break;
          }
        }
    }

    ca = 0;
    ca_counter = 0;
    timestamp = RTT::os::TimeService::Instance()->getTicks();

    return fully_started ? TaskContext::startHook() : false;
	}

	void YouBotOODL::updateHook()
	{
                // Time measurements to confirm period
		Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince( timestamp );
        	ca = ca + (elapsed - ca) / (++ca_counter);
	        timestamp = RTT::os::TimeService::Instance()->getTicks(); //used cumulative average, because timestamps can overflow for long operation times.

		if(!m_ec_master->receiveProcessData())
		{
			++m_communication_errors;
			if(m_communication_errors > m_max_communication_errors)
			{
				log(Error) << "Lost EtherCAT connection";
				this->error();
			}
		}
		else
		{
			m_communication_errors = 0;
		}

		if(m_ec_master->isErrorInSoemDriver())
		{
			log(Error) << "Error in the SOEM driver." << endlog();
			this->error();
		}

		// The mailbox messages are send/received immediatly

        for(unsigned int i=0;i<update_ops.size();++i)
        {
            if(update_ops[i].ready())
            {
                update_ops[i]();
            }
        }

		if(!m_ec_master->sendProcessData())
		{
			++m_communication_errors;
			if(m_communication_errors > m_max_communication_errors)
			{
				log(Error) << "Lost EtherCAT connection";
				this->error();
			}
		}
		else
		{
			m_communication_errors = 0;
		}

    TaskContext::updateHook();
	}

	void YouBotOODL::stopHook()
	{
        for(unsigned int i=0;i<stop_ops.size();++i)
        {
            if(stop_ops[i].ready())
            {
            	stop_ops[i]();
            }
        }

	log(Info) << "YouBot_OODL cumulative average updateHook period: " << ca << endlog();

        TaskContext::stopHook();
	}

	void YouBotOODL::cleanupHook()
	{
        for(unsigned int i=0;i<cleanup_ops.size();++i)
        {
            if(cleanup_ops[i].ready())
            {
            	cleanup_ops[i]();
            }
        }

        TaskContext::cleanupHook();
	}
}

ORO_CREATE_COMPONENT(YouBot::YouBotOODL)