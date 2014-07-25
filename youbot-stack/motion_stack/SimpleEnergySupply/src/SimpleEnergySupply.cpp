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

/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  src\SimpleEnergySupply.cpp
 *  subm:  SimpleEnergySupply
 *  model: motion_stack
 *  expmt: motion_stack
 *  date:  November 2, 2012
 *  time:  9:06:23 am
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.4.1
 **********************************************************/

/* Standard include files */
#include <stdio.h>
#include <math.h>
#include <stdexcept>

/* 20-sim include files */
#include "SimpleEnergySupply.hpp"

/* Orocos include */
#include <boost/algorithm/string.hpp>
#include <ocl/Component.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/carray.hpp>
#include <ros/package.h>
//#include <rtt/Logger.hpp>

using namespace RTT;
using namespace std;

namespace motion_stack
{
	SimpleEnergySupply::SimpleEnergySupply(string name): SimpleEnergySupplyModel(), TaskContext(name, PreOperational)
	{
		using namespace boost;

		//RTT::Logger* l = RTT::Logger::Instance();
		//l->setLogLevel(RTT::Logger::Debug);

    try
    {
      m_config_file = ros::package::getPath("SimpleEnergySupply");
      m_config_file = m_config_file + "/config/SimpleEnergySupply_config.xxsim";

      SimpleEnergySupplyModel::loadModelConfiguration(m_config_file);
      log(Info) << "Loaded XXModelProperties" << endlog();
    }
    catch(std::exception& e)
    {
      log(Error) << e.what() << endlog();
			this->error();
      this->exception();
      throw(e);
    }

    SimpleEnergySupplyModel::configure();

		setupComponentInterface();
	}

	SimpleEnergySupply::~SimpleEnergySupply(void)
	{
		cleanupPropertyBags(this->properties());
	}

	bool SimpleEnergySupply::configureHook()
	{
		if(! TaskContext::configureHook())
		{
			return false;
		}
		return true;
	}

	/* the initialization function for submodel */
	bool SimpleEnergySupply::startHook()
	{
    if(TaskContext::getPeriod() == 0.0)
    {
       log(Warning) << "No period set, setting integration_step_size as Activity period." << endlog();
       setPeriod(SimpleEnergySupplyModel::getPeriod());
    }
    else
    {
       // Sync period and integration_step_size;
      SimpleEnergySupplyModel::setPeriod(TaskContext::getPeriod());
    }

		if(! TaskContext::startHook())
		{
			return false;
		}

		for(unsigned int i = 0; i < inputPorts.size(); ++i)
		{
		  if(!inputPorts[i].getPort()->connected() )
      {
        log(Warning) << "InputPort (" << inputPorts[i].getFullName() << ") not connected." << endlog();
      }
		}

    for(unsigned int i = 0; i < outputPorts.size(); ++i)
    {
      if(!outputPorts[i].getPort()->connected() )
      {
        log(Warning) << "OutputPort (" << outputPorts[i].getFullName() << ") not connected." << endlog();
      }
      outputPorts[i].getPort()->setDataSample( outputPorts[i].getPortData() );
    }

    SimpleEnergySupplyModel::start();

#ifdef COMPUTATION_TIME_MEASUREMENT
    m_cum_avg = 0;
    m_cum_avg_counter = 0;
#endif

		return true;
	}

	/* the function that calculates the submodel */
	void SimpleEnergySupply::updateHook ()
	{
		TaskContext::updateHook();

#ifdef COMPUTATION_TIME_MEASUREMENT
		using namespace RTT::os;
    TimeService::ticks timestamp = TimeService::Instance()->getTicks();
#endif

		SimpleEnergySupplyModel::step();

#ifdef COMPUTATION_TIME_MEASUREMENT
		Seconds elapsed = TimeService::Instance()->secondsSince( timestamp );
    m_cum_avg = m_cum_avg + (elapsed - m_cum_avg) / (++m_cum_avg_counter);
#endif
	}

	/* the termination function for submodel */
	void SimpleEnergySupply::stopHook()
	{
		TaskContext::stopHook();

#ifdef COMPUTATION_TIME_MEASUREMENT
		log(Info) << "Cumulative average computation time: " << m_cum_avg << endlog();
#endif

		SimpleEnergySupplyModel::stop();
	}

	/* this PRIVATE function sets the input variables from the input vector */
	//@todo Improve for multiple component inputs to have a synchronized execution.
	void SimpleEnergySupply::CopyInputsToVariables ()
	{
		/* OROCOS Entry to copy port to input array */
    for (vector<Adapter20Sim<RTT::InputPort<flat_matrix_t> > >::iterator it =
				inputPorts.begin(); it != inputPorts.end(); ++it)
	  {
			if(it->getPort()->read(it->getPortData())!=RTT::NoData)
			{
				it->copyPortToVariable();
			}
	  }
	}

	/* this PRIVATE function uses the output variables to fill the output vector */
	void SimpleEnergySupply::CopyVariablesToOutputs ()
	{
		/* OROCOS Entry to copy output to port */
	  for (vector<Adapter20Sim<RTT::OutputPort<flat_matrix_t> > >::iterator it =
				outputPorts.begin(); it != outputPorts.end(); ++it)
	  {
			it->copyVariableToPort();
			it->getPort()->write(it->getPortData());
	  }
	}

	bool SimpleEnergySupply::setPeriod(RTT::Seconds s)
	{
		if(TaskContext::setPeriod(s))
		{
		  SimpleEnergySupplyModel::setPeriod(s);
			return true;
		}
		else
		{
			return false;
		}
	}

  void SimpleEnergySupply::setupComponentInterface()
  {
//    this->addProperty("integration_step_size", step_size ).doc("Integration step size.");
    this->addProperty("configuration_file", m_config_file).doc("Path to configuation xml, relative to run directory");

    std::vector<XVMatrix>& pps = SimpleEnergySupplyModel::getModelConfiguration().getConfiguration();
    log(Info) << "Number of ports and properties in XML: " << pps.size() << endlog();

    for(unsigned int i = 0; i < pps.size(); ++i)
    {
      log(Debug) << "Name: " << pps[i].name << " CEType: " << pps[i].type << " Storage: " << pps[i].storage.mat << " Rows: " << pps[i].storage.rows << " Columns: " << pps[i].storage.columns << endlog();

      switch(pps[i].type)
      {
        case(INPUT):
        {
          RTT::InputPort<flat_matrix_t> * rtt = new RTT::InputPort<flat_matrix_t>;
          Adapter20Sim<RTT::InputPort<flat_matrix_t> > xxsim(pps[i], rtt);
          this->addPort(xxsim.getFullName(), *rtt).doc(xxsim.getDescription());
          inputPorts.push_back(xxsim);
          break;
        }
        case(OUTPUT):
        {
          RTT::OutputPort<flat_matrix_t> * rtt = new RTT::OutputPort<flat_matrix_t>;
          Adapter20Sim<RTT::OutputPort<flat_matrix_t> > xxsim(pps[i], rtt);
          this->addPort(xxsim.getFullName(), *rtt).doc(xxsim.getDescription());
          outputPorts.push_back(xxsim);
          break;
        }
        case(PARAMETER):
        {
          RTT::PropertyBag* p_bag = createPropertyBags(pps[i].name, NULL); // Create the sub-model hierarchy
          Property<RTT::types::carray<double> >* prop = new Property<RTT::types::carray<double> >(makeShortName(pps[i].name), pps[i].description,
              RTT::types::carray<double>(pps[i].storage.mat, static_cast<std::size_t>(pps[i].storage.rows * pps[i].storage.columns)));
          Adapter20Sim<RTT::Property<RTT::types::carray<double> > > xxsim(pps[i], prop);
          p_bag->addProperty(*prop);
          propertyPorts.push_back(xxsim);
          break;
        }
        default:
        {
          break;
        }
      }
    }
    log(Info) << "Total input ports: " << inputPorts.size() << endlog();
    log(Info) << "Total output ports: " << outputPorts.size() << endlog();
    log(Info) << "Total properties: " << propertyPorts.size() << endlog();
  }

  RTT::PropertyBag* SimpleEnergySupply::createPropertyBags(std::string name, RTT::PropertyBag* head)
  {
    size_t found;
    found=name.find_first_of("\\");
    RTT::PropertyBag* p_bag(NULL);

    if(found != string::npos)
    {
      std::string sub_name(name, 0, found);
      sub_name = replaceIllegalCharacter(sub_name);

      if(head == NULL)
      {
        RTT::Property<PropertyBag>* ppb = dynamic_cast<RTT::Property<PropertyBag>*>(this->getProperty(sub_name));
        if(ppb == NULL)
        {
          p_bag = new RTT::PropertyBag;
          this->addProperty(sub_name, *p_bag).doc("Submodel parameters");
        }
        else
        {
          p_bag = &(ppb->value());
        }
      }
      else
      {
        RTT::Property<PropertyBag>* ppb = dynamic_cast<RTT::Property<PropertyBag>*>(head->getProperty(sub_name));
        RTT::PropertyBag* ptmp(NULL);
        if(ppb == NULL)
        {
          ptmp = new RTT::PropertyBag;
          head->addProperty(sub_name, *ptmp).doc("Submodel parameters");
        }
        else
        {
          ptmp = &(ppb->value());
        }
        p_bag = ptmp;
      }
      assert((name[found+1]) != NULL); // \ shouldn't be last character.
      return createPropertyBags(name.substr(found+1), p_bag);
    }
    else
    {
      if (head != NULL)
      {
        return head;
      }
      else
      {
        return this->properties();
      }
    }
  }

  void SimpleEnergySupply::cleanupPropertyBags(RTT::PropertyBag* p)
  {
    RTT::Property<PropertyBag>* ppb(NULL);

    for(RTT::PropertyBag::iterator it = p->begin(); it != p->end(); ++it)
    {
      ppb = dynamic_cast<RTT::Property<PropertyBag>*>(*it);
      if(ppb != NULL)
      {
        PropertyBag* ptmp = &(ppb->value());
        cleanupPropertyBags(ptmp);
        delete ptmp;
      }
    }
  }

  double SimpleEnergySupply::getTime()
  {
    return SimpleEnergySupplyModel::getTime();
  }
}

/* Macro to generate component library
 * Can be modified if the component is part of a big project with other components 
 */
ORO_CREATE_COMPONENT(motion_stack::SimpleEnergySupply)


