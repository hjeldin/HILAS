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

#include <rtt/OutputPort.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/internal/ConnectionManager.hpp>
#include <rtt/internal/ConnFactory.hpp>
#include <rtt/ConnPolicy.hpp>

#include <boost/tuple/tuple.hpp>

namespace YouBot
{
  using namespace RTT;

  class IConnectionMapping
  {
  public:
    virtual bool mapSourceToDest() = 0;
    virtual bool mapExecToDest() = 0;
    virtual bool isSourceToDestMapped() = 0;
    virtual ~IConnectionMapping() {}
  };

  template<typename T>
  class ConnectionMapping : public IConnectionMapping
  {
  private:
    RTT::InputPort<T>* exec_input_port; // Input port of the executive
    RTT::OutputPort<T>* exec_output_port; // Output port of the executive

    RTT::InputPort<T>* dest_input_port;
    RTT::OutputPort<T>* source_output_port;

    RTT::ConnPolicy source_policy;
    RTT::ConnPolicy exec_output_policy;

    bool is_input_stream, is_output_stream;

    bool source_to_dest_mapped;

    ConnectionMapping() {}

    bool getChannel(RTT::base::PortInterface* port, RTT::internal::ConnectionManager::ChannelDescriptor& channel)
    {
      if(port == NULL)
        return false;

      const internal::ConnectionManager* manager  = port->getManager();
      if(!manager->isSingleConnection())
      {
        log(Warning) << "Not a single connection, something might be wrong." << endlog();
      }

      std::list< RTT::internal::ConnectionManager::ChannelDescriptor > channels(manager->getChannels());
      if(channels.size() == 0)
      {
        log(Error) << "Executive input not connected." << endlog();
        return false; //No connection?
      }

      channel = channels.back();
      return true;
    }

    RTT::ConnPolicy getConnPolicy(RTT::internal::ConnectionManager::ChannelDescriptor& channel)
    {
      return ::boost::tuples::get<2>(channel);
    }

    RTT::OutputPort<T>* getCurrentSourcePort(RTT::internal::ConnectionManager::ChannelDescriptor& channel)
    {
      RTT::base::ChannelElementBase::shared_ptr other_end = ::boost::tuples::get<1>(channel)->getOutput();
      RTT::base::PortInterface* port = other_end->getPort();
      RTT::OutputPort<T>* outp_port = dynamic_cast<RTT::OutputPort<T>* >(port);
      if(outp_port == NULL)
        log(Error) << "Not an OutputPortInterface." << endlog();
      return outp_port;
    }

    RTT::InputPort<T>* getCurrentDestPort(RTT::internal::ConnectionManager::ChannelDescriptor& channel)
    {
      RTT::base::ChannelElementBase::shared_ptr other_end = ::boost::tuples::get<1>(channel)->getInput();
      RTT::base::PortInterface* port = other_end->getPort();
      RTT::InputPort<T>* inp_port = dynamic_cast<RTT::InputPort<T>* >(port);
      if(inp_port == NULL)
        log(Error) << "Not an InputPortInterface." << endlog();
      return inp_port;
    }

  public:

    ConnectionMapping(RTT::InputPort<T>* exec_input_port, RTT::OutputPort<T>* exec_output_port) :
      exec_input_port(exec_input_port), exec_output_port(exec_output_port),
      dest_input_port(NULL), source_output_port(NULL), is_input_stream(false), is_output_stream(false),
      source_to_dest_mapped(false)
    {
      RTT::internal::ConnectionManager::ChannelDescriptor channel;
      if(getChannel(exec_output_port, channel))
      {
        dest_input_port = getCurrentDestPort(channel);
        exec_output_policy = getConnPolicy(channel);

        is_output_stream = (exec_output_policy.transport != 0);
      }

      if(getChannel(exec_input_port, channel))
      {
        source_output_port = getCurrentSourcePort(channel);
        source_policy = getConnPolicy(channel);

        is_input_stream = (source_policy.transport != 0);
      }

    }

    virtual ~ConnectionMapping(){}

    bool mapSourceToDest()
    {
      if(!source_to_dest_mapped)
      {
        if(source_output_port == NULL || dest_input_port == NULL)
          return false; //cannot map

        exec_output_port->disconnect(); //fix me!
        internal::ConnFactory* fac = source_output_port->getConnFactory();
        if(is_input_stream)
        {
          return fac->createStream(*dest_input_port, source_policy);
        }
        else
        {
          return fac->createConnection(*source_output_port, *dest_input_port, source_policy);
        }
      }
      return true;
    }

    bool mapExecToDest()
    {
      if(source_to_dest_mapped)
      {
        if(exec_output_port == NULL || dest_input_port == NULL)
          return false; //cannot map

        source_output_port->disconnect(); //fix me!
        internal::ConnFactory* fac = exec_output_port->getConnFactory();
        if(is_output_stream)
        {
          return fac->createStream(*exec_output_port, exec_output_policy);
        }
        else
        {
          return fac->createConnection(*exec_output_port, *dest_input_port, exec_output_policy);
        }
      }
      return true;
    }

    bool isSourceToDestMapped()
    {
      return source_to_dest_mapped;
    }
  };
}
