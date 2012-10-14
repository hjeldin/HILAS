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
