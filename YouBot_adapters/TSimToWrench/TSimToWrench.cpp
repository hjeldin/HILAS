#include "TSimToWrench.hpp"

#include <iostream>
#include <ocl/Component.hpp>
#include <tf/tf.h>

using namespace RTT;

TSimToWrench::TSimToWrench(string const& name) :
    TaskContext(name)
{
  this->addEventPort("input_wrench",input_wrench)
    .doc("Connect 20Sim flat vectors. This is an event port.");

  this->addPort("output_wrench",output_wrench)
    .doc("Connect geometry_msgs::Wrench messages.");

  m_input_wrench.data.resize(6, 0.0);
  output_wrench.setDataSample(m_output_wrench);
}

TSimToWrench::~TSimToWrench() {}

bool TSimToWrench::startHook()
{
  if(!( output_wrench.connected() ) )
  {
    log(Error) << "The output ports needs to be connected." << endlog();
    return false;
  }

  if(! input_wrench.connected() )
  {
    log(Error) << "The input_wrench needs to be connected." << endlog();
    return false;
  }

  return TaskContext::startHook();
}

void TSimToWrench::updateHook()
{
  TaskContext::updateHook();
  if(input_wrench.read(m_input_wrench) == NewData)
  {
    m_output_wrench.torque.x = m_input_wrench.data[0];
    m_output_wrench.torque.y = m_input_wrench.data[1];
    m_output_wrench.torque.z = m_input_wrench.data[2];
    m_output_wrench.force.x = m_input_wrench.data[3];
    m_output_wrench.force.y = m_input_wrench.data[4];
    m_output_wrench.force.z = m_input_wrench.data[5];
    output_wrench.write(m_output_wrench);
  }
}


ORO_CREATE_COMPONENT(TSimToWrench)
