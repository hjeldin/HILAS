#include "TransformToTSim.hpp"

#include <iostream>
#include <ocl/Component.hpp>
#include <tf/tf.h>

using namespace RTT;

TransformToTSim::TransformToTSim(string const& name) :
    TaskContext(name)
{
  this->addPort("output_transform",output_transform)
    .doc("Connect 20Sim flat vectors.");

  this->addEventPort("input_transform",input_transform)
    .doc("Connect geometry_msgs::TransformStamped messages. This is an event port.");

  m_output_transform.data.resize(16, 0.0);
  output_transform.setDataSample(m_output_transform);
}

TransformToTSim::~TransformToTSim() {}

bool TransformToTSim::startHook()
{
  if(!( output_transform.connected() ) )
  {
    log(Error) << "The output ports needs to be connected." << endlog();
    return false;
  }

  if(! input_transform.connected() )
  {
    log(Error) << "The input_transform needs to be connected." << endlog();
    return false;
  }

  return TaskContext::startHook();
}

void TransformToTSim::updateHook()
{
  TaskContext::updateHook();
  if(input_transform.read(m_input_transform) == NewData)
  {
    tf::Quaternion quat(m_input_transform.transform.rotation.x, m_input_transform.transform.rotation.y, m_input_transform.transform.rotation.z, m_input_transform.transform.rotation.w);
    btMatrix3x3 rotMatrix;
    rotMatrix.setRotation(quat);
    memcpy(&m_output_transform.data[0], &rotMatrix[0], 3*sizeof(double));
    memcpy(&m_output_transform.data[4], &rotMatrix[1], 3*sizeof(double));
    memcpy(&m_output_transform.data[8], &rotMatrix[2], 3*sizeof(double));
    m_output_transform.data[3] = m_input_transform.transform.translation.x;
    m_output_transform.data[7] = m_input_transform.transform.translation.y;
    m_output_transform.data[11] = m_input_transform.transform.translation.z;
    m_output_transform.data[15] = 1;
    memset(&m_output_transform.data[12], 0.0, 3*sizeof(double));
    output_transform.write(m_output_transform);
  }
}


ORO_CREATE_COMPONENT(TransformToTSim)
