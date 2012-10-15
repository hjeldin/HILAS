#include "YouBotStateRepublisher.hpp"

/*
 * @brief Adapter to connect 20Sim to the YouBot OODL.
 */
namespace YouBot
{
using namespace RTT;
const size_t max_event_length = 255;
std::string& make_edge_event(std::string& s, const std::string& event, bool status)
 {
     char tmpstr[max_event_length];
     if(s.capacity() < max_event_length)
         log(Error) << "make_event: event string capacity < max_event_length." << endlog();

     if(status == true)
       snprintf(tmpstr, max_event_length, "%s,true", event.c_str());
     else
       snprintf(tmpstr, max_event_length, "%s,false", event.c_str());

     s.insert(0, tmpstr, max_event_length);
     return s;
 }
YouBotStateRepublisher::YouBotStateRepublisher(std::string const& name) :
				TaskContext(name, PreOperational), m_dimension(0), wheel(0)
{
	this->addPort("arm_state", arm_state);
	this->addPort("base_state", base_state);
	this->addPort("youbot_state", youbot_state);
	//Energy state republishing
	this->addPort("arm_energy_tank",arm_energy_tank);
	this->addPort("base_energy_tank",base_energy_tank);
	this->addPort("kinematics_energy_tank",kinematics_energy_tank);
	this->addPort("events",events);



	m_youbot_state.position.resize(SIZE_JOINT_NAME_ARRAY, 0.0);
	m_youbot_state.name.assign(JOINT_NAME_ARRAY,JOINT_NAME_ARRAY+SIZE_JOINT_NAME_ARRAY);
	youbot_state.setDataSample(m_youbot_state);
}

YouBotStateRepublisher::~YouBotStateRepublisher()
{

}

bool YouBotStateRepublisher::configureHook()
{

	if (!arm_state.connected())
	{
		log(Warning) << "The port arm_state is not connected" << endlog();
	}
	else if(arm_state.read(m_arm_state) != NoData && m_arm_state.position.size() != 5)
	{
	  log(Warning) << "The port arm_state does not have the right dimension." << endlog();
	}

	if (!base_state.connected())
	{
		log(Warning) << "The port base_state is not connected" << endlog();
	}
	else if(base_state.read(m_base_state) != NoData && m_base_state.position.size() != 4)
	{
		log(Warning) << "The port base_state does not have the right dimension." << endlog();
	}

	if (!arm_energy_tank.connected())
	{
		log(Warning) << "The port arm_energy_tank is not connected" << endlog();
	}

	if (!base_energy_tank.connected())
	{
		log(Warning) << "The port base_energy_tank is not connected" << endlog();
	}

	if (!kinematics_energy_tank.connected())
	{
		log(Warning) << "The port kinematics_energy_tank is not connected" << endlog();
	}

	if (!events.connected())
	{
		log(Warning) << "The port events is not connected" << endlog();
	}

	return TaskContext::configureHook();
}
bool YouBotStateRepublisher::startHook()
{
	return TaskContext::startHook();
}

void YouBotStateRepublisher::updateHook()
{
  m_youbot_state.header.stamp = ros::Time::now();
  if (arm_state.read(m_arm_state) == NewData)
  {
    m_youbot_state.position[0] = m_arm_state.position[0];
    m_youbot_state.position[1] = m_arm_state.position[1];
    m_youbot_state.position[2] = m_arm_state.position[2];
    m_youbot_state.position[3] = m_arm_state.position[3];
    m_youbot_state.position[4] = m_arm_state.position[4];
  }
  if (base_state.read(m_base_state) == NewData)
  {
    m_youbot_state.position[5] = m_base_state.position[0];
    m_youbot_state.position[6] = m_base_state.position[1];
    m_youbot_state.position[7] = m_base_state.position[2];
    m_youbot_state.position[8] = m_base_state.position[3];
  }

  // casters remain 0

  // Gripper cannot be read in real-time (yet)
  m_youbot_state.position[13] = 0.001;
  m_youbot_state.position[14] = 0.001;

  youbot_state.write(m_youbot_state);

  if(arm_energy_tank.read(m_energy_tank) == NewData)
  {
	  if (m_energy_tank.data[0]<MIN_ENERGY && m_arm_energy_tank > MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",true));
	  }
	  if (m_energy_tank.data[0]>MIN_ENERGY && m_arm_energy_tank < MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",false));
	  }
	  m_arm_energy_tank=m_energy_tank.data[0];
  }
  if(base_energy_tank.read(m_energy_tank) == NewData)
  {
	  if (m_energy_tank.data[0]<MIN_ENERGY && m_base_energy_tank > MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",true));
	  }
	  if (m_energy_tank.data[0]>MIN_ENERGY && m_base_energy_tank < MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",false));
	  }
	  m_base_energy_tank=m_energy_tank.data[0];
  }
  if(kinematics_energy_tank.read(m_energy_tank) == NewData)
  {
	  if (m_energy_tank.data[0]<MIN_ENERGY && m_kinematics_energy_tank > MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",true));
	  }
	  if (m_energy_tank.data[0]>MIN_ENERGY && m_kinematics_energy_tank < MIN_ENERGY )
	  {
		  events.write(make_edge_event(m_events,"energytank.LOW",false));
	  }
	  m_kinematics_energy_tank=m_energy_tank.data[0];
  }
	TaskContext::updateHook();
}

}


ORO_CREATE_COMPONENT( YouBot::YouBotStateRepublisher)
