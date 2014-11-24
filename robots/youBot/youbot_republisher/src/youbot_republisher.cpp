#include "youbot_republisher.hpp"

namespace YouBot
{

using namespace RTT;

YouBotStateRepublisher::YouBotStateRepublisher(std::string const& name):
Hilas::IRobotStateRepublisher(name,"youBot",SIZE_JOINT_NAME_ARRAY,JOINT_NAME_ARRAY){}

YouBotStateRepublisher::~YouBotStateRepublisher(){}

void YouBotStateRepublisher::updateHook()
{
	m_robot_state.header.stamp = ros::Time(RTT::os::TimeService::Instance()->getNSecs() * 1e9, RTT::os::TimeService::Instance()->getNSecs());

	if(arm_state_in[0]->read(m_arm_state) == NewData)
	{
		m_robot_state.position[0] = m_arm_state.position[0];
		m_robot_state.position[1] = m_arm_state.position[1];
		m_robot_state.position[2] = m_arm_state.position[2];
		m_robot_state.position[3] = m_arm_state.position[3];
		m_robot_state.position[4] = m_arm_state.position[4];
	}
	if(base_state_in[0]->read(m_base_state) == NewData)
	{
		m_robot_state.position[5] = m_base_state.position[0];
		m_robot_state.position[6] = m_base_state.position[1];
		m_robot_state.position[7] = m_base_state.position[2];
		m_robot_state.position[8] = m_base_state.position[3];
	}

	odometry_state_in.read(m_odometry_state);

	// Gripper cannot be read in real-time (yet)
	m_robot_state.position[13] = 0.001;
	m_robot_state.position[14] = 0.001;

	robot_state_out.write(m_robot_state);
	odometry_state_out.write(m_odometry_state);
}

}

ORO_CREATE_COMPONENT(YouBot::YouBotStateRepublisher)