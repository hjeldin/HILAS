#include "puma260_republisher-component.hpp"

namespace Puma260
{

Puma260StateRepublisher::Puma260StateRepublisher(std::string const& name):
Hilas::IRobotStateRepublisher(name,"puma260",Puma260::SIZE_JOINT_NAME_ARRAY,Puma260::JOINT_NAME_ARRAY){}

Puma260StateRepublisher::~Puma260StateRepublisher(){}

void Puma260StateRepublisher::updateHook()
{
	m_robot_state.header.stamp = ros::Time(RTT::os::TimeService::Instance()->getNSecs() * 1e9, RTT::os::TimeService::Instance()->getNSecs());

	if(arm_state_in[0]->read(m_arm_state) == NewData)
	{
		m_robot_state.position[0] = m_arm_state.position[0];
		m_robot_state.position[1] = m_arm_state.position[1];
		m_robot_state.position[2] = m_arm_state.position[2];
		m_robot_state.position[3] = m_arm_state.position[3];
		m_robot_state.position[4] = m_arm_state.position[4];
		m_robot_state.position[5] = m_arm_state.position[5];

		m_robot_state.velocity[0] = m_arm_state.velocity[0];
		m_robot_state.velocity[1] = m_arm_state.velocity[1];
		m_robot_state.velocity[2] = m_arm_state.velocity[2];
		m_robot_state.velocity[3] = m_arm_state.velocity[3];
		m_robot_state.velocity[4] = m_arm_state.velocity[4];
		m_robot_state.velocity[5] = m_arm_state.velocity[5];

		m_robot_state.effort[0] = m_arm_state.effort[0];
		m_robot_state.effort[1] = m_arm_state.effort[1];
		m_robot_state.effort[2] = m_arm_state.effort[2];
		m_robot_state.effort[3] = m_arm_state.effort[3];
		m_robot_state.effort[4] = m_arm_state.effort[4];
		m_robot_state.effort[5] = m_arm_state.effort[5];
	}

	robot_state_out.write(m_robot_state);
}

}
ORO_CREATE_COMPONENT(Puma260::Puma260StateRepublisher)