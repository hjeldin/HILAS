#include "wam7dof_republisher-component.hpp"

namespace Wam7dof
{

Wam7dofStateRepublisher::Wam7dofStateRepublisher(std::string const& name):
Hilas::IRobotStateRepublisher(name,"Wam7dof",Wam7dof::SIZE_JOINT_NAME_ARRAY,Wam7dof::JOINT_NAME_ARRAY){}

Wam7dofStateRepublisher::~Wam7dofStateRepublisher(){}

void Wam7dofStateRepublisher::updateHook()
{
	m_robot_state.header.stamp = ros::Time(RTT::os::TimeService::Instance()->getNSecs() * 1e9, RTT::os::TimeService::Instance()->getNSecs());

	if(arm_state_in[0]->read(m_arm_state) == NewData)
	{
		for(int i = 0; i<m_robot_state.position.size(); i++)
		{
			m_robot_state.position[i] = m_arm_state.position[i];
		}

		for(int i = 0; i<m_robot_state.velocity.size(); i++)
		{
			m_robot_state.velocity[i] = m_arm_state.velocity[i];
		}

		for(int i = 0; i<m_robot_state.effort.size(); i++)
		{
			m_robot_state.effort[i] = m_arm_state.effort[i];
		}
		
	}

	robot_state_out.write(m_robot_state);
}

}
ORO_CREATE_COMPONENT(Wam7dof::Wam7dofStateRepublisher)