#include "wam7dof_cmddemux-component.hpp"

namespace Wam7dof
{

Wam7dofCmdDemux::Wam7dofCmdDemux(std::string const& name):
Hilas::IRobotCmdDemux(name,"Wam7dof", Wam7dof::A_JOINT_ARM_NAME_ARRAY, NULL){}

Wam7dofCmdDemux::~Wam7dofCmdDemux(){}

void Wam7dofCmdDemux::updateHook()
{
    joint_velocities_in.read(m_joint_velocity_cmd);
    int id;

    for (int i = 0; i < arm_count; ++i)
    {
        for (int j = 0; j < arm_joint_count[i]; ++j)
        {
        	id = Wam7dof::JOINT_MAP[m_arm_velocity_cmd[i].names[j]];
            m_arm_velocity_cmd[i].velocities[j] = m_joint_velocity_cmd.velocities[id];
        }
        arm_velocity_command_out[i]->write(m_arm_velocity_cmd[i]);
    }
}

}

ORO_CREATE_COMPONENT(Wam7dof::Wam7dofCmdDemux)