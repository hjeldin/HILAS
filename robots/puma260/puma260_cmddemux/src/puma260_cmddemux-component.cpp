#include "puma260_cmddemux-component.hpp"

namespace Puma260
{

Puma260CmdDemux::Puma260CmdDemux(std::string const& name):
Hilas::IRobotCmdDemux(name,"puma260", Puma260::A_JOINT_ARM_NAME_ARRAY, NULL){}

Puma260CmdDemux::~Puma260CmdDemux(){}

void Puma260CmdDemux::updateHook()
{
    joint_velocities_in.read(m_joint_velocity_cmd);
    int id;

    for (int i = 0; i < arm_count; ++i)
    {
        for (int j = 0; j < arm_joint_count[i]; ++j)
        {
        	id = Puma260::JOINT_MAP[m_arm_velocity_cmd[i].names[j]];
            m_arm_velocity_cmd[i].velocities[j] = m_joint_velocity_cmd.velocities[id];
        }
        arm_velocity_command_out[i]->write(m_arm_velocity_cmd[i]);
    }
}

}

ORO_CREATE_COMPONENT(Puma260::Puma260CmdDemux)