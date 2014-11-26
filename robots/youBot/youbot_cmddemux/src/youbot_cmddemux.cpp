#include "youbot_cmddemux.hpp"

namespace YouBot
{

YouBotCmdDemux::YouBotCmdDemux(std::string const& name):
Hilas::IRobotCmdDemux(name,"youBot", YouBot::A_JOINT_ARM_NAME_ARRAY, YouBot::A_JOINT_BASE_NAME_ARRAY){}

YouBotCmdDemux::~YouBotCmdDemux(){}

void YouBotCmdDemux::updateHook()
{
    joint_velocities_in.read(m_joint_velocity_cmd);
    int id;

    for (int i = 0; i < arm_count; ++i)
    {
        for (int j = 0; j < arm_joint_count[i]; ++j)
        {
        	id = YouBot::JOINT_MAP[m_arm_velocity_cmd[i].names[j]];
            m_arm_velocity_cmd[i].velocities[j] = m_joint_velocity_cmd.velocities[id];
        }
        arm_velocity_command_out[i]->write(m_arm_velocity_cmd[i]);
    }

    for (int i = 0; i < base_count; ++i)
    {
        for (int j = 0; j < base_joint_count[i]; ++j)
        {
        	id = YouBot::JOINT_MAP[m_base_velocity_cmd[i].names[j]];
            m_base_velocity_cmd[i].velocities[j] = m_joint_velocity_cmd.velocities[id];
        }
        base_velocity_command_out[i]->write(m_base_velocity_cmd[i]);
    }
}

}

ORO_CREATE_COMPONENT(YouBot::YouBotCmdDemux)