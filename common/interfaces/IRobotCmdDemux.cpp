#include "IRobotCmdDemux.hpp"

namespace Hilas
{

IRobotCmdDemux::IRobotCmdDemux(std::string const& name, std::string robot_name, const string* ARM_JOINT_NAME_ARRAY[], const string* BASE_JOINT_NAME_ARRAY[]):
TaskContext(name, PreOperational)
{
    boost::property_tree::ptree pt;
    std::transform(robot_name.begin(), robot_name.end(), robot_name.begin(), ::tolower);
    boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/"+robot_name+".ini", pt);
    arm_count = pt.get<int>("robot.armCount");
    base_count = pt.get<int>("robot.baseCount");

    arm_velocity_command_out.resize(arm_count);
    base_velocity_command_out.resize(base_count);

    arm_joint_count.resize(arm_count);
    base_joint_count.resize(base_count);

    m_arm_velocity_cmd.resize(arm_count);
    m_base_velocity_cmd.resize(base_count);

    for (int i = 0; i < arm_count; ++i)
    {
        arm_joint_count[i] = pt.get<int>(std::string("robot.arm")+boost::lexical_cast<std::string>(i+1)+std::string("JointCount"));
    	arm_velocity_command_out[i] = new OutputPort<motion_control_msgs::JointVelocities>();
		this->addPort(std::string("arm_velocity_command_") + boost::lexical_cast<std::string>(i+1) + std::string("_out"), *arm_velocity_command_out[i]);

        m_arm_velocity_cmd[i].names.assign(ARM_JOINT_NAME_ARRAY[i], ARM_JOINT_NAME_ARRAY[i] + arm_joint_count[i]);
        m_arm_velocity_cmd[i].velocities.assign(arm_joint_count[i], 0.0);
        arm_velocity_command_out[i]->setDataSample(m_arm_velocity_cmd[i]);
    }

    for (int i = 0; i < base_count; ++i)
    {
        base_joint_count[i] = pt.get<int>(std::string("robot.base")+boost::lexical_cast<std::string>(i+1)+std::string("JointCount"));
    	base_velocity_command_out[i] = new OutputPort<motion_control_msgs::JointVelocities>();
		this->addPort(std::string("base_velocity_command_") +  boost::lexical_cast<std::string>(i+1) + std::string("_out"), *base_velocity_command_out[i]);

        m_base_velocity_cmd[i].names.assign(BASE_JOINT_NAME_ARRAY[i], BASE_JOINT_NAME_ARRAY[i] + base_joint_count[i]);
        m_base_velocity_cmd[i].velocities.assign(base_joint_count[i], 0.0);
        base_velocity_command_out[i]->setDataSample(m_base_velocity_cmd[i]);
    }

    this->addPort("joint_velocity_command_in",joint_velocities_in);    
}

IRobotCmdDemux::~IRobotCmdDemux(){}

bool IRobotCmdDemux::configureHook()
{
	return true;
}

bool IRobotCmdDemux::startHook()
{
	return true;
}

void IRobotCmdDemux::stopHook(){}

void IRobotCmdDemux::cleanupHook(){}

}