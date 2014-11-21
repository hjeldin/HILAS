#include "IRobotStateRepublisher.hpp"

namespace Hilas
{

IRobotStateRepublisher::IRobotStateRepublisher(std::string const& name, std::string robot_name, int SIZE_JOINT_NAME_ARRAY, const std::string* JOINT_NAME_ARRAY):
TaskContext(name, PreOperational)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/"+robot_name+".ini", pt);
    arm_count = pt.get<int>("robot.armCount");
    base_count = pt.get<int>("robot.baseCount");

    arm_state_in.resize(arm_count);
    base_state_in.resize(base_count);

    for (int i = 0; i < arm_count; ++i)
    {
    	arm_state_in[i] = new InputPort<sensor_msgs::JointState>();
		this->addPort(std::string("arm_state_") + boost::lexical_cast<std::string>(i+1) + std::string("_in"), *arm_state_in[i]);
    }

    for (int i = 0; i < base_count; ++i)
    {
    	base_state_in[i] = new InputPort<sensor_msgs::JointState>();
		this->addPort(std::string("base_state_") +  boost::lexical_cast<std::string>(i+1) + std::string("_in"), *base_state_in[i]);
    }
    this->addPort("odometry_state_in",odometry_state_in);    

	this->addPort("robot_state_out", robot_state_out);
	this->addPort("events_out",events_out);
  	this->addPort("odometry_state_out",odometry_state_out);

	m_robot_state.position.resize(SIZE_JOINT_NAME_ARRAY, 0.0);
	m_robot_state.name.assign(JOINT_NAME_ARRAY, JOINT_NAME_ARRAY + SIZE_JOINT_NAME_ARRAY);
	robot_state_out.setDataSample(m_robot_state);
}

IRobotStateRepublisher::~IRobotStateRepublisher(){}

bool IRobotStateRepublisher::configureHook()
{
	return true;
}

bool IRobotStateRepublisher::startHook()
{
	return true;
}

void IRobotStateRepublisher::stopHook(){}

void IRobotStateRepublisher::cleanupHook(){}

}