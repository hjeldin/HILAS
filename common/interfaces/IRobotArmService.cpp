#include "IRobotArmService.hpp"

namespace Hilas
{

IRobotArmService::IRobotArmService(const string& name, TaskContext* parent, int arm_slave_count, int robot_slave_count, const std::string arm_slave_name[], long m_clientID = -1):
Service(name, parent), NR_OF_ARM_SLAVES(arm_slave_count), m_joint_limits(arm_slave_count), m_joint_ctrl_modes(arm_slave_count, MOTOR_STOP), m_calibrated(false), clientID(m_clientID)
{
	for (int i = 0; i < arm_slave_count; ++i)
	{
		m_joint_state.name.push_back(arm_slave_name[i]);
	}

	m_joint_state.position.assign(NR_OF_ARM_SLAVES, 0);
	m_joint_state.velocity.assign(NR_OF_ARM_SLAVES, 0);
	m_joint_state.effort.assign(NR_OF_ARM_SLAVES, 0);

	m_joint_position_command.positions.assign(robot_slave_count, 0);
	m_joint_velocity_command.velocities.assign(robot_slave_count, 0);
	m_joint_effort_command.efforts.assign(robot_slave_count, 0);

    // Pre-allocate port memory for outputs
	joint_state.setDataSample(m_joint_state);

    // Events - Pre-allocate port memory for outputs
	m_events.reserve(max_event_length);
	events.setDataSample(m_events);

	setupComponentInterface();
}

IRobotArmService::~IRobotArmService(){}

void IRobotArmService::setupComponentInterface()
{
	this->addPort("joint_state_out", joint_state).doc("Joint states");

	this->addPort("joint_position_command_in", joint_position_command).doc("Command joint angles");
	this->addPort("joint_velocity_command_in", joint_velocity_command).doc("Command joint velocities");
	this->addPort("joint_effort_command_in", joint_effort_command).doc("Command joint torques");

	this->addPort("joint_state_in", in_joint_state).doc("Joint states");

	this->addPort("events", events).doc("Joint events");

	this->addOperation("start", &IRobotArmService::start, this);
	this->addOperation("update", &IRobotArmService::update, this);
	this->addOperation("calibrate", &IRobotArmService::calibrate, this);
	this->addOperation("stop", &IRobotArmService::stop, this);
	this->addOperation("cleanup", &IRobotArmService::cleanup, this);

	this->addOperation("setControlModesAll", &IRobotArmService::setControlModesAll, this, OwnThread).doc("Control modes can be set individually.");
	this->addOperation("setControlModes", &IRobotArmService::setControlModes, this, OwnThread).doc("Control modes can be set individually.");
	this->addOperation("getControlModes", &IRobotArmService::getControlModes, this, OwnThread).doc("Control modes are individual.");
	this->addOperation("displayMotorStatuses", &IRobotArmService::displayMotorStatuses, this, OwnThread);
	this->addOperation("clearControllerTimeouts", &IRobotArmService::clearControllerTimeouts, this, OwnThread);
	this->addOperation("setSimMode", &IRobotArmService::setsim_mode,this, OwnThread).doc("Set simulation mode.");    

}

void IRobotArmService::setsim_mode(int mode)
{
	switch(mode)
	{
		case 1:
	    	is_in_visualization_mode = true;
	  	break;
	  
	  	case 2:
	    	is_in_visualization_mode = false;
	  	break;
	  
	  	default: break;
	}
}

void IRobotArmService::setControlModes(vector<ctrl_modes>& all)
{
	m_joint_ctrl_modes = all;
}

void IRobotArmService::setControlModesAll(int mode)
{
	for(int i=0; i<NR_OF_ARM_SLAVES; ++i)
	{
		m_joint_ctrl_modes[i] = static_cast<ctrl_modes>(mode);
	}
}

void IRobotArmService::getControlModes(vector<ctrl_modes>& all)
{
	all = m_joint_ctrl_modes;
}

void IRobotArmService::displayMotorStatuses()
{
	std::cout << "Not yet implemented" << std::endl;
}

void IRobotArmService::clearControllerTimeouts()
{
	std::cout << "Not yet implemented" << std::endl;
}

void IRobotArmService::checkMotorStatuses(){}

bool IRobotArmService::start()
{
	if(m_calibrated)
	{
		clearControllerTimeouts();
		return true;
	}
	else
	{
		return false;
	}
}

void IRobotArmService::update()
{
	readJointStates();
	updateJointSetpoints();
	checkMotorStatuses();
}

bool IRobotArmService::calibrate()
{
	std::cout << "Not yet implemented, setting to true" << std::endl;

	m_calibrated = true;
	return true;
}

void IRobotArmService::stop()
{
	std::cout << "Not yet implemented" << std::endl;
}

void IRobotArmService::cleanup()
{
	std::cout << "Not yet implemented" << std::endl;
}

}