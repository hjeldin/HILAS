#include "IRobotGripperService.hpp"

namespace Hilas
{

IRobotGripperService::IRobotGripperService(const string& name, TaskContext* parent, long m_clientID = -1):
Service(name, parent), m_calibrated(false), clientID(m_clientID)
{
	m_gripper_cmd_position.positions.resize(1, 0);

	this->addPort("gripper_cmd_position_in", gripper_cmd_position).doc("Command the gripper position");

	this->addOperation("start", &IRobotGripperService::start, this);
	this->addOperation("update", &IRobotGripperService::update, this);
	this->addOperation("calibrate", &IRobotGripperService::calibrate, this);
	this->addOperation("stop", &IRobotGripperService::stop, this);
	this->addOperation("cleanup", &IRobotGripperService::cleanup, this);
	this->addOperation("setSimMode", &IRobotGripperService::setsim_mode,this).doc("Set simulation mode.");   
}

IRobotGripperService::~IRobotGripperService(){}

void IRobotGripperService::setsim_mode(int mode)
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

void IRobotGripperService::displayGripperStatus()
{
	std::cout << "Not yet implemented" << std::endl;
}

void IRobotGripperService::checkForErrors(){}  

bool IRobotGripperService::start()
{
	return m_calibrated;
}

void IRobotGripperService::setGripperSetpoints(){}

void IRobotGripperService::update()
{
	setGripperSetpoints();
	checkForErrors();
}

bool IRobotGripperService::calibrate()
{
	std::cout << "Not yet implemented, setting to true" << std::endl;

	m_calibrated = true;
	return true;  	
}

void IRobotGripperService::stop()
{
	std::cout << "Not yet implemented" << std::endl;  	
}

void IRobotGripperService::cleanup()
{
	std::cout << "Not yet implemented" << std::endl;
}

}