#include "Puma260ArmService.hpp"


namespace Puma260
{
extern unsigned int non_errors;

Puma260ArmService::Puma260ArmService(const string& name, TaskContext* parent):
Hilas::IRobotArmService(name, parent, Puma260::NR_OF_ARM_SLAVES, 6, Puma260::JOINT_ARM_NAME_ARRAY,-1), m_min_slave_nr(1){}

Puma260ArmService::~Puma260ArmService(){}

void Puma260ArmService::displayMotorStatuses(){}

void Puma260ArmService::clearControllerTimeouts(){}

void Puma260ArmService::readJointStates(){}

void Puma260ArmService::updateJointSetpoints(){}

void Puma260ArmService::checkMotorStatuses(){}

bool Puma260ArmService::calibrate()
{
	log(Info) << "Calibrated." << endlog();
	return (m_calibrated = true);
}

void Puma260ArmService::stop(){}

void Puma260ArmService::cleanup()
{
	m_calibrated = false;
}

}
