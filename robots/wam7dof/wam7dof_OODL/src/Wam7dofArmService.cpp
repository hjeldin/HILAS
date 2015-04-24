#include "Wam7dofArmService.hpp"


namespace Wam7dof
{
extern unsigned int non_errors;

Wam7dofArmService::Wam7dofArmService(const string& name, TaskContext* parent):
Hilas::IRobotArmService(name, parent, Wam7dof::NR_OF_ARM_SLAVES, 6, Wam7dof::JOINT_ARM_NAME_ARRAY,-1), m_min_slave_nr(1){}

Wam7dofArmService::~Wam7dofArmService(){}

void Wam7dofArmService::displayMotorStatuses(){}

void Wam7dofArmService::clearControllerTimeouts(){}

void Wam7dofArmService::readJointStates(){}

void Wam7dofArmService::updateJointSetpoints(){}

void Wam7dofArmService::checkMotorStatuses(){}

bool Wam7dofArmService::calibrate()
{
	log(Info) << "Calibrated." << endlog();
	return (m_calibrated = true);
}

void Wam7dofArmService::stop(){}

void Wam7dofArmService::cleanup()
{
	m_calibrated = false;
}

}
