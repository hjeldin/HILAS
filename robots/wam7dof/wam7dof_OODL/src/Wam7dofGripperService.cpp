#include "Wam7dofGripperService.hpp"

namespace Wam7dof
{

  Wam7dofGripperService::Wam7dofGripperService(const string& name, TaskContext* parent):
  IRobotGripperService(name, parent,-1){}

  Wam7dofGripperService::~Wam7dofGripperService(){}

  void Wam7dofGripperService::displayGripperStatus()
  {
    log(Warning) << "Not implemented." << endlog();
  }

  void Wam7dofGripperService::setGripperSetpoints(){}

  bool Wam7dofGripperService::calibrate()
  {
    log(Info) << "Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void Wam7dofGripperService::checkForErrors(){}

  void Wam7dofGripperService::stop(){}

  void Wam7dofGripperService::cleanup()
  {
    m_calibrated = false;
  }
}
