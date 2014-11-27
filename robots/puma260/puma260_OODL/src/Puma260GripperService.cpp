#include "Puma260GripperService.hpp"

namespace Puma260
{

  Puma260GripperService::Puma260GripperService(const string& name, TaskContext* parent):
  IRobotGripperService(name, parent,-1){}

  Puma260GripperService::~Puma260GripperService(){}

  void Puma260GripperService::displayGripperStatus()
  {
    log(Warning) << "Not implemented." << endlog();
  }

  void Puma260GripperService::setGripperSetpoints(){}

  bool Puma260GripperService::calibrate()
  {
    log(Info) << "Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void Puma260GripperService::checkForErrors(){}

  void Puma260GripperService::stop(){}

  void Puma260GripperService::cleanup()
  {
    m_calibrated = false;
  }
}
