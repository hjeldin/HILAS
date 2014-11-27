#include "Puma260GripperService.hpp"

namespace Puma260
{

  Puma260GripperService::Puma260GripperService(const string& name, TaskContext* parent, long clientID):
  IRobotGripperService(name, parent, clientID){}

  Puma260GripperService::~Puma260GripperService(){}

  void Puma260GripperService::displayGripperStatus()
  {
    log(Warning) << "[VREP] Not implemented." << endlog();
  }

  void Puma260GripperService::setGripperSetpoints()
  {
    // Update gripper setpoint
    if (gripper_cmd_position.read(m_gripper_cmd_position) == NewData) //setData has SLEEP_MILLISECOND :-(
    {
        //@todo send cmd to VREP
    }  
  }

  bool Puma260GripperService::calibrate()
  {
    log(Info) << "[VREP] Calibrating Puma260GripperService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "[VREP] Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "[VREP] Calibrated." << endlog();
    return (m_calibrated = true);  }

  void Puma260GripperService::checkForErrors(){}

  void Puma260GripperService::stop(){}

  void Puma260GripperService::cleanup()
  {
    m_calibrated = false;
  }
}
