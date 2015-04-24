#include "Wam7dofGripperService.hpp"

namespace Wam7dof
{

  Wam7dofGripperService::Wam7dofGripperService(const string& name, TaskContext* parent, long i_clientID):
  IRobotGripperService(name, parent, i_clientID){}

  Wam7dofGripperService::~Wam7dofGripperService(){}

  void Wam7dofGripperService::displayGripperStatus()
  {
    log(Warning) << "[VREP] Not implemented." << endlog();
  }

  void Wam7dofGripperService::setGripperSetpoints()
  {
    // Update gripper setpoint
    if (gripper_cmd_position.read(m_gripper_cmd_position) == NewData) //setData has SLEEP_MILLISECOND :-(
    {
        //@todo send cmd to VREP
    }  
  }

  bool Wam7dofGripperService::calibrate()
  {
    log(Info) << "[VREP] Calibrating Wam7dofGripperService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "[VREP] Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "[VREP] Calibrated." << endlog();
    return (m_calibrated = true);  }

  void Wam7dofGripperService::checkForErrors(){}

  void Wam7dofGripperService::stop(){}

  void Wam7dofGripperService::cleanup()
  {
    m_calibrated = false;
  }
}
