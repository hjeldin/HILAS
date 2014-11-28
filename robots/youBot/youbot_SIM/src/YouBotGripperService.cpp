#include "YouBotGripperService.hpp"

namespace YouBot
{

  YouBotGripperService::YouBotGripperService(const string& name, TaskContext* parent, long i_clientID):
  IRobotGripperService(name, parent, i_clientID){}

  YouBotGripperService::~YouBotGripperService(){}

  void YouBotGripperService::displayGripperStatus()
  {
    log(Warning) << "[VREP] Not implemented." << endlog();
  }

  void YouBotGripperService::setGripperSetpoints()
  {
    // Update gripper setpoint
    if (gripper_cmd_position.read(m_gripper_cmd_position) == NewData) //setData has SLEEP_MILLISECOND :-(
    {
        //@todo send cmd to VREP
    }  
  }

  bool YouBotGripperService::calibrate()
  {
    log(Info) << "[VREP] Calibrating YouBotGripperService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "[VREP] Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "[VREP] Calibrated." << endlog();
    return (m_calibrated = true);  }

  void YouBotGripperService::checkForErrors(){}

  void YouBotGripperService::stop(){}

  void YouBotGripperService::cleanup()
  {
    m_calibrated = false;
  }
}
