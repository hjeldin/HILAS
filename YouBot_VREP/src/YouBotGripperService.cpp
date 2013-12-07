#include "YouBotGripperService.hpp"

#include <stdio.h>
#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>

#include "YouBotHelpers.hpp"

namespace YouBot
{
  using namespace RTT;
  using namespace RTT::types;
  using namespace std;

  YouBotGripperService::YouBotGripperService(const string& name,
      TaskContext* parent) :
      Service(name, parent),
      // Set the commands to zero depending on the number of joints
      m_calibrated(false)
  {

  //    m_gripper_state.name.assign(1, "");
  //    m_gripper_state.name[0] = "gripper";

  //    m_gripper_state.position.assign(1, 0);
  //    m_gripper_state.velocity.assign(0);
  //    m_gripper_state.effort.assign(0);

    // gripper_finger_l & gripper_finger_r
    m_gripper_cmd_position.positions.resize(2, 0);

    this->addPort("gripper_cmd_position", gripper_cmd_position).doc(
        "Command the gripper position");

    this->addOperation("start", &YouBotGripperService::start, this);
    this->addOperation("update", &YouBotGripperService::update, this);
    this->addOperation("calibrate", &YouBotGripperService::calibrate, this);
    this->addOperation("stop", &YouBotGripperService::stop, this);
    this->addOperation("cleanup", &YouBotGripperService::cleanup, this);

  //        this->addOperation("displayGripperStatus",&YouBotGripperService::displayGripperStatus,this, OwnThread);

  // Pre-allocate port memory for outputs
  //        gripper_state.setDataSample(m_gripper_state);
  }

  YouBotGripperService::~YouBotGripperService()
  {
    
  }

  void YouBotGripperService::displayGripperStatus()
  {
    log(Warning) << "[VREP] Not implemented." << endlog();
  }

  bool YouBotGripperService::start()
  {
    return m_calibrated;
  }

  void YouBotGripperService::update()
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
    return (m_calibrated = true);
  }

  void YouBotGripperService::checkForErrors()
  {
    log(Warning) << "checkForErrors - Not implemented" << endlog();
  }

  void YouBotGripperService::stop()
  {
  }

  void YouBotGripperService::cleanup()
  {
    m_calibrated = false;
  }

}
