#include "YouBotGripperService.hpp"

namespace YouBot
{

  YouBotGripperService::YouBotGripperService(const string& name, TaskContext* parent):
  IRobotGripperService(name, parent,-1){}

  YouBotGripperService::~YouBotGripperService()
  {
    delete m_manipulator;
  }

  void YouBotGripperService::displayGripperStatus()
  {
    log(Warning) << "Not implemented." << endlog();
  }

  void YouBotGripperService::setGripperSetpoints()
  {
    if (gripper_cmd_position.read(m_gripper_cmd_position) == NewData) //setData has SLEEP_MILLISECOND :-(
    {
      m_tmp_gripper_cmd_position.barSpacing = m_gripper_cmd_position.positions[0] * si::meter;
      m_gripper->setData(m_tmp_gripper_cmd_position);
    }    
  }

  bool YouBotGripperService::calibrate()
  {
    log(Info) << "Calibrating YouBotGripperService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "Already calibrated." << endlog();
      return m_calibrated;
    }

    try
    {
      m_manipulator = new YouBotManipulator("/youbot-manipulator", OODL_YOUBOT_CONFIG_DIR);
      if (m_manipulator == NULL)
      {
        log(Error) << "Could not create the YouBotManipulator." << endlog();
        return false;
      }

      // Gripper
      m_manipulator->calibrateGripper();
      try{
        m_gripper = &(m_manipulator->getArmGripper());
      }catch(std::exception & e)
      {
        log(Info) << "Could not calibrate gripper (goddammnit kuka)" << endlog();
      }

      log(Info) << "Gripper calibration min_position: " << m_gripper_limits.min_position << " max_position: " << m_gripper_limits.max_position << endlog();
    } catch (std::exception& e)
    {
      log(Error) << e.what() << endlog();
      return false;
    }

    log(Info) << "Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void YouBotGripperService::checkForErrors(){}

  void YouBotGripperService::stop(){}

  void YouBotGripperService::cleanup()
  {
    m_calibrated = false;
  }
}
