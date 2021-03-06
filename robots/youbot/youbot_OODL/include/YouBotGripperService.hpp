#pragma once

#include <IRobotGripperService.hpp>

#include <youbot/YouBotManipulator.hpp>
#include <youbot/ProtocolDefinitions.hpp>

#include <youBot.hpp>

namespace YouBot
{

  using namespace RTT;
  using namespace RTT::types;  
  using namespace std;
  using namespace youbot;
  using namespace boost::units;
  using namespace boost::units::si;

  class YouBotGripperService: public Hilas::IRobotGripperService
  {

    public:

      YouBotGripperService(const string& name, TaskContext* parent);
      ~YouBotGripperService();

      void displayGripperStatus();

    private:

      bool calibrate();
      void cleanup();
      void stop();
      void checkForErrors();
      void setGripperSetpoints();

      YouBotManipulator* m_manipulator;
      YouBotGripper* m_gripper;
      GripperBarSpacingSetPoint m_tmp_gripper_cmd_position;
  };

}
