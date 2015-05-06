#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

#include <stdio.h>
#include <cassert>

#include <hilas.hpp>
#include <IRobot.hpp>

namespace Hilas
{

  using namespace RTT;
  using namespace RTT::types;
  using namespace std;

  class IRobotGripperService: public Service
  {

    public:

      IRobotGripperService(const string& name, TaskContext* parent, long m_clientID);
      virtual ~IRobotGripperService();

      virtual void displayGripperStatus();
      void setsim_mode(int mode);

    protected:

      InputPort<motion_control_msgs::JointPositions> gripper_cmd_position;

      bool start();
      void update();
      virtual void cleanup();
      virtual void stop();

      virtual bool calibrate();
      virtual void checkForErrors();
      virtual void setGripperSetpoints() = 0;

      motion_control_msgs::JointPositions m_gripper_cmd_position;
      GripperJointLimits m_gripper_limits;

      long clientID;
      bool is_in_visualization_mode;
      bool m_calibrated;
  };

}