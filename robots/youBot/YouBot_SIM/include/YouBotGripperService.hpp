#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <youbot/YouBotManipulator.hpp>

#include "YouBotSIM.hpp"

#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
  using namespace RTT;
  using namespace std;
  using namespace youbot;
  using namespace boost::units;
  using namespace boost::units::si;

  /**
   * @brief Part of the workaround to prevent the generation of Exceptions in OODL.
   */
  struct _GripperLimits
  {
      quantity<si::length> min_position;
      quantity<si::length> max_position;

      _GripperLimits() :
          min_position(0 * meter), max_position(0 * meter)
      {
      }
  };
  typedef struct _GripperLimits GripperLimits;

  class YouBotGripperService: public Service
  {

    public:
      YouBotGripperService(const string& name, TaskContext* parent, long i_clientID);
      virtual ~YouBotGripperService();

      void displayGripperStatus();
      void setsim_mode(int mode);
    protected:
      // Gripper
      InputPort<motion_control_msgs::JointPositions> gripper_cmd_position;

    private:
      bool calibrate();
      bool start();
      void update();
      void cleanup();
      void stop();

      void checkForErrors();

      long m_clientID;
      bool is_in_visualization_mode;
      
      // Gripper
      motion_control_msgs::JointPositions m_gripper_cmd_position;
      bool m_calibrated;
  };

}
