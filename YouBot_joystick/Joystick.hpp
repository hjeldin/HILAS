#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>

#include <sensor_msgs/typekit/Types.hpp>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

	using namespace RTT;
	using namespace std;
class JoystickControl: public TaskContext
{
  public:
    JoystickControl(const string& name);
    virtual ~JoystickControl();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:
    InputPort<sensor_msgs::Joy> joystick;
    InputPort< nav_msgs::Odometry > odometry_state;
    OutputPort< geometry_msgs::Twist > cmd_twist;

    nav_msgs::Odometry m_odometry_state;
    geometry_msgs::Twist m_cmd_twist;
    sensor_msgs::Joy m_joystick;

    double m_K[3];
    double m_error[3];
};
