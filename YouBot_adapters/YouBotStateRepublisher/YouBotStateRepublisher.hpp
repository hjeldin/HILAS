#pragma once

#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <ocl/Component.hpp>

namespace YouBot
{
  using namespace RTT;
  using namespace std;

  static const string JOINT_NAME_ARRAY[] =
  //{"j0","j1","j2","j3","j4","w1","w2","w3","w4","f1","f2"};
  { "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5",
      "wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br",
      "caster_joint_fl","caster_joint_fr", "caster_joint_bl", "caster_joint_br","f1", "f2" };

  static const unsigned int SIZE_JOINT_NAME_ARRAY=15;


  class YouBotStateRepublisher: public RTT::TaskContext
  {
  public:
    YouBotStateRepublisher(std::string const& name);
    ~YouBotStateRepublisher();

    bool startHook();
    void updateHook();
    bool configureHook();

  private:

    InputPort<sensor_msgs::JointState> arm_state;
    InputPort<sensor_msgs::JointState> base_state;
    sensor_msgs::JointState m_arm_state;
    sensor_msgs::JointState m_base_state;
  //	InputPort<flat_matrix_t> odometry;

    OutputPort<sensor_msgs::JointState> youbot_state;
    sensor_msgs::JointState m_youbot_state;

    unsigned int m_dimension;
    double wheel;
  };

}

