#pragma once

#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <ocl/Component.hpp>
#include <std_msgs/Float64MultiArray.h>

namespace YouBot
{
  using namespace RTT;
  using namespace std;

  static const string JOINT_NAME_ARRAY[] =
  //{"j0","j1","j2","j3","j4","w1","w2","w3","w4","f1","f2"};
  { "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5",
      "wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br",
      "caster_joint_fl","caster_joint_fr", "caster_joint_bl", "caster_joint_br",
      "gripper_finger_joint_l", "gripper_finger_joint_r"};

  static const unsigned int SIZE_JOINT_NAME_ARRAY=15;
  static const double MIN_ENERGY =0.0;

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

    std_msgs::Float64MultiArray m_energy_tank;
    double m_arm_energy_tank;
    double m_base_energy_tank;
    double m_kinematics_energy_tank;
    std::string m_events;
  //	InputPort<flat_matrix_t> odometry;

    OutputPort<sensor_msgs::JointState> youbot_state;
    sensor_msgs::JointState m_youbot_state;
    // Republishing energy states to make an event when energy in the tanks drop below zeros
    InputPort<std_msgs::Float64MultiArray> arm_energy_tank;
    InputPort<std_msgs::Float64MultiArray> base_energy_tank;
    InputPort<std_msgs::Float64MultiArray> kinematics_energy_tank;
    OutputPort<std::string> events;

    unsigned int m_dimension;
    double wheel;
  };

}

