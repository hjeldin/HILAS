/*
 * YouBot_executive.h
 *
 *  Created on: Dec 15, 2011
 *      Author: Yury Brodskiy
 */
#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <motion_control_msgs/JointPositions.h>

#include "ExecutiveTypes.hpp"

namespace YouBot
{

  using namespace RTT;
  using namespace std;

  class YouBot_executive: public TaskContext
  {
  public:
    YouBot_executive(const string& name);
    virtual ~YouBot_executive();

    virtual void updateHook();

    void stateTransition(state_t new_state);
    void stateFullControl();
    void stateGravityMode();
    void stateJointControl();
    void stateCartesianControl();

    void doneEvent();

// predefined actions
    void unfoldArm();
    void foldArm();
    void gravityMode();
    void fullControlMode();
    void cartesianControlMode();
    void jointspaceControlMode();
    void openGripper();
    void closeGripper();

    // configurable actions
    void setBaseJointActive(vector<double> inp);
    void setArmJointActive(vector<double> inp);

    // Cartesian space actions
    void setHvp0(vector<double> position_c);
    void setHtipCC(vector<double> position_c);
    void setCartesianStiffness(vector<double> stiffness_c);
    //Joint Space actions
    void setArmJointAngles(vector<double> position_j);

    //readouts
    void getArmJointStates(vector<double>& sample);
    void getTip_xyzypr(vector<double>& sample);
    void getHtip0(vector<double>& sample_H);

    // hacks
    void sleep(double seconds);

    // Ports and their variables
    RTT::OutputPort<flat_matrix_t> ArmJointAnglesSetpoint;

    RTT::OutputPort<flat_matrix_t> ArmJointActive;
    RTT::OutputPort<flat_matrix_t> BaseJointActive;

    RTT::OutputPort<flat_matrix_t> CartSpaceSetpoint;
    RTT::OutputPort<flat_matrix_t> CartSpaceStiffness;
    RTT::OutputPort<flat_matrix_t> HtipCC;

    RTT::InputPort<flat_matrix_t> Htip0;
    RTT::InputPort<flat_matrix_t> ArmJointStates;
    RTT::InputPort<flat_matrix_t> Wtip0;
    RTT::InputPort<flat_matrix_t> H_base_0;

    RTT::InputPort<std_msgs::Float64MultiArray> stiffness_slider;

    RTT::InputPort<std_msgs::Bool> open_gripper;

    RTT::OutputPort<motion_control_msgs::JointPositions> gripper_cmd;

    RTT::OutputPort<std::string> events;

  protected:
    void setupComponentInterface();
    void init();

    void calculateCartStiffness();

    void readAll();
    void writeAll();

    // The following are set from operations.
    vector<double> m_force_cart;
    bool checkForce;

    // Variables for ports
    flat_matrix_t m_ArmJointState;
    flat_matrix_t m_ArmJointAnglesSetpoint;
    flat_matrix_t m_ArmJointActive;
    flat_matrix_t m_BaseJointActive;

    flat_matrix_t m_Htip0;
    flat_matrix_t m_H_base_0;

    flat_matrix_t m_Hvp0;
    flat_matrix_t m_CartSpaceStiffness;
    flat_matrix_t m_CartSpaceStiffness_orig;
    flat_matrix_t m_HtipCC;

    flat_matrix_t m_Wtip0;

    std_msgs::Bool m_open_gripper;
    std_msgs::Float64MultiArray m_stiffness_slider;

    motion_control_msgs::JointPositions m_gripper_cmd;

    state_t m_state;

    std::string m_events;

  };

}
