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

#include <list>

#include <YouBotTypes.hpp>

#include "ExecutiveTypes.hpp"
#include "ConnectionMapping.hpp"

namespace YouBot
{

  using namespace RTT;
  using namespace std;

  class YouBot_executive: public TaskContext
  {
  public:
    YouBot_executive(const string& name);
    virtual ~YouBot_executive();

    virtual bool configureHook();
    virtual void updateHook();

    void setupGravityMode();
    void setupJointControl();
    void setupCartesianControl();
    void setupDualControl();
    void setupNavigationControl();

    // predefined settings
    void unfoldArmPosition();
    void foldArmPosition();
    // Specify the DOF to use
    void useBaseOnly();
    void useArmOnly();
    void useFullRobot();

    // Apply
    void execute();

    // predefined actions
    void openGripper();
    void closeGripper();

    // Cartesian space actions
    void setHvp0(vector<double> position_c);
    void setHtipCC(vector<double> position_c);
    void setCartesianStiffness(vector<double> stiffness_c);
    void setCartesianDamping(vector<double> damping);

    //Joint Space actions
    void setArmJointAngles(vector<double> position_j);
    void setHBase0(vector<double> position);

    //readouts
    void getArmJointStates(vector<double>& sample);
    void getTip_xyzypr(vector<double>& sample);
    void getHtip0(vector<double>& sample_H);
    void getHBase0(vector<double>& sample_H);

    void doneEvent();

    // hacks
    void sleep(double seconds);
    void quaternionToH(vector<double>& quat, vector<double>& H);

    // Ports and their variables
    RTT::OutputPort<flat_matrix_t> ArmJointAnglesSetpoint;
    RTT::OutputPort<flat_matrix_t> HBase0Setpoint;

    RTT::OutputPort<flat_matrix_t> ArmJointActive;
    RTT::OutputPort<flat_matrix_t> BaseJointActive;

    RTT::OutputPort<flat_matrix_t> CartSpaceSetpoint;
    RTT::OutputPort<flat_matrix_t> CartSpaceStiffness;
    RTT::OutputPort<flat_matrix_t> HtipCC;
    RTT::OutputPort<flat_matrix_t> CartSpaceDamping;

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

    void stateTransition(state_t new_state);

    void clearControlModes(); // Go to gravityMode settings -> always safe!

    // Variables for ports
    flat_matrix_t m_ArmJointAnglesSetpoint;
    flat_matrix_t m_HBase0Setpoint;

    flat_matrix_t m_ArmJointActive;
    flat_matrix_t m_BaseJointActive;

    flat_matrix_t m_Hvp0;
    flat_matrix_t m_CartSpaceStiffness;
    flat_matrix_t m_CartSpaceStiffness_orig;
    flat_matrix_t m_HtipCC;
    flat_matrix_t m_CartSpaceDamping;

    flat_matrix_t m_Htip0;
    flat_matrix_t m_ArmJointState;
    flat_matrix_t m_Wtip0;
    flat_matrix_t m_H_base_0;

    std_msgs::Float64MultiArray m_stiffness_slider;

    std_msgs::Bool m_open_gripper;

    motion_control_msgs::JointPositions m_gripper_cmd;

    OperationCaller<void(vector<ctrl_modes>)> base_setControlModes;
    OperationCaller<void(vector<ctrl_modes>)> arm1_setControlModes;
    vector<ctrl_modes> m_base_ctrl_modes;
    vector<ctrl_modes> m_arm1_ctrl_modes;

    state_t m_state;

    bool use_stiffness_slider;

    std::string m_events;
  };

}
