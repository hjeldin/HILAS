/*
 * RCC_executive.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: Yury Brodskiy
 */
#include "YouBot_executive.h"

#include <tf/transform_broadcaster.h>
#include "ExecutiveHelpers.hpp"
#include <boost/thread/thread.hpp>

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

  YouBot_executive::YouBot_executive(const string& name) :
      TaskContext(name)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();

    this->init();

    this->setupComponentInterface();
  }

  YouBot_executive::~YouBot_executive()
  {
  }

  void YouBot_executive::setupComponentInterface()
  {
    // Predefined actions
    this->addOperation("unfoldArm", &YouBot_executive::unfoldArm, this,
        OwnThread).doc("Unfold the arm. Takes no arguments");
    this->addOperation("foldArm", &YouBot_executive::foldArm, this, OwnThread).doc(
        "Fold the arm. Takes no arguments");

    this->addOperation("gravityMode", &YouBot_executive::gravityMode, this,
        OwnThread).doc(
        "Set gravity compensation mode (disables joint/cartesian/full mode setpoints.");
    this->addOperation("cartesianControlMode",
        &YouBot_executive::cartesianControlMode, this, OwnThread).doc(
        "Set cartesian space control mode (disables joint/full mode setpoints).");
    this->addOperation("jointspaceControlMode",
        &YouBot_executive::jointspaceControlMode, this, OwnThread).doc(
        "Set joint space control mode (disables cartesian/full mode setpoints).");
    this->addOperation("fullControlMode", &YouBot_executive::fullControlMode,
        this, OwnThread).doc(
        "Set full control mode. Superposition of joint and cartesian space control.");

    this->addOperation("openGripper", &YouBot_executive::openGripper, this,
        OwnThread).doc("Set gripper gap  to maximum. Takes no arguments");
    this->addOperation("closeGripper", &YouBot_executive::closeGripper, this,
        OwnThread).doc("Set gripper gap  to minimum. Takes no arguments");

// Configurable actions
    this->addOperation("setBaseJointActive", &YouBot_executive::setBaseJointActive,
        this, OwnThread).doc("Enable/disable Base joints.");
    this->addOperation("setArmJointActive", &YouBot_executive::setArmJointActive,
        this, OwnThread).doc("Enable/disable Arm joints.");
    this->addOperation("setArmJointAngles", &YouBot_executive::setArmJointAngles,
        this, OwnThread).doc("Joint space control");
    this->addOperation("setHvp0", &YouBot_executive::setHvp0, this, OwnThread).doc(
        "Define attraction point with respect to inertial frame. Takes vector flatten H matrix");
    this->addOperation("setHtipCC", &YouBot_executive::setHtipCC, this,
        OwnThread).doc(
        "Define center of stiffness and principal axes with respect to tool tip frame.Takes vector flatten H matrix");

    this->addOperation("setCartesianStiffness",
        &YouBot_executive::setCartesianStiffness, this, OwnThread).doc(" ");

// Sampling state
    this->addOperation("getArmJointStates", &YouBot_executive::getArmJointStates,
        this, OwnThread).doc("Get joint space positions");
    this->addOperation("getTip_xyzypr", &YouBot_executive::getTip_xyzypr, this,
        OwnThread).doc(
        "DEPRECATED -- Get tool tip frame in xyz roll pitch yaw. ");
    this->addOperation("getHtip0", &YouBot_executive::getHtip0, this, OwnThread).doc(
        "Get tool tip frame. Returns vector flatten H matrix");

    // Ports
    this->addPort("ArmJointActive", ArmJointActive).doc("");
    this->addPort("BaseJointActive", BaseJointActive).doc("");

    this->addPort("ArmJointAnglesSetpoint", ArmJointAnglesSetpoint).doc("");
    this->addPort("CartSpaceSetpoint", CartSpaceSetpoint).doc("");
    this->addPort("CartSpaceStiffness", CartSpaceStiffness).doc("");
    this->addPort("HtipCC", HtipCC).doc("");

    this->addPort("H_base_0", H_base_0).doc("Base only pose");
    this->addPort("Htip0", Htip0).doc("Gripper pose");
    this->addPort("ArmJointStates", ArmJointStates).doc("Joint space joint states");
    this->addPort("Wtip0", Wtip0).doc("Wrench input from control");

    this->addEventPort("open_gripper", open_gripper).doc(
        "Event port to open the gripper.");

    this->addEventPort("stiffness_slider", stiffness_slider).doc(
        "Slider to go from pure driving to arm+base control state. Expects input values between -1 and 1.");

    this->addPort("gripper_cmd", gripper_cmd).doc("");
    this->addPort("events_out", events).doc("");

    // Debugging/introspection properties
    this->addProperty("ArmJointActive", m_ArmJointActive.data);
    this->addProperty("BaseJointActive", m_BaseJointActive.data);

    this->addProperty("ArmJointAnlgesSetpoint", m_ArmJointAnglesSetpoint.data);
    this->addProperty("Hvp0", m_Hvp0.data);
    this->addProperty("H_base_0", m_H_base_0.data).doc("Base only pose");
    this->addProperty("CartSpaceStiffness", m_CartSpaceStiffness.data);
    this->addProperty("Wtip0", m_Wtip0.data);
    this->addProperty("HtipCC", m_HtipCC.data);
    this->addProperty("state", m_state);
    this->addProperty("cartesianSlider", m_stiffness_slider.data);

    this->addOperation("sleep", &YouBot_executive::sleep, this, OwnThread).doc(
        "Hack: Sleeping management.");

  }

  void YouBot_executive::init()
  {
    // Variables for ports
    m_ArmJointState.data.resize(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_ArmJointAnglesSetpoint.data.resize(SIZE_ARM_JOINTS_ARRAY, 0.0);

    m_ArmJointActive.data.resize(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_BaseJointActive.data.resize(SIZE_BASE_JOINTS_ARRAY, 0.0);

    m_H_base_0.data.resize(SIZE_H, 0.0);
    m_Htip0.data.resize(SIZE_H, 0.0);
    m_Hvp0.data.resize(SIZE_H, 0.0);
    m_CartSpaceStiffness.data.resize(SIZE_CART_STIFFNESS, 0.0);
    m_CartSpaceStiffness_orig.data.resize(SIZE_CART_STIFFNESS, 0.0);
    m_HtipCC.data.assign(EYE4, EYE4 + SIZE_H);
    m_Hvp0.data.assign(EYE4, EYE4 + SIZE_H);

    m_Wtip0.data.resize(SIZE_CART_SPACE, 0.0);

    m_gripper_cmd.positions.resize(1, 0.0);

    m_open_gripper.data = false;

    m_stiffness_slider.data.resize(1, -1); // 0%

    m_events.reserve(max_event_length);

    ArmJointActive.setDataSample(m_ArmJointActive);
    BaseJointActive.setDataSample(m_BaseJointActive);

    ArmJointAnglesSetpoint.setDataSample(m_ArmJointAnglesSetpoint);
    CartSpaceSetpoint.setDataSample(m_Hvp0);
    CartSpaceStiffness.setDataSample(m_CartSpaceStiffness);
    HtipCC.setDataSample(m_HtipCC);
    gripper_cmd.setDataSample(m_gripper_cmd);

    m_state = GRAVITY_MODE; //default state
    stateGravityMode();
  }

  void YouBot_executive::openGripper()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    m_gripper_cmd.positions[0] = GRIPPER_OPENING;
    gripper_cmd.write(m_gripper_cmd);
  }

  void YouBot_executive::closeGripper()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    m_gripper_cmd.positions[0] = 0.0001;
    gripper_cmd.write(m_gripper_cmd);
  }

  void YouBot_executive::unfoldArm()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    m_ArmJointAnglesSetpoint.data.assign(UNFOLD_JOINT_POSE,
        UNFOLD_JOINT_POSE + SIZE_ARM_JOINTS_ARRAY);
    m_HtipCC.data.assign(EYE4, EYE4 + SIZE_H);
    stateTransition(FULL_CONTROL);
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 1.0);
    stateFullControl();
  }

  void YouBot_executive::foldArm()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    m_ArmJointAnglesSetpoint.data.assign(FOLD_JOINT_POSE,
        FOLD_JOINT_POSE + SIZE_ARM_JOINTS_ARRAY);
    m_HtipCC.data.assign(EYE4, EYE4 + SIZE_H);
    stateTransition(FULL_CONTROL);
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 1.0);
    stateFullControl();
  }

  void YouBot_executive::gravityMode()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    stateTransition(GRAVITY_MODE);
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 0.0);
    stateGravityMode();
  }

  void YouBot_executive::fullControlMode()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    stateTransition(FULL_CONTROL);
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 1.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 1.0);
    stateFullControl();
  }

  void YouBot_executive::cartesianControlMode()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    stateTransition(CARTESIAN_CONTROL);
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 1.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 1.0);
    stateCartesianControl();
  }

  void YouBot_executive::jointspaceControlMode()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    stateTransition(JOINT_CONTROL);
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 1.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 1.0);
    stateJointControl();
  }

  void YouBot_executive::setCartesianStiffness(vector<double> stiffness_c)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (stiffness_c.size() != SIZE_CART_STIFFNESS)
    {
      log(Error) << "setCartesianStiffness - expects a " << SIZE_CART_STIFFNESS
          << " dimensional vector" << endlog();
      return;
    }
    m_CartSpaceStiffness_orig.data.assign(stiffness_c.begin(),
        stiffness_c.end());
    calculateCartStiffness(); // adjust for the slider position
  }

  void YouBot_executive::setArmJointAngles(vector<double> position_j)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (position_j.size() != SIZE_ARM_JOINTS_ARRAY)
    {
      log(Error) << "positionArm - expects a " << SIZE_ARM_JOINTS_ARRAY
          << " dimensional vector" << endlog();
      return;
    }
    m_ArmJointAnglesSetpoint.data.assign(position_j.begin(), position_j.end());
  }

  void YouBot_executive::setHvp0(vector<double> position_c)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (position_c.size() != SIZE_H)
    {
      log(Error) << "positionGripper - expects a " << SIZE_H
          << " dimensional vector" << endlog();
      return;
    }
    m_Hvp0.data.assign(position_c.begin(), position_c.end());
  }

  void YouBot_executive::setHtipCC(vector<double> position_c)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (position_c.size() != SIZE_H)
    {
      log(Error) << "positionGripper - expects a " << SIZE_H
          << " dimensional vector" << endlog();
      return;
    }
    m_HtipCC.data.assign(position_c.begin(), position_c.end());
  }

  void YouBot_executive::getArmJointStates(vector<double> & sample)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (sample.size() != SIZE_ARM_JOINTS_ARRAY)
    {
      log(Error) << "getArmPose - expects a " << SIZE_ARM_JOINTS_ARRAY
          << " dimensional vector" << endlog();
      return;
    }
    readAll();
    sample.assign(m_ArmJointState.data.begin(), m_ArmJointState.data.end());
  }

  void YouBot_executive::getTip_xyzypr(vector<double> & sample)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (sample.size() != SIZE_CART_SPACE)
    {
      log(Error) << "getGripperPose - expects a " << SIZE_CART_SPACE
          << " dimensional vector" << endlog();
      return;
    }
    readAll();
    homogeneous_to_xyzypr(m_Htip0.data, sample);
  }

  void YouBot_executive::getHtip0(vector<double>& sample_H)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (sample_H.size() != SIZE_H)
    {
      log(Error) << "getGripperH - expects a " << SIZE_H
          << " dimensional vector" << endlog();
      return;
    }
    readAll();
    sample_H.assign(m_Htip0.data.begin(), m_Htip0.data.end());
  }

  void YouBot_executive::doneEvent()
  {
    events.write(make_event(m_events, "executive.e_done"));
  }

  void YouBot_executive::stateTransition(state_t new_state)
  {
    m_state = new_state;
  }

  void YouBot_executive::updateHook()
  {
    TaskContext::updateHook();

    // EventPort
    if (open_gripper.read(m_open_gripper) == NewData)
    {
      if (m_open_gripper.data)
        openGripper();
      else
        closeGripper();
    }

    if (stiffness_slider.read(m_stiffness_slider) == NewData)
    {
      calculateCartStiffness();
      if (m_state == FULL_CONTROL || m_state == CARTESIAN_CONTROL) // Apply immediately iff in these modes. Does NOT affect gravity nor jointspace control modes
      {
        CartSpaceStiffness.write(m_CartSpaceStiffness); // no 'commit' necessary)
      }
    }
  }

  void YouBot_executive::calculateCartStiffness()
  {
    double percentage = (m_stiffness_slider.data[0] + 1) / 2; // For the Logitech joystick the input will be between -1 and +1
    if (percentage >= 0.0 && percentage <= 1.0)
    {
      for (unsigned int i = 0; i < SIZE_CART_STIFFNESS; ++i)
      {
        m_CartSpaceStiffness.data[i] = m_CartSpaceStiffness_orig.data[i]
            * percentage;
      }
    }
  }

  void YouBot_executive::readAll()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    // read all input ports
    Htip0.read(m_Htip0);
    H_base_0.read(m_H_base_0);
    ArmJointStates.read(m_ArmJointState);
    Wtip0.read(m_Wtip0);
  }

  void YouBot_executive::writeAll()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    // write setpoints
    assert(m_ArmJointAnglesSetpoint.data.size() == SIZE_ARM_JOINTS_ARRAY);
    ArmJointAnglesSetpoint.write(m_ArmJointAnglesSetpoint);

    assert(m_Hvp0.data.size() == SIZE_H);
    CartSpaceSetpoint.write(m_Hvp0);

    assert(m_CartSpaceStiffness.data.size() == SIZE_CART_STIFFNESS);
    CartSpaceStiffness.write(m_CartSpaceStiffness);

    assert(m_HtipCC.data.size() == SIZE_H);
    HtipCC.write(m_HtipCC);

    assert(m_ArmJointActive.data.size() == SIZE_ARM_JOINTS_ARRAY);
    ArmJointActive.write(m_ArmJointActive);

    assert(m_BaseJointActive.data.size() == SIZE_BASE_JOINTS_ARRAY);
    BaseJointActive.write(m_BaseJointActive);

  }

  void YouBot_executive::stateFullControl()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    readAll();
    // No additional actions required
    writeAll();
  }

  void YouBot_executive::stateGravityMode()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    readAll();
    // Assigns the current states as setpoints and sets the stiffness zero
    // zero out Joint control part
    m_ArmJointAnglesSetpoint.data.assign(m_ArmJointState.data.begin(),
        m_ArmJointState.data.end());

    // zero out Cartesian control part
    m_CartSpaceStiffness.data.assign(SIZE_CART_STIFFNESS, 0.0);
    m_Hvp0.data.assign(m_Htip0.data.begin(), m_Htip0.data.end());
    writeAll();
  }

  void YouBot_executive::stateJointControl()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    readAll();
    // zero out Cartesian control part
    m_CartSpaceStiffness.data.assign(SIZE_CART_STIFFNESS, 0.0);
    m_Hvp0.data.assign(m_Htip0.data.begin(), m_Htip0.data.end());
    writeAll();
  }

  void YouBot_executive::stateCartesianControl()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    readAll();
    // zero out Joint control part
//	m_JointSpaceStiffness.data.assign(SIZE_ARM_JOINTS_ARRAY,0.0);
    m_ArmJointAnglesSetpoint.data.assign(m_ArmJointState.data.begin(),
        m_ArmJointState.data.end());
    writeAll();
  }

  void YouBot_executive::setBaseJointActive(vector<double> inp)
  {
    if(inp.size() == SIZE_BASE_JOINTS_ARRAY)
    {
      m_BaseJointActive.data = inp;
      writeAll();
    }
    else
    {
      log(Warning) << "Number of joints is incorrect, please specify: " << SIZE_BASE_JOINTS_ARRAY << " joints." << endlog();
    }
  }

  void YouBot_executive::setArmJointActive(vector<double> inp)
  {
    if(inp.size() == SIZE_ARM_JOINTS_ARRAY)
    {
      m_ArmJointActive.data = inp;
      writeAll();
    }
    else
    {
      log(Warning) << "Number of joints is incorrect, please specify: " << SIZE_ARM_JOINTS_ARRAY << " joints." << endlog();
    }
  }

  void YouBot_executive::sleep(double seconds)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    boost::this_thread::sleep(boost::posix_time::seconds(seconds));
  }

}

ORO_CREATE_COMPONENT( YouBot::YouBot_executive)

