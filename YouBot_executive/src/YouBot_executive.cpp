/*
 * RCC_executive.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: Yury Brodskiy
 */
#include "YouBot_executive.h"
#include "ConnectionMapping.hpp"

#include <tf/transform_broadcaster.h>
#include "ExecutiveHelpers.hpp"
#include <boost/thread/thread.hpp>

#include <math.h>

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

  YouBot_executive::YouBot_executive(const string& name) :
      TaskContext(name, PreOperational)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();

    this->init();

    this->setupComponentInterface();
  }

  YouBot_executive::~YouBot_executive()
  {
  }

  bool YouBot_executive::configureHook()
  {
    TaskContext* driver = getPeer("driver"); //setControlModes(vector<ctrl_modes>& all)
    if(driver == NULL)
    {
        log(Error) << "No driver TaskContext found, please connectPeers()" << endlog();
        this->exception();
    }

    Service::shared_ptr base = driver->provides(std::string("Base"));
    Service::shared_ptr arm1 = driver->provides(std::string("Arm1"));

    if(base == NULL || arm1 == NULL)
    {
      log(Error) << "Base or Arm1 service not present in driver!" << endlog();
      this->exception();
    }

    base_setControlModes = base->getOperation("setControlModes");
    if( !base_setControlModes.ready() )
    {
      log(Error) << "setControlModes Operation not found!" << endlog();
      this->exception();
    }

    arm1_setControlModes = arm1->getOperation("setControlModes");
    if( !arm1_setControlModes.ready() )
    {
      log(Error) << "setControlModes Operation not found!" << endlog();
      this->exception();
    }
  }

  void YouBot_executive::setupComponentInterface()
  {
    // Predefined actions
    this->addOperation("unfoldArmPosition", &YouBot_executive::unfoldArmPosition, this,
        OwnThread).doc("Unfold the arm. Takes no arguments");
    this->addOperation("foldArmPosition", &YouBot_executive::foldArmPosition, this, OwnThread).doc(
        "Fold the arm. Takes no arguments");

    // Transaction configuration
    this->addOperation("gravityCompensation", &YouBot_executive::setupGravityMode, this, OwnThread).doc(
        "Set gravity compensation mode.");
    this->addOperation("cartesianControl", &YouBot_executive::setupCartesianControl, this, OwnThread).doc(
        "Set cartesian space control mode.");
    this->addOperation("jointspaceControl", &YouBot_executive::setupJointControl, this, OwnThread).doc(
        "Set joint space control mode.");
    this->addOperation("dualControl", &YouBot_executive::setupDualControl, this, OwnThread).doc(
        "Set joint space and Cartesian control mode simultaneously.");

    this->addOperation("useBaseOnly", &YouBot_executive::useBaseOnly, this, OwnThread).doc("");
    this->addOperation("useArmOnly", &YouBot_executive::useArmOnly, this, OwnThread).doc("");
    this->addOperation("useFullRobot", &YouBot_executive::useFullRobot, this, OwnThread).doc("");

    this->addOperation("setArmJointAngles", &YouBot_executive::setArmJointAngles, this, OwnThread).doc(
        "Joint space setpoints for the arm");
    this->addOperation("setHBase0", &YouBot_executive::setHBase0, this, OwnThread).doc(
        "Joint space setpoints for the base");

    this->addOperation("setHvp0", &YouBot_executive::setHvp0, this, OwnThread).doc(
        "Defines attraction point with respect to inertial frame. Takes vector flatten H matrix");
    this->addOperation("setHtipCC", &YouBot_executive::setHtipCC, this, OwnThread).doc(
        "Defines center of stiffness and principal axes with respect to tool tip frame. Takes vector flatten H matrix");
    this->addOperation("setCartesianStiffness", &YouBot_executive::setCartesianStiffness, this, OwnThread).doc(
        "Defines the cartesian stiffness for the virtual springs.");
    this->addOperation("setCartesianDamping", &YouBot_executive::setCartesianDamping, this, OwnThread).doc(
            "Defines the cartesian damping in the tooltip of the robot.");

    this->addOperation("execute", &YouBot_executive::execute, this, OwnThread).doc(
        "Apply all settings in one transaction.");

    // immediate effect functions
    this->addOperation("openGripper", &YouBot_executive::openGripper, this,
        OwnThread).doc("Set gripper gap  to maximum. Takes no arguments");
    this->addOperation("closeGripper", &YouBot_executive::closeGripper, this,
        OwnThread).doc("Set gripper gap  to minimum. Takes no arguments");

    // Sampling state
    this->addOperation("getArmJointStates", &YouBot_executive::getArmJointStates,
        this, OwnThread).doc("Get joint space positions");

    this->addOperation("getHBase0", &YouBot_executive::getHBase0,
        this, OwnThread).doc("Get cartesian position for the base.");
    this->addOperation("getTip_xyzypr", &YouBot_executive::getTip_xyzypr, this, OwnThread).doc(
        "DEPRECATED -- Get tool tip frame in xyz roll pitch yaw. ");
    this->addOperation("getHtip0", &YouBot_executive::getHtip0, this, OwnThread).doc(
        "Get tool tip frame. Returns vector flatten H matrix");

    this->addOperation("quaternionToH", &YouBot_executive::quaternionToH, this, OwnThread).doc(
        "Converts a Quaternion(1) to H matrix(2).");

    // Ports
    this->addPort("ArmJointActive", ArmJointActive).doc("");
    this->addPort("BaseJointActive", BaseJointActive).doc("");

    this->addPort("ArmJointAnglesSetpoint", ArmJointAnglesSetpoint).doc("");
    this->addPort("HBase0Setpoint", HBase0Setpoint).doc("");

    this->addPort("CartSpaceSetpoint", CartSpaceSetpoint).doc("");
    this->addPort("CartSpaceStiffness", CartSpaceStiffness).doc("");
    this->addPort("HtipCC", HtipCC).doc("");
    this->addPort("CartSpaceDamping", CartSpaceDamping).doc("");

    this->addPort("ArmJointStates", ArmJointStates).doc("Joint space joint states");

    this->addPort("H_base_0", H_base_0).doc("Base only pose");
    this->addPort("Htip0", Htip0).doc("Gripper pose");
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

    this->addProperty("ArmJointAnglesSetpoint", m_ArmJointAnglesSetpoint.data);
    this->addProperty("HBase0Setpoint", m_HBase0Setpoint.data);

    this->addProperty("Hvp0", m_Hvp0.data);
    this->addProperty("H_base_0", m_H_base_0.data).doc("Base only pose");
    this->addProperty("CartSpaceStiffness", m_CartSpaceStiffness.data);
    this->addProperty("CartSpaceDamping", m_CartSpaceDamping.data);

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

    m_ArmJointAnglesSetpoint.data.resize(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_HBase0Setpoint.data.assign(EYE4, EYE4 + SIZE_H);

    m_ArmJointActive.data.resize(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_BaseJointActive.data.resize(SIZE_BASE_JOINTS_ARRAY, 0.0);

    m_Hvp0.data.assign(EYE4, EYE4 + SIZE_H);
    m_CartSpaceStiffness.data.resize(SIZE_CART_STIFFNESS, 0.0);
    m_CartSpaceStiffness_orig.data.resize(SIZE_CART_STIFFNESS, 0.0);
    m_HtipCC.data.assign(EYE4, EYE4 + SIZE_H);
    m_CartSpaceDamping.data.assign(SIZE_CART_SPACE, 0.0);

    m_Htip0.data.resize(SIZE_H, 0.0);
    m_ArmJointState.data.resize(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_Wtip0.data.resize(SIZE_CART_SPACE, 0.0);
    m_H_base_0.data.resize(SIZE_H, 0.0);

    m_stiffness_slider.data.resize(1, -1); // 0%

    m_open_gripper.data = false;

    m_gripper_cmd.positions.resize(1, 0.0);

    m_events.reserve(max_event_length);


    // set data samples
    ArmJointAnglesSetpoint.setDataSample(m_ArmJointAnglesSetpoint);
    HBase0Setpoint.setDataSample(m_HBase0Setpoint);

    ArmJointActive.setDataSample(m_ArmJointActive);
    BaseJointActive.setDataSample(m_BaseJointActive);

    CartSpaceSetpoint.setDataSample(m_Hvp0);
    CartSpaceStiffness.setDataSample(m_CartSpaceStiffness);
    HtipCC.setDataSample(m_HtipCC);
    CartSpaceDamping.setDataSample(m_CartSpaceDamping);

    gripper_cmd.setDataSample(m_gripper_cmd);

    use_stiffness_slider = false;

    // default state
    m_state = GRAVITY_MODE;
    setupGravityMode();
    execute();
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

  void YouBot_executive::unfoldArmPosition()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    m_ArmJointAnglesSetpoint.data.assign(UNFOLD_JOINT_POSE,
        UNFOLD_JOINT_POSE + SIZE_ARM_JOINTS_ARRAY);
  }

  void YouBot_executive::foldArmPosition()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    m_ArmJointAnglesSetpoint.data.assign(FOLD_JOINT_POSE,
        FOLD_JOINT_POSE + SIZE_ARM_JOINTS_ARRAY);
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
    m_CartSpaceStiffness_orig.data.assign(stiffness_c.begin(), stiffness_c.end());
    calculateCartStiffness(); // adjust for the slider position
  }

  void YouBot_executive::setCartesianDamping(vector<double> damping)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (damping.size() != SIZE_CART_SPACE)
    {
      log(Error) << "setCartesianDamping - expects a " << SIZE_CART_SPACE
          << " dimensional vector" << endlog();
      return;
    }
    m_CartSpaceDamping.data.assign(damping.begin(), damping.end());
  }

  void YouBot_executive::setArmJointAngles(vector<double> position_j)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    if (position_j.size() != SIZE_ARM_JOINTS_ARRAY)
    {
      log(Error) << __FUNCTION__ << " - expects a " << SIZE_ARM_JOINTS_ARRAY
          << " dimensional vector" << endlog();
      return;
    }
    m_ArmJointAnglesSetpoint.data.assign(position_j.begin(), position_j.end());
  }

  void YouBot_executive::setHBase0(vector<double> position)
  {
    if (position.size() != SIZE_BASE_JOINTS_ARRAY)
    {
      log(Error) << __FUNCTION__ << " - expects a " << SIZE_BASE_JOINTS_ARRAY
          << " dimensional vector/matrix" << endlog();
      return;
    }
    m_HBase0Setpoint.data.assign(position.begin(), position.end());
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

  void YouBot_executive::getHBase0(vector<double> & sample_H)
  {
    if (sample_H.size() != SIZE_BASE_JOINTS_ARRAY)
    {
      log(Error) << __FUNCTION__ << " - expects a " << SIZE_BASE_JOINTS_ARRAY
          << " dimensional vector/matrix" << endlog();
      return;
    }
    readAll();
    sample_H.assign(m_H_base_0.data.begin(), m_H_base_0.data.end());
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

  void YouBot_executive::stateTransition(state_t new_state)
  {
    readAll();

    // The clearControlModes will setup gravity mode always
    clearControlModes();

    m_state = new_state;
  }

  void YouBot_executive::calculateCartStiffness()
  {
    double percentage = 1.0;

    if(use_stiffness_slider)
      percentage = (m_stiffness_slider.data[0] + 1) / 2; // For the Logitech joystick the input will be between -1 and +1

    if (percentage >= 0.0 && percentage <= 1.0)
    {
      for (unsigned int i = 0; i < SIZE_CART_STIFFNESS; ++i)
      {
        m_CartSpaceStiffness.data[i] = m_CartSpaceStiffness_orig.data[i] * percentage;
//        log(Info) << "CartSpaceStiffness[" << i << "] = " << m_CartSpaceStiffness.data[i] << endlog();
      }
    }
  }

  void YouBot_executive::updateHook()
  {
    TaskContext::updateHook();

    readAll(); // inputs from the controller components

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
      use_stiffness_slider = true;
      calculateCartStiffness();
      CartSpaceStiffness.write(m_CartSpaceStiffness);
    }
  }

  void YouBot_executive::readAll()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    // read all input ports
    // Cartesian space
    Htip0.read(m_Htip0);
    Wtip0.read(m_Wtip0);

    // Joint space
    H_base_0.read(m_H_base_0);
    ArmJointStates.read(m_ArmJointState);
  }

  void YouBot_executive::setupGravityMode()
  {
    stateTransition(GRAVITY_MODE);
  }

  void YouBot_executive::setupJointControl()
  {
    stateTransition(JOINT_CONTROL);
  }

  void YouBot_executive::setupCartesianControl()
  {
    stateTransition(CARTESIAN_CONTROL);

    // Default parameters
    m_HtipCC.data.assign(EYE4, EYE4 + SIZE_H);
    m_CartSpaceStiffness_orig.data.assign(BASIC_CART_STIFFNESS, BASIC_CART_STIFFNESS + SIZE_CART_STIFFNESS);
    m_CartSpaceDamping.data.assign(SIZE_CART_SPACE, 10.0);
    calculateCartStiffness();
  }

  void YouBot_executive::setupDualControl()
  {
    stateTransition(DUAL_CONTROL);

    // Default parameters
    m_HtipCC.data.assign(EYE4, EYE4 + SIZE_H);
    m_CartSpaceStiffness_orig.data.assign(BASIC_CART_STIFFNESS, BASIC_CART_STIFFNESS + SIZE_CART_STIFFNESS);
    m_CartSpaceDamping.data.assign(SIZE_CART_SPACE, 10.0);
    calculateCartStiffness();
  }

  void YouBot_executive::clearControlModes()
  {
    // Assigns the current states as setpoints and sets the stiffness zero
    // zero out Joint control part
    m_ArmJointAnglesSetpoint.data.assign(m_ArmJointState.data.begin(), m_ArmJointState.data.end());
    m_HBase0Setpoint.data.assign(m_H_base_0.data.begin(), m_H_base_0.data.end());
    // Disable both PD controllers
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 0.0);

    // zero out Cartesian control part
    m_CartSpaceStiffness_orig.data.assign(SIZE_CART_STIFFNESS, 0.0);
    m_Hvp0.data.assign(m_Htip0.data.begin(), m_Htip0.data.end());
    m_HtipCC.data.assign(EYE4, EYE4 + SIZE_H);
    m_CartSpaceDamping.data.assign(SIZE_CART_SPACE, 0.0);
    calculateCartStiffness();

    base_setControlModes(vector<ctrl_modes>(NR_OF_BASE_SLAVES, TORQUE));
    arm1_setControlModes(vector<ctrl_modes>(NR_OF_ARM_SLAVES, TORQUE));
  }

  /**
   * @brief Process all transaction settings and apply them to the outputs.
   */
  void YouBot_executive::execute()
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    assert(m_ArmJointAnglesSetpoint.data.size() == SIZE_ARM_JOINTS_ARRAY);
    ArmJointAnglesSetpoint.write(m_ArmJointAnglesSetpoint);

    assert(m_Hvp0.data.size() == SIZE_H);
    CartSpaceSetpoint.write(m_Hvp0);

    assert(m_CartSpaceStiffness.data.size() == SIZE_CART_STIFFNESS);
    CartSpaceStiffness.write(m_CartSpaceStiffness);

    assert(m_HtipCC.data.size() == SIZE_H);
    HtipCC.write(m_HtipCC);

    assert(m_CartSpaceDamping.data.size() == SIZE_CART_SPACE);
    CartSpaceDamping.write(m_CartSpaceDamping);

    assert(m_HBase0Setpoint.data.size() == SIZE_H);
    HBase0Setpoint.write(m_HBase0Setpoint);

    assert(m_ArmJointActive.data.size() == SIZE_ARM_JOINTS_ARRAY);
    ArmJointActive.write(m_ArmJointActive);

    assert(m_BaseJointActive.data.size() == SIZE_BASE_JOINTS_ARRAY);
    BaseJointActive.write(m_BaseJointActive);
  }

  /*
   * Specify the DOF to use for the control mode.
   */

  void YouBot_executive::useBaseOnly()
  {
    if(m_state != JOINT_CONTROL && m_state != DUAL_CONTROL)
    {
      log(Warning) << "Cannot specify: " << __FUNCTION__ << ", this is only sensible in joint space and dual mode control." << endlog();
    }
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 0.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 1.0);
  }

  void YouBot_executive::useArmOnly()
  {
    if(m_state != JOINT_CONTROL && m_state != DUAL_CONTROL)
    {
      log(Warning) << "Cannot specify: " << __FUNCTION__ << ", this is only sensible in joint space and dual mode control." << endlog();
    }
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 1.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 0.0);
  }

  void YouBot_executive::useFullRobot()
  {
    if(m_state != JOINT_CONTROL && m_state != DUAL_CONTROL)
    {
      log(Warning) << "Cannot specify: " << __FUNCTION__ << ", this is only sensible in joint space and dual mode control." << endlog();
    }
    m_ArmJointActive.data.assign(SIZE_ARM_JOINTS_ARRAY, 1.0);
    m_BaseJointActive.data.assign(SIZE_BASE_JOINTS_ARRAY, 1.0);
  }

  void YouBot_executive::quaternionToH(vector<double>& quat, vector<double>& H)
  {
    if (H.size() != SIZE_H)
    {
      log(Error) << __FUNCTION__ << " - expects a " << SIZE_H
          << " dimensional output(2) vector/matrix" << endlog();
      return;
    }

    if (quat.size() != 7)
    {
      log(Error) << __FUNCTION__ << " - expects a " << 7
          << " dimensional input(1) vector" << endlog();
      return;
    }

    double qw, qx, qy, qz;
    qw = quat[7];
    qx = quat[4];
    qy = quat[5];
    qz = quat[6];

    H[0] = pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2);
    H[1] = 2*qx*qy - 2*qz*qw;
    H[2] = 2*qx*qz + 2*qy*qw;

    H[4] = 2*qx*qy + 2*qz*qw;
    H[5] = pow(qw,2) - pow(qx,2) + pow(qy,2) - pow(qz,2);
    H[6] = 2*qy*qz - 2*qx*qw;

    H[8] = 2*qx*qz - 2*qy*qw;
    H[9] = 2*qy*qz + 2*qx*qw;
    H[10] = pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2);

    H[3] = quat[0];
    H[7] = quat[1];
    H[11] = quat[2];

    H[12] = 0;
    H[13] = 0;
    H[14] = 0;
    H[15] = 1;
  }

  /**
   * @brief Hack!
   */
  void YouBot_executive::sleep(double seconds)
  {
    //log(Info) << "Executing: " << __FUNCTION__ << endlog();
    boost::this_thread::sleep(boost::posix_time::seconds(seconds));
  }

  /**
   * @brief HACK?
   */
  void YouBot_executive::doneEvent()
  {
    events.write(make_event(m_events, "executive.e_done"));
  }

}

ORO_CREATE_COMPONENT( YouBot::YouBot_executive)

