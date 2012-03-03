#include "YouBotBaseService.hpp"

#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>
#include <base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp>

#include "YouBotHelpers.hpp"

#include <tf/tf.h>

namespace YouBot
{
  using namespace RTT;
  using namespace RTT::types;
  using namespace std;

  extern unsigned int non_errors;

  YouBotBaseService::YouBotBaseService(const string& name, TaskContext* parent,
      unsigned int min_slave_nr) :
      Service(name, parent), m_joint_ctrl_modes(NR_OF_BASE_SLAVES, MOTOR_STOP),
      // Set the commands to zero depending on the number of joints
      m_calibrated(false), m_min_slave_nr(min_slave_nr)
  {
    m_OODL = (YouBotOODL*) parent;

    m_joint_states.position.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_states.velocity.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_states.effort.assign(NR_OF_BASE_SLAVES, 0);

    m_joint_cmd_angles.positions.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_cmd_velocities.velocities.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_cmd_torques.efforts.assign(NR_OF_BASE_SLAVES, 0);

    // Pre-allocate port memory for outputs
    joint_states.setDataSample(m_joint_states);
    odometry_state.setDataSample(m_odometry_state);

    // odometry pose estimates frame
    m_odometry_state.header.frame_id = "odometry";
    m_odometry_state.header.seq = 0;
    // odometry twist estimates frame
    m_odometry_state.child_frame_id = "base_link";

    // odometry estimates - set to zero
    m_odometry_state.pose.pose.position.x = 0;
    m_odometry_state.pose.pose.position.y = 0;
    m_odometry_state.pose.pose.position.z = 0;
    m_odometry_state.pose.pose.orientation.x = 0;
    m_odometry_state.pose.pose.orientation.y = 0;
    m_odometry_state.pose.pose.orientation.z = 0;
    m_odometry_state.pose.pose.orientation.w = 0;
    m_odometry_state.twist.twist.linear.x = 0;
    m_odometry_state.twist.twist.linear.y = 0;
    m_odometry_state.twist.twist.linear.z = 0;
    m_odometry_state.twist.twist.angular.x = 0;
    m_odometry_state.twist.twist.angular.y = 0;
    m_odometry_state.twist.twist.angular.z = 0;

    // set to false
    memset(m_overcurrent, 0, NR_OF_BASE_SLAVES);
    memset(m_undervoltage, 0, NR_OF_BASE_SLAVES);
    memset(m_overvoltage, 0, NR_OF_BASE_SLAVES);
    memset(m_overtemperature, 0, NR_OF_BASE_SLAVES);
    memset(m_connectionlost, 0, NR_OF_BASE_SLAVES);
    memset(m_i2texceeded, 0, NR_OF_BASE_SLAVES);
    memset(m_timeout, 0, NR_OF_BASE_SLAVES);

    setupComponentInterface();
  }

  YouBotBaseService::~YouBotBaseService()
  {
    delete m_base;
  }

  void YouBotBaseService::setupComponentInterface()
  {
    this->addPort("joint_states", joint_states).doc("Joint states");
    this->addPort("odometry_state", odometry_state).doc("Base odometry");

    this->addPort("joint_cmd_angles", joint_cmd_angles).doc(
        "Command joint angles");
    this->addPort("joint_cmd_velocities", joint_cmd_velocities).doc(
        "Command joint velocities");
    this->addPort("joint_cmd_torques", joint_cmd_torques).doc(
        "Command joint torques");

    this->addPort("cmd_twist", cmd_twist).doc("Command base twist");

    this->addOperation("start", &YouBotBaseService::start, this);
    this->addOperation("update", &YouBotBaseService::update, this);
    this->addOperation("calibrate", &YouBotBaseService::calibrate, this);
    this->addOperation("stop", &YouBotBaseService::stop, this);
    this->addOperation("cleanup", &YouBotBaseService::cleanup, this);

    this->addOperation("setControlModes", &YouBotBaseService::setControlModes,
        this, OwnThread);
    this->addOperation("getControlModes", &YouBotBaseService::getControlModes,
        this, OwnThread);

    this->addOperation("displayMotorStatuses",
        &YouBotBaseService::displayMotorStatuses, this, OwnThread);
    this->addOperation("clearControllerTimeouts",
        &YouBotBaseService::clearControllerTimeouts, this, OwnThread);
  }

  void YouBotBaseService::getControlModes(vector<ctrl_modes>& all)
  {
    all = m_joint_ctrl_modes;
  }

  void YouBotBaseService::setControlModes(vector<ctrl_modes>& all)
  {
    if (all.size() != NR_OF_BASE_SLAVES)
    {
      log(Error) << "The number of ctrl_modes should match the number of motors."
          << endlog();
      this->getOwner()->error();
      return;
    }

    // If one ctrl_mode is TWIST, check to see if all ctrl_modes are set to TWIST.
    bool twist(false);
    for (unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {
      if (all[i] == TWIST && !twist)
      {
        twist = true;
        if (i != 0)
        {
          this->getOwner()->error();
          log(Error) << "If the ctrl_mode TWIST is used, all "
              << NR_OF_BASE_SLAVES << " motors should be set to this!"
              << endlog();
          return;
        }
      }
      else if (twist && all[i] != TWIST)
      {
        this->getOwner()->error();
        log(Error) << "If the ctrl_mode TWIST is used, all " << NR_OF_BASE_SLAVES
            << " motors should be set to this!" << endlog();
        return;
      }
    }

    m_joint_ctrl_modes = all;
  }

  bool YouBotBaseService::start()
  {
    return m_calibrated;
  }

  void YouBotBaseService::setTwistSetpoints()
  {
    cmd_twist.read(m_cmd_twist);

    quantity < si::velocity > longitudinalVelocity = m_cmd_twist.linear.x
        * si::meter_per_second;
    quantity < si::velocity > transversalVelocity = m_cmd_twist.linear.y
        * si::meter_per_second;
    quantity < si::angular_velocity > angularVelocity = m_cmd_twist.angular.z
        * si::radian_per_second;

    m_base->setBaseVelocity(longitudinalVelocity, transversalVelocity,
        angularVelocity);

  //		std::vector<quantity<angular_velocity> > wheelVelocities(NR_OF_BASE_SLAVES, 0);
  //
  //		m_kinematics.cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, wheelVelocities);
  //
  //		for(unsigned int joint_nr = 0; joint_nr < NR_OF_BASE_SLAVES; ++joint_nr)
  //		{
  //			m_tmp_joint_cmd_velocity.angularVelocity = wheelVelocities[joint_nr];
  //			m_joints[joint_nr]->setData(m_tmp_joint_cmd_velocity);
  //		}
  }

  void YouBotBaseService::setJointSetpoints()
  {
    joint_cmd_angles.read(m_joint_cmd_angles);
    joint_cmd_velocities.read(m_joint_cmd_velocities);
    joint_cmd_torques.read(m_joint_cmd_torques);

    for (unsigned int joint_nr = 0; joint_nr < NR_OF_BASE_SLAVES; ++joint_nr)
    {
      switch (m_joint_ctrl_modes[joint_nr])
      {
      case (PLANE_ANGLE):
      {
        m_tmp_joint_cmd_angle.angle = m_joint_cmd_angles.positions[joint_nr]
            * si::radian;
        m_joints[joint_nr]->setData(m_tmp_joint_cmd_angle);
        break;
      }
      case (ANGULAR_VELOCITY):
      {
        m_tmp_joint_cmd_velocity.angularVelocity =
            m_joint_cmd_velocities.velocities[joint_nr] * si::radian_per_second;
        m_joints[joint_nr]->setData(m_tmp_joint_cmd_velocity);
        break;
      }
      case (TORQUE):
      {
        m_tmp_joint_cmd_torque.torque = m_joint_cmd_torques.efforts[joint_nr]
            * si::newton_meter;
        m_joints[joint_nr]->setData(m_tmp_joint_cmd_torque);
        break;
      }
      case (MOTOR_STOP):
      {
        m_joints[joint_nr]->stopJoint();
        break;
      }
      case (TWIST):
      {
        log(Error) << "Cannot be in TWIST ctrl_mode (programming error)"
            << endlog();
        this->getOwner()->error();
        break;
      }
      default:
      {
        log(Error) << "Case not recognized." << endlog();
        this->getOwner()->error();
        break;
      }
      }
    }
  }

  void YouBotBaseService::readJointStates()
  {
    m_joint_states.header.stamp = ros::Time::now();

    // YouBot -> OutputPort
    JointSensedAngle joint_angle;
    JointSensedVelocity joint_velocity;
    JointSensedTorque joint_torque;

    for (int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i]->getData(joint_angle);
      m_joint_states.position[i] = joint_angle.angle.value();

      m_joints[i]->getData(joint_velocity);
      m_joint_states.velocity[i] = joint_velocity.angularVelocity.value();

      m_joints[i]->getData(joint_torque);
      m_joint_states.effort[i] = sign(m_joint_states.velocity[i])
          * joint_torque.torque.value();
    }

    joint_states.write(m_joint_states);
  }

  void YouBotBaseService::readOdometry()
  {
    m_odometry_state.header.stamp = ros::Time::now();

    quantity < si::velocity > longitudinalVelocity;
    quantity < si::velocity > transversalVelocity;
    quantity < angular_velocity > angularVelocity;

    m_base->getBaseVelocity(longitudinalVelocity, transversalVelocity,
        angularVelocity);

    m_odometry_state.twist.twist.linear.x = longitudinalVelocity.value();
    m_odometry_state.twist.twist.linear.y = transversalVelocity.value();
    //		m_odometry_state.twist.twist.linear.z = 0;
    //		m_odometry_state.twist.twist.angular.x = 0;
    //		m_odometry_state.twist.twist.angular.y = 0;
    m_odometry_state.twist.twist.angular.z = angularVelocity.value();

    quantity < si::length > longitudinalPosition;
    quantity < si::length > transversalPosition;
    quantity < plane_angle > orientation; //yaw

    m_base->getBasePosition(longitudinalPosition, transversalPosition,
        orientation);

    m_odometry_state.pose.pose.position.x = longitudinalPosition.value();
    m_odometry_state.pose.pose.position.y = transversalPosition.value();
    //		m_odometry_state.pose.pose.position.z = 0;

    m_odometry_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        orientation.value());

    odometry_state.write(m_odometry_state);
  }

  void YouBotBaseService::update()
  {
    // Sensors
    if (joint_states.connected()) //Optimization -> converting messages costs a lot of CPU
      readJointStates();

    if (odometry_state.connected()) //Optimization -> calculation uses 14% CPU on youBot
      readOdometry();

    // Actuators
    if (m_joint_ctrl_modes[0] == TWIST) // All joints will be in TWIST ctrl_mode (see setControlModes)
    {
      setTwistSetpoints();
    }
    else
    {
      setJointSetpoints();
    }

    // Check for errors -> events
    checkMotorStatuses();
  }

  void YouBotBaseService::checkMotorStatuses()
  {
    unsigned int tmp = 0;
    for (unsigned int joint = 0; joint < NR_OF_BASE_SLAVES; ++joint)
    {
      m_joints[joint]->getStatus(tmp);

      CHECK_EVENT_EDGE(::OVER_CURRENT, m_overcurrent, E_OVERCURRENT)

      CHECK_EVENT_EDGE(::UNDER_VOLTAGE, m_undervoltage, E_UNDERVOLTAGE)

      CHECK_EVENT_EDGE(::OVER_VOLTAGE, m_overvoltage, E_OVERVOLTAGE)

      CHECK_EVENT_EDGE(::OVER_TEMPERATURE, m_overtemperature, E_OVERTEMP)

  //      CHECK_EVENT_EDGE(::E_EC_CON_LOST, m_connectionlost, E_OVERTEMP)

      CHECK_EVENT_EDGE(::I2T_EXCEEDED, m_i2texceeded, E_I2T_EXCEEDED)

      CHECK_EVENT_EDGE(::TIMEOUT, m_timeout, E_EC_TIMEOUT)

      // level events
      CHECK_EVENT_LEVEL(::HALL_SENSOR_ERROR, E_HALL_ERR)

  //      CHECK_EVENT_LEVEL(::ENCODER_ERROR, E_ENCODER_ERR)

  //      CHECK_EVENT_LEVEL(::, E_SINE_COMM_INIT_ERR)

  //      CHECK_EVENT_LEVEL(::EMERGENCY_STOP, E_EMERGENCY_STOP)
    }
  }

  bool YouBotBaseService::calibrate()
  {
    if (m_calibrated)
    {
      log(Info) << "Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "Calibrating YouBotBaseService..." << endlog();
    try
    {
      m_base = new YouBotBase("/youbot-base", OODL_YOUBOT_CONFIG_DIR);
      if (m_base == NULL)
      {
        log(Error) << "Could not create the YouBotBase." << endlog();
        return false;
      }

      m_base->doJointCommutation();

      for (unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
      {
        //@todo Fixme: m_min_slave_nr
        m_joints[i] = &(m_base->getBaseJoint(m_min_slave_nr + i)); //get the appropriate arm slave.
      }
    } catch (std::exception& e)
    {
      log(Error) << e.what();
      m_base = NULL;
      this->getOwner()->error();
      return false;
    }

    log(Info) << "Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void YouBotBaseService::stop()
  {
    for (unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i]->stopJoint();
    }
  }

  void YouBotBaseService::cleanup()
  {
    for (unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i] = NULL;
    }
    delete m_base;
    m_base = NULL;
    m_calibrated = false;
  }

  void YouBotBaseService::displayMotorStatuses()
  {
    unsigned int tmp = 0;
    for (unsigned int joint = 0; joint < NR_OF_BASE_SLAVES; ++joint)
    {
      m_joints[joint]->getStatus(tmp);
      log(Info) << "Joint[" << joint + 1 << "] is " << motor_status_tostring(tmp)
          << endlog();
    }
  }

  void YouBotBaseService::clearControllerTimeouts()
  {
    unsigned int tmp = 0;
    for (unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i]->getStatus(tmp);
      if (tmp & ::TIMEOUT)
      {
        ClearMotorControllerTimeoutFlag clearTimeoutFlag;
        m_joints[i]->setConfigurationParameter(clearTimeoutFlag);
      }
    }
  }

}
