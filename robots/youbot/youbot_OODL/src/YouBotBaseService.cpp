#include "YouBotBaseService.hpp"

namespace YouBot
{

  YouBotBaseService::YouBotBaseService(const string& name, TaskContext* parent):
  Hilas::IRobotBaseService(name, parent, YouBot::NR_OF_BASE_SLAVES, 15, YouBot::JOINT_BASE_NAME_ARRAY,-1), m_min_slave_nr(1)
  {
    memset(m_overcurrent, 0, YouBot::NR_OF_BASE_SLAVES);
    memset(m_undervoltage, 0, YouBot::NR_OF_BASE_SLAVES);
    memset(m_overvoltage, 0, YouBot::NR_OF_BASE_SLAVES);
    memset(m_overtemperature, 0, YouBot::NR_OF_BASE_SLAVES);
    memset(m_connectionlost, 0, YouBot::NR_OF_BASE_SLAVES);
    memset(m_i2texceeded, 0, YouBot::NR_OF_BASE_SLAVES);
    memset(m_timeout, 0, YouBot::NR_OF_BASE_SLAVES);
  }

  YouBotBaseService::~YouBotBaseService()
  {
    delete m_base;
  }

  void YouBotBaseService::displayMotorStatuses()
  {
    unsigned int tmp = 0;
    for(unsigned int joint = 0; joint < YouBot::NR_OF_BASE_SLAVES; ++joint)
    {
      m_joints[joint]->getStatus(tmp);
      log(Info) << "Joint[" << joint + 1 << "] is " << tmp << endlog();
    }
  }

  void YouBotBaseService::clearControllerTimeouts()
  {
    unsigned int tmp = 0;
    for (unsigned int i = 0; i < YouBot::NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i]->getStatus(tmp);
      if (tmp & youbot::TIMEOUT)
      {
        ClearMotorControllerTimeoutFlag clearTimeoutFlag;
        m_joints[i]->setConfigurationParameter(clearTimeoutFlag);
      }
    }
  }  

  void YouBotBaseService::setTwistSetpoints()
  {
    cmd_twist.read(m_cmd_twist);

    quantity < si::velocity > longitudinalVelocity = m_cmd_twist.linear.x * si::meter_per_second;
    quantity < si::velocity > transversalVelocity = m_cmd_twist.linear.y * si::meter_per_second;
    quantity < si::angular_velocity > angularVelocity = m_cmd_twist.angular.z * si::radian_per_second;

    m_base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
  }

  void YouBotBaseService::setJointSetpoints()
  {
    joint_position_command.read(m_joint_position_command);
    joint_velocity_command.read(m_joint_velocity_command);
    joint_effort_command.read(m_joint_effort_command);

    for (unsigned int joint_nr = 0; joint_nr < YouBot::NR_OF_BASE_SLAVES; ++joint_nr)
    {
      switch (m_joint_ctrl_modes[joint_nr])
      {
      case (Hilas::PLANE_ANGLE):
      {
        m_tmp_joint_position_command.angle = m_joint_position_command.positions[joint_nr] * si::radian;
        m_joints[joint_nr]->setData(m_tmp_joint_position_command);
        break;
      }
      case (Hilas::ANGULAR_VELOCITY):
      {
        m_tmp_joint_velocity_command.angularVelocity = m_joint_velocity_command.velocities[joint_nr] * si::radian_per_second;
        m_joints[joint_nr]->setData(m_tmp_joint_velocity_command);
        break;
      }
      case (Hilas::TORQUE):
      {
        m_tmp_joint_effort_command.torque = m_joint_effort_command.efforts[joint_nr] * si::newton_meter;
        m_joints[joint_nr]->setData(m_tmp_joint_effort_command);
        break;
      }
      case (Hilas::MOTOR_STOP):
      {
        m_joints[joint_nr]->stopJoint();
        break;
      }
      case (Hilas::TWIST):
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
    m_joint_state.header.stamp = ros::Time(RTT::os::TimeService::Instance()->getNSecs() * 1e9, RTT::os::TimeService::Instance()->getNSecs());

    JointSensedAngle joint_angle;
    JointSensedVelocity joint_velocity;
    JointSensedTorque joint_torque;

    for (int i = 0; i < YouBot::NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i]->getData(joint_angle);
      m_joint_state.position[i] = joint_angle.angle.value();

      m_joints[i]->getData(joint_velocity);
      m_joint_state.velocity[i] = joint_velocity.angularVelocity.value();

      m_joints[i]->getData(joint_torque);
      m_joint_state.effort[i] = joint_torque.torque.value();
    }

    joint_state.write(m_joint_state);
  }

  void YouBotBaseService::readOdometry()
  {
    m_odometry_state.header.stamp = ros::Time(RTT::os::TimeService::Instance()->getNSecs() * 1e9, RTT::os::TimeService::Instance()->getNSecs());

    quantity < si::velocity > longitudinalVelocity;
    quantity < si::velocity > transversalVelocity;
    quantity < angular_velocity > angularVelocity;

    m_base->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);

    m_odometry_state.twist.twist.linear.x = longitudinalVelocity.value();
    m_odometry_state.twist.twist.linear.y = transversalVelocity.value();
    //		m_odometry_state.twist.twist.linear.z = 0;
    //		m_odometry_state.twist.twist.angular.x = 0;
    //		m_odometry_state.twist.twist.angular.y = 0;
    m_odometry_state.twist.twist.angular.z = angularVelocity.value();

    quantity < si::length > longitudinalPosition;
    quantity < si::length > transversalPosition;
    quantity < plane_angle > orientation; //yaw

    m_base->getBasePosition(longitudinalPosition, transversalPosition, orientation);

    m_odometry_state.pose.pose.position.x = longitudinalPosition.value();
    m_odometry_state.pose.pose.position.y = transversalPosition.value();
    //		m_odometry_state.pose.pose.position.z = 0;

    m_odometry_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation.value());

    odometry_state.write(m_odometry_state);
  }

  void YouBotBaseService::checkMotorStatuses(){}

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

      for (unsigned int i = 0; i < YouBot::NR_OF_BASE_SLAVES; ++i)
      {
        m_joints[i] = &(m_base->getBaseJoint(m_min_slave_nr + i)); //get the appropriate arm slave.
      }
    }
    catch (std::exception& e)
    {
      log(Error) << e.what() << endlog();
      m_base = NULL;
      this->getOwner()->error();
      return false;
    }

    log(Info) << "Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void YouBotBaseService::stop()
  {
    for (unsigned int i = 0; i < YouBot::NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i]->stopJoint();
    }
  }

  void YouBotBaseService::cleanup()
  {
    for (unsigned int i = 0; i < YouBot::NR_OF_BASE_SLAVES; ++i)
    {
      m_joints[i] = NULL;
    }
    delete m_base;
    m_base = NULL;
    m_calibrated = false;
  }
}