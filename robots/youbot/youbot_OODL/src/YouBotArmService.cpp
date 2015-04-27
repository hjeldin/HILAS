#include "YouBotArmService.hpp"


namespace YouBot
{
extern unsigned int non_errors;

YouBotArmService::YouBotArmService(const string& name, TaskContext* parent):
Hilas::IRobotArmService(name, parent, YouBot::NR_OF_ARM_SLAVES, 15, YouBot::JOINT_ARM_NAME_ARRAY,-1), m_min_slave_nr(1)
{
	memset(m_overcurrent, 0, YouBot::NR_OF_ARM_SLAVES);
	memset(m_undervoltage, 0, YouBot::NR_OF_ARM_SLAVES);
	memset(m_overvoltage, 0, YouBot::NR_OF_ARM_SLAVES);
	memset(m_overtemperature, 0, YouBot::NR_OF_ARM_SLAVES);
	memset(m_connectionlost, 0, YouBot::NR_OF_ARM_SLAVES);
	memset(m_i2texceeded, 0, YouBot::NR_OF_ARM_SLAVES);
	memset(m_timeout, 0, YouBot::NR_OF_ARM_SLAVES);
}

YouBotArmService::~YouBotArmService()
{
	delete m_manipulator;    
}

void YouBotArmService::displayMotorStatuses()
{
	unsigned int tmp = 0;

	for (unsigned int joint = 0; joint < YouBot::NR_OF_ARM_SLAVES; ++joint)
	{
		m_joints[joint]->getStatus(tmp);
		log(Info) << "Joint[" << joint + 1 << "] is " << tmp << endlog();
	}
}

void YouBotArmService::clearControllerTimeouts()
{
	unsigned int tmp = 0;
	for (unsigned int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
	{
		m_joints[i]->getStatus(tmp);
		if (tmp & youbot::TIMEOUT)
		{
			ClearMotorControllerTimeoutFlag clearTimeoutFlag;
			m_joints[i]->setConfigurationParameter(clearTimeoutFlag);
		}
	}
}

void YouBotArmService::readJointStates()
{
	// YouBot -> OutputPort
	JointSensedAngle joint_angle;
	JointSensedVelocity joint_velocity;
	JointSensedTorque joint_torque;

	m_joint_state.header.stamp = ros::Time(RTT::os::TimeService::Instance()->getNSecs() * 1e9, RTT::os::TimeService::Instance()->getNSecs());

	for (int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
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

void YouBotArmService::updateJointSetpoints()
{
	// InputPort -> YouBot
	joint_position_command.read(m_joint_position_command);
	joint_velocity_command.read(m_joint_velocity_command);
	joint_effort_command.read(m_joint_effort_command);

	// Update joint setpoints
	for (unsigned int joint_nr = 0; joint_nr < YouBot::NR_OF_ARM_SLAVES; ++joint_nr)
	{
		assert(joint_nr < YouBot::NR_OF_ARM_SLAVES);

		switch (m_joint_ctrl_modes[joint_nr])
		{
			case(Hilas::PLANE_ANGLE):
			{
				m_tmp_joint_position_command.angle = m_joint_position_command.positions[joint_nr] * si::radian;
    			// below limits
				if (m_tmp_joint_position_command.angle < m_joint_limits[joint_nr].min_angle * si::radian)
				{
					m_tmp_joint_position_command.angle = m_joint_limits[joint_nr].min_angle * si::radian;
				}
    			// above limits
				else if (m_tmp_joint_position_command.angle > m_joint_limits[joint_nr].max_angle * si::radian)
				{
					m_tmp_joint_position_command.angle = m_joint_limits[joint_nr].max_angle * si::radian;
				}
				m_joints[joint_nr]->setData(m_tmp_joint_position_command);
				break;
			}
			case (Hilas::ANGULAR_VELOCITY):
			{
				m_tmp_joint_cmd_velocity.angularVelocity = m_joint_velocity_command.velocities[joint_nr] * si::radian_per_second;
				m_joints[joint_nr]->setData(m_tmp_joint_cmd_velocity);
				break;
			}
			case (Hilas::TORQUE):
			{
				m_tmp_joint_cmd_torque.torque = m_joint_effort_command.efforts[joint_nr] * si::newton_meter;
				m_joints[joint_nr]->setData(m_tmp_joint_cmd_torque);
				break;
			}
			case (Hilas::MOTOR_STOP):
			{
				m_joints[joint_nr]->stopJoint();
				break;
			}
			default:
			{
				log(Error) << "ctrl_mode not recognized." << endlog();
				break;
			}
		}
	}
}

void YouBotArmService::checkMotorStatuses(){}

bool YouBotArmService::calibrate()
{
	log(Info) << "Calibrating YouBotArmService" << endlog();
	if (m_calibrated)
	{
		log(Info) << "Already calibrated." << endlog();
		return m_calibrated;
	}

	//@todo What about 2 arms?
	try
	{
		m_manipulator = new YouBotManipulator("/youbot-manipulator", OODL_YOUBOT_CONFIG_DIR);
		if (m_manipulator == NULL)
		{
			log(Error) << "Could not create the YouBotManipulator." << endlog();
			return false;
		}

		m_manipulator->doJointCommutation();
		m_manipulator->calibrateManipulator();

		for (unsigned int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
		{
   			m_joints[i] = &(m_manipulator->getArmJoint(m_min_slave_nr + i)); //get the appropriate arm slave.
		}

		for (unsigned int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
		{
			youbot::JointLimits lim;
			m_joints[i]->getConfigurationParameter(lim);
			int lower_limit, upper_limit;
			bool limits_active;
			lim.getParameter(lower_limit, upper_limit, limits_active);

			if (!limits_active)
			{
				log(Error) << "JointLimits are not active, cannot function like this." << endlog();
				return false;
			}

			EncoderTicksPerRound enc;
			m_joints[i]->getConfigurationParameter(enc);

			unsigned int ticks_per_round(0);
			enc.getParameter(ticks_per_round);

			GearRatio gRatio;
			double gearRatio;

			m_joints[i]->getConfigurationParameter(gRatio);
			gRatio.getParameter(gearRatio);

			m_joint_limits[i].min_angle = ((double) lower_limit / ticks_per_round) * gearRatio * (2.0 * M_PI);
			m_joint_limits[i].max_angle = ((double) upper_limit / ticks_per_round) * gearRatio * (2.0 * M_PI);

			InverseMovementDirection invMov;
			m_joints[i]->getConfigurationParameter(invMov);

			bool invMov2(false);
			invMov.getParameter(invMov2);

			if(invMov2)
			{
				quantity<plane_angle> tmp = m_joint_limits[i].min_angle * si::radian;
				m_joint_limits[i].min_angle = -m_joint_limits[i].max_angle * 1.001;
				m_joint_limits[i].max_angle = -tmp.value() * 0.999;
			}
			else
			{
				m_joint_limits[i].min_angle *= 0.999;
				m_joint_limits[i].max_angle *= 1.001;
			}

			log(Info) << "Min angle: " << m_joint_limits[i].min_angle << " Max angle: " << m_joint_limits[i].max_angle << endlog();
		}

	}
	catch (std::exception& e)
	{
		log(Error) << e.what() << endlog();
		m_manipulator = NULL;
		this->getOwner()->error();
		return false;
	}

	log(Info) << "Calibrated." << endlog();
	return (m_calibrated = true);
}

void YouBotArmService::stop()
{
	for (unsigned int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
	{
		m_joints[i]->stopJoint();
	}
}

void YouBotArmService::cleanup()
{
	for (unsigned int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
	{
		m_joints[i] = NULL;
	}

	delete m_manipulator;
	m_manipulator = NULL;
	m_calibrated = false;
}

}
