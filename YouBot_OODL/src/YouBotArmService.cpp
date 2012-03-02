#include "YouBotArmService.hpp"

#include <stdio.h>
#include <cassert>

#include "YouBotHelpers.hpp"
#include <youbot/ProtocolDefinitions.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	extern unsigned int non_errors;

	YouBotArmService::YouBotArmService(const string& name, TaskContext* parent, unsigned int min_slave_nr) :
    Service(name, parent),
    m_joint_limits(NR_OF_ARM_SLAVES),
    m_joint_ctrl_modes(NR_OF_ARM_SLAVES, MOTOR_STOP),

    // Set the commands to zero depending on the number of joints
    m_calibrated(false),
    m_min_slave_nr(min_slave_nr)
	{
	  m_OODL = (YouBotOODL*)parent;

		m_joint_states.name.assign(NR_OF_ARM_SLAVES,"");
		m_joint_states.name[0]="arm_joint_1";
		m_joint_states.name[1]="arm_joint_2";
		m_joint_states.name[2]="arm_joint_3";
		m_joint_states.name[3]="arm_joint_4";
		m_joint_states.name[4]="arm_joint_5";

		m_joint_states.position.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.velocity.assign(NR_OF_ARM_SLAVES,0);
		m_joint_states.effort.assign(NR_OF_ARM_SLAVES,0);

		m_joint_cmd_angles.positions.assign(NR_OF_ARM_SLAVES,0);
		m_joint_cmd_velocities.velocities.assign(NR_OF_ARM_SLAVES,0);
		m_joint_cmd_torques.efforts.assign(NR_OF_ARM_SLAVES,0);

    // Pre-allocate port memory for outputs
    joint_states.setDataSample( m_joint_states );

    // set to false
		memset(m_overcurrent, 0, NR_OF_ARM_SLAVES);
		memset(m_undervoltage, 0, NR_OF_ARM_SLAVES);
		memset(m_overvoltage, 0, NR_OF_ARM_SLAVES);
		memset(m_overtemperature, 0, NR_OF_ARM_SLAVES);
		memset(m_connectionlost, 0, NR_OF_ARM_SLAVES);
		memset(m_i2texceeded, 0, NR_OF_ARM_SLAVES);
		memset(m_timeout, 0, NR_OF_ARM_SLAVES);

		setupComponentInterface();
	}

	YouBotArmService::~YouBotArmService()
	{
		delete m_manipulator;
	}

	void YouBotArmService::setupComponentInterface()
	{
		this->addPort("joint_states",joint_states).doc("Joint states");

		this->addPort("joint_cmd_angles",joint_cmd_angles).doc("Command joint angles");
		this->addPort("joint_cmd_velocities",joint_cmd_velocities).doc("Command joint velocities");
		this->addPort("joint_cmd_torques",joint_cmd_torques).doc("Command joint torques");

    this->addOperation("start",&YouBotArmService::start,this);
    this->addOperation("update",&YouBotArmService::update,this);
    this->addOperation("calibrate",&YouBotArmService::calibrate,this);
    this->addOperation("stop",&YouBotArmService::stop,this);
    this->addOperation("cleanup",&YouBotArmService::cleanup,this);

    this->addOperation("setControlModes",&YouBotArmService::setControlModes,this, OwnThread);
    this->addOperation("getControlModes",&YouBotArmService::getControlModes,this, OwnThread);
    this->addOperation("displayMotorStatuses",&YouBotArmService::displayMotorStatuses,this, OwnThread);
    this->addOperation("clearControllerTimeouts",&YouBotArmService::clearControllerTimeouts,this, OwnThread);
	}

	void YouBotArmService::setControlModes(vector<ctrl_modes>& all)
	{
		m_joint_ctrl_modes = all;
	}

	void YouBotArmService::getControlModes(vector<ctrl_modes>& all)
	{
		all = m_joint_ctrl_modes;
	}

	void YouBotArmService::displayMotorStatuses()
	{
    unsigned int tmp = 0;;
    for(unsigned int joint = 0; joint < NR_OF_ARM_SLAVES; ++joint)
    {
      m_joints[joint]->getStatus(tmp);
      log(Info) << "Joint[" << joint+1 << "] is " << motor_status_tostring(tmp) << endlog();
    }
	}

	bool YouBotArmService::start()
	{
		if(m_calibrated)
		{
			clearControllerTimeouts();
			return true;
		}
		else
		{
			return false;
		}
	}

	void YouBotArmService::clearControllerTimeouts()
	{
	  unsigned int tmp = 0;
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i]->getStatus(tmp);
			if( tmp & ::TIMEOUT )
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

		m_joint_states.header.stamp = ros::Time::now();

		for(int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
		  m_joints[i]->getData(joint_angle);
			m_joint_states.position[i] = joint_angle.angle.value();

			m_joints[i]->getData(joint_velocity);
			m_joint_states.velocity[i] = joint_velocity.angularVelocity.value();

			m_joints[i]->getData(joint_torque);
			m_joint_states.effort[i] = sign(m_joint_states.velocity[i]) * joint_torque.torque.value();
		}

		joint_states.write(m_joint_states);
	}

	void YouBotArmService::updateJointSetpoints()
	{
		// InputPort -> YouBot
		joint_cmd_angles.read(m_joint_cmd_angles);
		joint_cmd_velocities.read(m_joint_cmd_velocities);
		joint_cmd_torques.read(m_joint_cmd_torques);

		// Update joint setpoints
		for(unsigned int joint_nr = 0; joint_nr < NR_OF_ARM_SLAVES; ++joint_nr)
		{
			assert(joint_nr < NR_OF_ARM_SLAVES);

			switch(m_joint_ctrl_modes[joint_nr])
			{
				case(PLANE_ANGLE):
				{
					m_tmp_joint_cmd_angle.angle = m_joint_cmd_angles.positions[joint_nr] * si::radian;
					// below limits
					if(m_tmp_joint_cmd_angle.angle < m_joint_limits[joint_nr].min_angle)
					{
						m_tmp_joint_cmd_angle.angle = m_joint_limits[joint_nr].min_angle;
					}
					// above limits
					else if(m_tmp_joint_cmd_angle.angle > m_joint_limits[joint_nr].max_angle)
					{
						m_tmp_joint_cmd_angle.angle = m_joint_limits[joint_nr].max_angle;
					}
					m_joints[joint_nr]->setData(m_tmp_joint_cmd_angle);
					break;
				}
				case(ANGULAR_VELOCITY):
				{
					m_tmp_joint_cmd_velocity.angularVelocity = m_joint_cmd_velocities.velocities[joint_nr] * si::radian_per_second;
					m_joints[joint_nr]->setData(m_tmp_joint_cmd_velocity);
					break;
				}
				case(TORQUE):
				{
					m_tmp_joint_cmd_torque.torque = m_joint_cmd_torques.efforts[joint_nr] * si::newton_meter;
					m_joints[joint_nr]->setData(m_tmp_joint_cmd_torque);
					break;
				}
				case(MOTOR_STOP):
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

	void YouBotArmService::checkMotorStatuses()
	{
	  unsigned int tmp = 0;;
		for(unsigned int joint = 0; joint < NR_OF_ARM_SLAVES; ++joint)
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

//			CHECK_EVENT_LEVEL(::ENCODER_ERROR, E_ENCODER_ERR)

//			CHECK_EVENT_LEVEL(::, E_SINE_COMM_INIT_ERR)

//			CHECK_EVENT_LEVEL(::EMERGENCY_STOP, E_EMERGENCY_STOP)
		}
	}

	void YouBotArmService::update()
	{
		readJointStates();

		updateJointSetpoints();

		checkMotorStatuses();
	}

	bool YouBotArmService::calibrate()
	{
		log(Info) << "Calibrating YouBotArmService" << endlog();
		if(m_calibrated)
		{
			log(Info) << "Already calibrated." << endlog();
			return m_calibrated;
		}

		//@todo What about 2 arms?
		try
		{
			m_manipulator = new YouBotManipulator("/youbot-manipulator", OODL_YOUBOT_CONFIG_DIR);
			if(m_manipulator == NULL)
			{
				log(Error) << "Could not create the YouBotManipulator." << endlog();
				return false;
			}

			m_manipulator->doJointCommutation();
			m_manipulator->calibrateManipulator();

			for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
			{
				//@todo Fixme: m_min_slave_nr
				m_joints[i] = &(m_manipulator->getArmJoint(m_min_slave_nr + i)); //get the appropriate arm slave.
//				m_joint_cmd =
			}

			// Determine JointLimit's -> WORKAROUND to prevent exceptions!
			for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
			{
				// position limits
				youbot::JointLimits lim;
				m_joints[i]->getConfigurationParameter(lim);
				int lower_limit, upper_limit;
				bool limits_active;
				lim.getParameter(lower_limit, upper_limit, limits_active);
				if(!limits_active)
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
				m_joint_limits[i].min_angle = ((double) lower_limit / ticks_per_round) * gearRatio * (2.0 * M_PI) * radian;
				m_joint_limits[i].max_angle = ((double) upper_limit / ticks_per_round) * gearRatio * (2.0 * M_PI) * radian;

				InverseMovementDirection invMov;
				m_joints[i]->getConfigurationParameter(invMov);
				bool invMov2(false);
				invMov.getParameter(invMov2);
				if(invMov2) //@todo: strange!!
				{
					quantity<plane_angle> tmp = m_joint_limits[i].min_angle;
					m_joint_limits[i].min_angle = -m_joint_limits[i].max_angle * 1.001;
					m_joint_limits[i].max_angle = -tmp * 0.999;
				}
				else
				{
					// OODL uses value < max/min instead of <=
					m_joint_limits[i].min_angle *= 0.999;
					m_joint_limits[i].max_angle *= 1.001;
				}

				log(Info) << "Min angle: " << m_joint_limits[i].min_angle << " Max angle: " << m_joint_limits[i].max_angle << endlog();

				// velocity limits
				// There are no velocity limits at the moment!

				// torque limits
				// There are no current limits at the moment!


			}

		}
		catch (std::exception& e)
		{
			log(Error) << e.what();
			m_manipulator = NULL;
			this->getOwner()->error();
			return false;
		}

		log(Info) << "Calibrated." << endlog();
		return (m_calibrated = true);
	}

	void YouBotArmService::stop()
	{
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i]->stopJoint();
		}
	}

	void YouBotArmService::cleanup()
	{
		for(unsigned int i = 0; i < NR_OF_ARM_SLAVES; ++i)
		{
			m_joints[i] = NULL;
		}
		delete m_manipulator;
		m_manipulator = NULL;
		m_calibrated = false;
	}

}
