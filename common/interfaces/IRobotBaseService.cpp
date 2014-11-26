#include "IRobotBaseService.hpp"

namespace Hilas
{

IRobotBaseService::IRobotBaseService(const string& name, TaskContext* parent, int base_slave_count, int robot_slave_count, const std::string base_slave_name[], long m_clientID = -1):
Service(name, parent), NR_OF_BASE_SLAVES(base_slave_count), m_joint_ctrl_modes(base_slave_count, MOTOR_STOP), m_calibrated(false), clientID(m_clientID)
{
	for (int i = 0; i < base_slave_count; ++i)
	{
		m_joint_state.name.push_back(base_slave_name[i]);
	}

	m_joint_state.position.assign(NR_OF_BASE_SLAVES, 0);
	m_joint_state.velocity.assign(NR_OF_BASE_SLAVES, 0);
	m_joint_state.effort.assign(NR_OF_BASE_SLAVES, 0);

	m_joint_position_command.positions.assign(robot_slave_count, 0);
	m_joint_velocity_command.velocities.assign(robot_slave_count, 0);
	m_joint_effort_command.efforts.assign(robot_slave_count, 0);

	// Pre-allocate port memory for outputs
	joint_state.setDataSample(m_joint_state);
	odometry_state.setDataSample(m_odometry_state);

	// Events - Pre-allocate port memory for outputs
	m_events.reserve(max_event_length);
	events.setDataSample(m_events);

	// odometry pose estimates frame
	m_odometry_state.header.frame_id = "odom";
	m_odometry_state.header.seq = 0;

	// odometry twist estimates frame
	m_odometry_state.child_frame_id = "base_footprint";

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

	setupComponentInterface();
}

IRobotBaseService::~IRobotBaseService(){}

void IRobotBaseService::setupComponentInterface()
{
	this->addPort("joint_state_out", joint_state).doc("Joint states");
	this->addPort("odometry_state_out", odometry_state).doc("Base odometry");

	this->addPort("joint_position_command_in", joint_position_command).doc("Command joint angles");
	this->addPort("joint_velocity_command_in", joint_velocity_command).doc("Command joint velocities");
	this->addPort("joint_effort_command_in", joint_effort_command).doc("Command joint torques");

	this->addPort("cmd_twist_in", cmd_twist).doc("Command base twist");
	this->addPort("events", events).doc("Joint events");

    this->addPort("joint_state_in", in_joint_state).doc("Joint states from HW or SIM");
    this->addPort("odometry_state_in", in_odometry_state).doc("Base odometry from HW or SIM");

	this->addOperation("start", &IRobotBaseService::start, this);
	this->addOperation("update", &IRobotBaseService::update, this);
	this->addOperation("calibrate", &IRobotBaseService::calibrate, this);
	this->addOperation("stop", &IRobotBaseService::stop, this);
	this->addOperation("cleanup", &IRobotBaseService::cleanup, this);

	this->addOperation("setControlModesAll", &IRobotBaseService::setControlModesAll, this, OwnThread).doc("Control modes can be set individually.");
	this->addOperation("setControlModes", &IRobotBaseService::setControlModes, this, OwnThread);
	this->addOperation("getControlModes", &IRobotBaseService::getControlModes, this, OwnThread);
	this->addOperation("displayMotorStatuses", &IRobotBaseService::displayMotorStatuses, this, OwnThread);
	this->addOperation("clearControllerTimeouts", &IRobotBaseService::clearControllerTimeouts, this, OwnThread);
	this->addOperation("setSimMode", &IRobotBaseService::setsim_mode,this, OwnThread).doc("Set simulation mode.");   

}

void IRobotBaseService::setsim_mode(int mode)
{
	switch(mode)
	{
		case 1:
		is_in_visualization_mode = true;
		break;

		case 2:
		is_in_visualization_mode = false;
		break;

		default: break;
	}
}

void IRobotBaseService::setControlModes(vector<ctrl_modes>& all)
{
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
				log(Error) << "If the ctrl_mode TWIST is used, all " << NR_OF_BASE_SLAVES << " motors should be set to this!" << endlog();
				return;
			}
		}
		else if (twist && all[i] != TWIST)
		{
			this->getOwner()->error();
			log(Error) << "If the ctrl_mode TWIST is used, all " << NR_OF_BASE_SLAVES << " motors should be set to this!" << endlog();
			return;
		}
	}

	m_joint_ctrl_modes = all;
}

void IRobotBaseService::setControlModesAll(int mode)
{
	for(int i=0;i<NR_OF_BASE_SLAVES;++i)
	{
		m_joint_ctrl_modes[i] = static_cast<ctrl_modes>(mode);
	}
}

void IRobotBaseService::getControlModes(vector<ctrl_modes>& all)
{
	all = m_joint_ctrl_modes;
}	

void IRobotBaseService::displayMotorStatuses()
{
	std::cout << "Not yet implemented" << std::endl;
}

void IRobotBaseService::clearControllerTimeouts()
{
	std::cout << "Not yet implemented" << std::endl;
}

void IRobotBaseService::checkMotorStatuses(){}

bool IRobotBaseService::start()
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

void IRobotBaseService::update()
{
	if(joint_state.connected())
	{
		readJointStates();      
	}

	if(odometry_state.connected())
	{
		readOdometry();      
	}

	if (m_joint_ctrl_modes[0] == TWIST)
	{
		setTwistSetpoints();
	}
	else
	{
		setJointSetpoints();
	}
	checkMotorStatuses();
}

bool IRobotBaseService::calibrate()
{
	std::cout << "Not yet implemented, setting to true" << std::endl;

	m_calibrated = true;
	return true;
}

void IRobotBaseService::stop()
{
	std::cout << "Not yet implemented" << std::endl;
}

void IRobotBaseService::cleanup()
{
	std::cout << "Not yet implemented" << std::endl;
}

}