#include "arm_bridge_ros_orocos.hpp"

ArmBridgeRosOrocos::ArmBridgeRosOrocos(const string& name) :  TaskContext(name, PreOperational), m_youbot_arm_dof(5)
{
	for(size_t i=0; i < m_youbot_arm_dof; ++i)
		m_trajectory_controller.push_back(new JointTrajectoryController);

	m_trajectory_action_srv = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction > (
      m_nh, "/arm_1/arm_controller/joint_trajectory_action",
      boost::bind(&ArmBridgeRosOrocos::armJointTrajectoryGoalCallback, this, _1),
      boost::bind(&ArmBridgeRosOrocos::armJointTrajectoryCancelCallback, this, _1), false);

	m_cartesian_with_impedance_ctrl_action_srv = new actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction > (
	      m_nh, "/arm_1/arm_controller/MoveToCartesianPoseDirect",
	      boost::bind(&ArmBridgeRosOrocos::armCartesianPoseWithImpedanceCtrlGoalCallback, this, _1), false);


	m_orocos_joint_positions.data.resize(m_youbot_arm_dof, 0.0);
	m_orocos_homog_matrix.data.resize(16, 0.0);
	m_orocos_arm_stiffness.data.resize(9, 0.0);

	orocos_joint_positions.setDataSample(m_orocos_joint_positions);
	orocos_homog_matrix.setDataSample(m_orocos_homog_matrix);
	orocos_arm_stiffness.setDataSample(m_orocos_arm_stiffness);

	this->addPort("brics_joint_positions", brics_joint_positions).doc("Input of joint positions in BRICS data types");
	this->addPort("orocos_joint_positions", orocos_joint_positions).doc("Output of joint positions in Orocos data type");
	this->addPort("orocos_homog_matrix", orocos_homog_matrix).doc("Output of a Cartesian Pose as homogeneous coordinates in Orocos data type");
	this->addPort("orocos_arm_stiffness", orocos_arm_stiffness).doc("Output of a arm stiffness to set");
}

ArmBridgeRosOrocos::~ArmBridgeRosOrocos()
{
	delete m_trajectory_action_srv;
	delete m_cartesian_with_impedance_ctrl_action_srv;
}

bool ArmBridgeRosOrocos::configureHook()
{
	return TaskContext::configureHook();
}

bool ArmBridgeRosOrocos::startHook()
{
	m_trajectory_action_srv->start();
	m_cartesian_with_impedance_ctrl_action_srv->start();

	m_arm_has_active_joint_trajectory_goal = false;


	if (!brics_joint_positions.connected())
	{
		log(Error) << "BRICS joint positions not connected." << endlog();
		return false;
	}

	if (!orocos_joint_positions.connected())
	{
		log(Error) << "Orocos joint positions not connected." << endlog();
		return false;
	}

	if (!orocos_homog_matrix.connected())
	{
		log(Error) << "Orocos homog_matrix not connected." << endlog();
		return false;
	}

	if (!orocos_arm_stiffness.connected())
	{
		log(Error) << "arm stiffness port not connected." << endlog();
		return false;
	}

	return TaskContext::startHook();
}

void ArmBridgeRosOrocos::updateHook()
{
	TaskContext::updateHook();

	ros::spinOnce();

	// check if trajectory controller is finished
	bool areTrajectoryControllersDone = true;

	for (size_t i = 0; i < m_youbot_arm_dof; ++i)
	{
		if (m_trajectory_controller[i].isTrajectoryControllerActive())
		{
			areTrajectoryControllersDone = false;
			break;
		}
	}

	if (areTrajectoryControllersDone && m_arm_has_active_joint_trajectory_goal)
	{
		m_arm_has_active_joint_trajectory_goal = false;
		control_msgs::FollowJointTrajectoryResult trajectoryResult;
		trajectoryResult.error_code = trajectoryResult.SUCCESSFUL;
		m_arm_active_joint_trajectory_goal.setSucceeded(trajectoryResult, "trajectory successful");
	}


	if (brics_joint_positions.read(m_brics_joint_positions) == NoData)
	{
		//log(Error) << "Cannot read the joystick" << endlog();
		return;
	}

	for (size_t i = 0; i < m_brics_joint_positions.positions.size(); i++)
		m_orocos_joint_positions.data[i] = m_brics_joint_positions.positions[i].value;

	orocos_joint_positions.write(m_orocos_joint_positions);
}

void ArmBridgeRosOrocos::stopHook()
{
	TaskContext::stopHook();
}

void ArmBridgeRosOrocos::cleanupHook()
{
	TaskContext::cleanupHook();
}


void ArmBridgeRosOrocos::armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbot_arm_goal)
{
	trajectory_msgs::JointTrajectory trajectory = youbot_arm_goal.getGoal()->trajectory;

	// validate that the correct number of joints is provided in the goal
	if (trajectory.joint_names.size() != m_youbot_arm_dof)
	{
		log(Error) << "Trajectory is malformed! Goal has " << trajectory.joint_names.size() << " joint names, but only " << m_youbot_arm_dof << " joints are supported" << endlog();
		youbot_arm_goal.setRejected();
		return;
	}

	std::vector<JointTrajectory> jointTrajectories(m_youbot_arm_dof);
  
	// convert from the ROS trajectory representation to the controller's representation
	std::vector<std::vector< double > > positions(m_youbot_arm_dof);
	std::vector<std::vector< double > > velocities(m_youbot_arm_dof);
	std::vector<std::vector< double > > accelerations(m_youbot_arm_dof);
	TrajectorySegment segment;
	for (size_t i = 0; i < trajectory.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
		// validate the trajectory point
		if ((point.positions.size() != m_youbot_arm_dof
						|| point.velocities.size() != m_youbot_arm_dof
						|| point.accelerations.size() != m_youbot_arm_dof))
		{
			log(Error) << "A trajectory point is malformed! " << m_youbot_arm_dof << " positions, velocities and accelerations must be provided" << endlog();
			youbot_arm_goal.setRejected();
			return;
		}
    
		for (size_t j = 0; j < m_youbot_arm_dof; j++)
		{
			segment.positions = point.positions[j];
			segment.velocities = point.velocities[j];
			segment.accelerations = point.accelerations[j];
			segment.time_from_start = boost::posix_time::microsec(point.time_from_start.toNSec()/1000);
			jointTrajectories[j].segments.push_back(segment);
		}
	}

	for (size_t j = 0; j < m_youbot_arm_dof; j++)
	{
      jointTrajectories[j].start_time = boost::posix_time::microsec_clock::local_time(); //TODO is this correct to set the trajectory start time to now
	}

  
  

	// cancel the old goal
	/*
	if (m_arm_has_active_joint_trajectory_goal)
	{
		m_arm_active_joint_trajectory_goal.setCanceled();
		m_arm_has_active_joint_trajectory_goal = false;
		for (int i = 0; i < m_youbot_arm_dof; ++i)
		{
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).cancelTrajectory();
		}
	}
	 */

	// replace the old goal with the new one
	youbot_arm_goal.setAccepted();
	m_arm_active_joint_trajectory_goal = youbot_arm_goal;
	m_arm_has_active_joint_trajectory_goal = true;


	// send the trajectory to the controller
	for (size_t i = 0; i < m_youbot_arm_dof; ++i)
	{
		try
		{
			// youBot joints start with 1 not with 0 -> i + 1
			m_trajectory_controller[i].setTrajectory(jointTrajectories[i]);
			log(Info) << "set trajectories " << i << endlog();
		} catch (std::exception& e)
		{
			std::string errorMessage = e.what();
			log(Warning) << "Cannot set trajectory for joint " << i + 1 << "\n " << errorMessage.c_str() << endlog();
		}
	}
	log(Info) << "set all trajectories" << endlog();
}

void ArmBridgeRosOrocos::armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbot_arm_goal)
{
	// stop the controller
	for (size_t i = 0; i < m_youbot_arm_dof; ++i)
	{
		try
		{
			m_trajectory_controller[i].cancelCurrentTrajectory();
		} catch (std::exception& e)
		{
			std::string errorMessage = e.what();
			log(Warning) << "Cannot stop joint " << i + 1 << "\n" << errorMessage.c_str();
		}
	}

	if (m_arm_active_joint_trajectory_goal == youbot_arm_goal)
	{
		// Marks the current goal as canceled.
		youbot_arm_goal.setCanceled();
		m_arm_has_active_joint_trajectory_goal = false;
	}
}

void ArmBridgeRosOrocos::armCartesianPoseWithImpedanceCtrlGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle youbot_arm_goal)
{
	btVector3 trans_vec;
	btQuaternion bt_quat;

	geometry_msgs::PoseStamped goal_pose = youbot_arm_goal.getGoal()->goal;

	// create rotation matrix from quaternion
	tf::quaternionMsgToTF(goal_pose.pose.orientation, bt_quat);
	btMatrix3x3 rot_mat = btMatrix3x3(bt_quat);

	m_orocos_homog_matrix.data[0] = rot_mat[0][0];
	m_orocos_homog_matrix.data[1] = rot_mat[0][1];
	m_orocos_homog_matrix.data[2] = rot_mat[0][2];
	m_orocos_homog_matrix.data[3] = goal_pose.pose.position.x;
	m_orocos_homog_matrix.data[4] = rot_mat[1][0];
	m_orocos_homog_matrix.data[5] = rot_mat[1][1];
	m_orocos_homog_matrix.data[6] = rot_mat[1][2];
	m_orocos_homog_matrix.data[7] = goal_pose.pose.position.y;
	m_orocos_homog_matrix.data[8] = rot_mat[2][0];
	m_orocos_homog_matrix.data[9] = rot_mat[2][1];
	m_orocos_homog_matrix.data[10] = rot_mat[2][2];
	m_orocos_homog_matrix.data[11] = goal_pose.pose.position.z;
	m_orocos_homog_matrix.data[12] = 0.0;
	m_orocos_homog_matrix.data[13] = 0.0;
	m_orocos_homog_matrix.data[14] = 0.0;
	m_orocos_homog_matrix.data[15] = 1.0;

	m_orocos_arm_stiffness.data[0] = 100;
	m_orocos_arm_stiffness.data[1] = 100;
	m_orocos_arm_stiffness.data[2] = 100;
	m_orocos_arm_stiffness.data[3] = 0;
	m_orocos_arm_stiffness.data[4] = 0;
	m_orocos_arm_stiffness.data[5] = 0;
	m_orocos_arm_stiffness.data[6] = 0;
	m_orocos_arm_stiffness.data[7] = 0;
	m_orocos_arm_stiffness.data[8] = 0;

	std::cout << "write homog matrix to output port" << std::endl;

	orocos_arm_stiffness.write(m_orocos_arm_stiffness);
	orocos_homog_matrix.write(m_orocos_homog_matrix);
}


ORO_CREATE_COMPONENT( ArmBridgeRosOrocos )
