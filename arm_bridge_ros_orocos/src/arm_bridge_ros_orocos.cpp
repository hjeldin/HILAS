#include "arm_bridge_ros_orocos.hpp"

#include <ocl/Component.hpp>

#include <vector>

using namespace RTT;
using namespace std;


arm_bridge_ros_orocos::arm_bridge_ros_orocos(const string& name) :
    TaskContext(name, PreOperational), youBotArmDoF(5)
{
  trajectoryController.push_back(new JointTrajectoryController);
  trajectoryController.push_back(new JointTrajectoryController);
  trajectoryController.push_back(new JointTrajectoryController);
  trajectoryController.push_back(new JointTrajectoryController);
  trajectoryController.push_back(new JointTrajectoryController);

  m_trajectory_action_srv = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction > (
      m_nh, "/arm_1/arm_controller/joint_trajectory_action",
      boost::bind(&arm_bridge_ros_orocos::armJointTrajectoryGoalCallback, this, _1),
      boost::bind(&arm_bridge_ros_orocos::armJointTrajectoryCancelCallback, this, _1), false);


  m_orocos_joint_positions.data.resize(5, 0.0);

  orocos_joint_positions.setDataSample(m_orocos_joint_positions);

  this->addPort("brics_joint_positions", brics_joint_positions).doc(
      "Input of joint positions in BRICS data types");
  this->addPort("orocos_joint_positions", orocos_joint_positions).doc("Output of joint positions in Orocos data type");
}

arm_bridge_ros_orocos::~arm_bridge_ros_orocos()
{
	delete m_trajectory_action_srv;
}

bool arm_bridge_ros_orocos::configureHook()
{
  return TaskContext::configureHook();
}

bool arm_bridge_ros_orocos::startHook()
{
  m_trajectory_action_srv->start();
  armHasActiveJointTrajectoryGoal = false;


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

  return TaskContext::startHook();
}

void arm_bridge_ros_orocos::updateHook()
{
  TaskContext::updateHook();

  ros::spinOnce();

  
	// check if trajectory controller is finished
	bool areTrajectoryControllersDone = true;
	for (size_t i = 0; i < youBotArmDoF; ++i) {
		if (trajectoryController[i].isTrajectoryControllerActive()) {
			areTrajectoryControllersDone = false;
			break;
		}
	}
	if (areTrajectoryControllersDone && armHasActiveJointTrajectoryGoal) {
		armHasActiveJointTrajectoryGoal = false;
		control_msgs::FollowJointTrajectoryResult trajectoryResult;
		trajectoryResult.error_code = trajectoryResult.SUCCESSFUL;
		armActiveJointTrajectoryGoal.setSucceeded(trajectoryResult, "trajectory successful");
	}



  if (brics_joint_positions.read(m_brics_joint_positions) == NoData)
  {
//        	log(Error) << "Cannot read the joystick" << endlog();
    return;
  }

  for (size_t i = 0; i < m_brics_joint_positions.positions.size(); i++) {
    m_orocos_joint_positions.data[i] = m_brics_joint_positions.positions[i].value;
  }

  orocos_joint_positions.write(m_orocos_joint_positions);
}

void arm_bridge_ros_orocos::stopHook()
{
  TaskContext::stopHook();
}

void arm_bridge_ros_orocos::cleanupHook()
{
  TaskContext::cleanupHook();
}


void arm_bridge_ros_orocos::armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal) {
	trajectory_msgs::JointTrajectory trajectory = youbotArmGoal.getGoal()->trajectory;

	// validate that the correct number of joints is provided in the goal
	if (trajectory.joint_names.size() != youBotArmDoF) {
		log(Error) << "Trajectory is malformed! Goal has " << trajectory.joint_names.size() << " joint names, but only " << youBotArmDoF << " joints are supported" << endlog();
		youbotArmGoal.setRejected();
		return;
	}

	std::vector<JointTrajectory> jointTrajectories(youBotArmDoF);
  
	// convert from the ROS trajectory representation to the controller's representation
	std::vector<std::vector< double > > positions(youBotArmDoF);
	std::vector<std::vector< double > > velocities(youBotArmDoF);
	std::vector<std::vector< double > > accelerations(youBotArmDoF);
	TrajectorySegment segment;
	for (size_t i = 0; i < trajectory.points.size(); i++) {
		trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
		// validate the trajectory point
		if ((point.positions.size() != youBotArmDoF
						|| point.velocities.size() != youBotArmDoF
						|| point.accelerations.size() != youBotArmDoF)) {
			log(Error) << "A trajectory point is malformed! " << youBotArmDoF << " positions, velocities and accelerations must be provided" << endlog();
			youbotArmGoal.setRejected();
			return;
		}
    
		for (size_t j = 0; j < youBotArmDoF; j++) {
      segment.positions = point.positions[j];
      segment.velocities = point.velocities[j];
      segment.accelerations = point.accelerations[j];
      segment.time_from_start = boost::posix_time::microsec(point.time_from_start.toNSec()/1000);
      jointTrajectories[j].segments.push_back(segment);
		}
	}
  for (size_t j = 0; j < youBotArmDoF; j++) {
      jointTrajectories[j].start_time = boost::posix_time::microsec_clock::local_time(); //TODO is this correct to set the trajectory start time to now
  }

  
  

	// cancel the old goal
  /*
	if (armHasActiveJointTrajectoryGoal) {
		armActiveJointTrajectoryGoal.setCanceled();
		armHasActiveJointTrajectoryGoal = false;
		for (int i = 0; i < youBotArmDoF; ++i) {
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).cancelTrajectory();
		}
	}
  */

	// replace the old goal with the new one
	youbotArmGoal.setAccepted();
	armActiveJointTrajectoryGoal = youbotArmGoal;
	armHasActiveJointTrajectoryGoal = true;


	// send the trajectory to the controller
	for (size_t i = 0; i < youBotArmDoF; ++i) {
		try {
			// youBot joints start with 1 not with 0 -> i + 1
			trajectoryController[i].setTrajectory(jointTrajectories[i]);
			log(Info) << "set trajectories " << i << endlog();
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			log(Warning) << "Cannot set trajectory for joint " << i + 1 << "\n " << errorMessage.c_str() << endlog();
		}
	}
	log(Info) << "set all trajectories" << endlog();
}

void arm_bridge_ros_orocos::armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal) {
	// stop the controller
	for (size_t i = 0; i < youBotArmDoF; ++i) {
		try {
			trajectoryController[i].cancelCurrentTrajectory();
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			log(Warning) << "Cannot stop joint " << i + 1 << "\n" << errorMessage.c_str();
		}
	}

	if (armActiveJointTrajectoryGoal == youbotArmGoal) {
		// Marks the current goal as canceled.
		youbotArmGoal.setCanceled();
		armHasActiveJointTrajectoryGoal = false;
	}
}


ORO_CREATE_COMPONENT( arm_bridge_ros_orocos )
