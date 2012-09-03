#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>
#include <ocl/Component.hpp>

#include <string>
#include <vector>

#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <brics_actuator/typekit/Types.h>
#include <actionlib/server/simple_action_server.h>
#include <raw_arm_navigation/MoveToCartesianPoseAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_datatypes.h>

#include <boost/ptr_container/ptr_vector.hpp>

#include "JointTrajectoryController.hpp"
#include "JointTrajectory.hpp"


using namespace RTT;
using namespace std;

class ArmBridgeRosOrocos: public TaskContext
{
  public:
    ArmBridgeRosOrocos(const string& name);
    virtual ~ArmBridgeRosOrocos();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    /**
     * @brief Callback that is executed when an action goal to perform a joint trajectory with the arm comes in.
     * @param youbot_arm_goal Actionlib goal that contains the trajectory.
     */
    void armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbot_arm_goal);

    /**
     * @brief Callback that is executed when an action goal of a joint trajectory is canceled.
     * @param youbot_arm_goal Actionlib goal that contains the trajectory.
     */
    void armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbot_arm_goal);

    /**
	  * @brief Callback that is executed when an action goal to go to a Cartesian pose with the arm comes in.
	  * @param youbot_arm_goal Actionlib goal that contains the 6DOF pose.
	  */
    void armCartesianPoseWithImpedanceCtrlGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle youbot_arm_goal);

  private:

    InputPort<brics_actuator::JointPositions> brics_joint_positions;
    OutputPort<std_msgs::Float64MultiArray> orocos_joint_positions;
    OutputPort<std_msgs::Float64MultiArray> orocos_homog_matrix;
    OutputPort<std_msgs::Float64MultiArray> orocos_arm_stiffness;
    OutputPort<std_msgs::Float64MultiArray> orocos_HtipCC;

    brics_actuator::JointPositions m_brics_joint_positions;
    std_msgs::Float64MultiArray m_orocos_joint_positions;
    std_msgs::Float64MultiArray m_orocos_homog_matrix;
    std_msgs::Float64MultiArray m_orocos_arm_stiffness;
    std_msgs::Float64MultiArray m_orocos_HtipCC;

	ros::NodeHandle m_nh;

	/* Action Server */
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> *m_trajectory_action_srv;
    control_msgs::FollowJointTrajectoryResult m_action_result;

    actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction> *m_cartesian_with_impedance_ctrl_action_srv;


    boost::ptr_vector<JointTrajectoryController> m_trajectory_controller;
    bool m_arm_has_active_joint_trajectory_goal;
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle m_arm_active_joint_trajectory_goal;

    const size_t m_youbot_arm_dof;
};

