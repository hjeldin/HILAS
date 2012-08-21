#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <brics_actuator/typekit/Types.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include "JointTrajectoryController.hpp"
#include "JointTrajectory.hpp"

#include <boost/ptr_container/ptr_vector.hpp>


using namespace RTT;
using namespace std;
class arm_bridge_ros_orocos: public TaskContext
{
  public:
    arm_bridge_ros_orocos(const string& name);
    virtual ~arm_bridge_ros_orocos();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    /**
     * @brief Callback that is executed when an action goal to perform a joint trajectory with the arm comes in.
     * @param youbotArmGoal Actionlib goal that contains the trajectory.
     * @param armIndex Index that identifies the arm
     */
     void armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal);

    /**
     * @brief Callback that is executed when an action goal of a joint trajectory is canceled.
     * @param youbotArmGoal Actionlib goal that contains the trajectory.
     * @param armIndex Index that identifies the arm
     */
     void armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal);

  private:
    void trajectoryActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    OutputPort<std_msgs::Float64MultiArray> orocos_joint_positions;

    std_msgs::Float64MultiArray m_orocos_joint_positions;

    ros::NodeHandle m_nh;
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> *m_trajectory_action_srv;
    control_msgs::FollowJointTrajectoryResult m_action_result;

    boost::ptr_vector<JointTrajectoryController> trajectoryController;
    bool armHasActiveJointTrajectoryGoal;
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle armActiveJointTrajectoryGoal;

    const size_t youBotArmDoF;
};
