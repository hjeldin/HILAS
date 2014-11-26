#pragma once

#include <rtt/RTT.hpp>
#include <iostream>

#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <rtt/Component.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <math.h>
#include <fstream>

#include <boost/lexical_cast.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include "../../definitions/hilas.hpp"

namespace Hilas
{

using namespace KDL;
using namespace RTT;

class IRobotKinematics: public RTT::TaskContext
{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	IRobotKinematics(std::string const& name, int kine_joint_count, int robot_joint_count, int SIZE_JOINT_NAME_ARRAY, const std::string* JOINT_NAME_ARRAY);
	~IRobotKinematics();

protected:

	bool configureHook();
	bool startHook();
	virtual void updateHook();
	void stopHook();
	void cleanupHook();

	virtual void createKinematicChain() = 0;
	virtual void assignJointToChain();
	virtual void differentialKinematic() = 0;
	virtual void forwardKinematic() = 0;

	/** Input Port **/
	RTT::InputPort<sensor_msgs::JointState> port_joint_state_in;
	RTT::InputPort<nav_msgs::Odometry> port_odom_in;
	RTT::InputPort<KDL::Twist> port_ee_twist_in;
	RTT::InputPort<std::vector<double> > port_w_js_in, port_w_ts_in;

	/** Output Port **/
	RTT::OutputPort<motion_control_msgs::JointVelocities> port_joint_velocities_out;
	RTT::OutputPort<geometry_msgs::Pose> port_ee_pose_out;

	/** URDF Robot Model **/
	std::string prop_urdf_model;
  	KDL::Tree my_tree;
	KDL::Chain robot_chain;
	KDL::JntArrayVel robot_joint_array;
	int chain_joint_count;
	int robot_joint_count;

	/** General Variables **/
	sensor_msgs::JointState robot_joint_state;
	nav_msgs::Odometry m_odom;
	geometry_msgs::Pose m_ee_pose;
	geometry_msgs::Twist m_ee_twist;
	motion_control_msgs::JointVelocities m_joint_velocities;

	std::vector<double> m_w_js,m_w_ts;
	KDL::FrameVel m_frame_vel;
	KDL::Twist m_twist;

	/* Joint Space Weighting matrix */
	Eigen::MatrixXd m_Mq_identity;
	Eigen::MatrixXd m_My_identity;

	/* Joint Space Weighting matrix */
	Eigen::MatrixXd m_Mq_jlc;
	Eigen::MatrixXd m_My_jlc;

	/* Norm Joints */
	std::vector<double> joints_norm;

	boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_pose_solver;
	boost::shared_ptr<KDL::ChainIkSolverVel_wdls> pose_to_jnt_solver;

	/* Joints limits */
	std::vector<double> joints_min_limits;
	std::vector<double> joints_max_limits;
};

}