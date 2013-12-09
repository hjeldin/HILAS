#ifndef OROCOS_YOUBOT_KINEMATICS_COMPONENT_HPP
#define OROCOS_YOUBOT_KINEMATICS_COMPONENT_HPP

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

#include <boost/lexical_cast.hpp>

class Youbot_kinematics : public RTT::TaskContext{
  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Youbot_kinematics(std::string const& name);
private:
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    /** Input Port **/
    RTT::InputPort<sensor_msgs::JointState> port_joint_state;
    RTT::InputPort<nav_msgs::Odometry> port_odom;
    RTT::InputPort<geometry_msgs::Twist> port_ee_twist_ros;
    RTT::InputPort<KDL::Twist> port_ee_twist_rtt;
    RTT::InputPort<std::vector<double> > port_w_js, port_w_ts;

    /** Output Port **/
    RTT::OutputPort<motion_control_msgs::JointVelocities> port_joint_velocities;
    RTT::OutputPort<geometry_msgs::Twist> port_base_twist; 
    RTT::OutputPort<geometry_msgs::Pose> port_ee_pose_ros;
    RTT::OutputPort<KDL::Frame> port_ee_pose_rtt;
    RTT::OutputPort<geometry_msgs::Twist> port_ee_twist_msr;

    /** URDF Robot Model **/
    std::string prop_urdf_model;

    /** General Variables **/
    sensor_msgs::JointState m_joint_state;
    nav_msgs::Odometry m_odom;
    geometry_msgs::Pose m_ee_pose;
    geometry_msgs::Twist m_ee_twist, m_base_twist, m_ee_twist_msr;
    motion_control_msgs::JointVelocities m_joint_velocities;
    std::vector<double> m_w_js,m_w_ts;    

    KDL::JntArrayVel m_jnt_array;

    KDL::FrameVel m_frame_vel;
    KDL::Twist m_twist;
    KDL::Chain m_chain;
    Eigen::Matrix<double,8,8> m_Mq;
    Eigen::Matrix<double,6,6> m_My;

    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_pose_solver_;
    boost::shared_ptr<KDL::ChainIkSolverVel_wdls> pose_to_jnt_solver_;
    //boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

};
#endif