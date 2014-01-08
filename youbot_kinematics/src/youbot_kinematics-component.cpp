#include "youbot_kinematics-component.hpp"
#include <rtt/Component.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <iostream>

using namespace KDL;
using namespace RTT; 

Youbot_kinematics::Youbot_kinematics(std::string const& name) : TaskContext(name), m_jnt_array(8){
  this->addEventPort("JointState",port_joint_state);
  this->addEventPort("BaseOdom",port_odom);
  this->addPort("EEPose",port_ee_pose_ros);
  this->addPort("EEPoseRTT",port_ee_pose_rtt);
  this->addPort("EETwistMsr",port_ee_twist_msr);
  this->addEventPort("EETwist",port_ee_twist_ros);
  this->addEventPort("EETwistRTT",port_ee_twist_rtt);
  this->addPort("JointVelocities",port_joint_velocities);
  this->addPort("BaseTwist",port_base_twist);
  this->addEventPort("JointSpaceWeights",port_w_js);
  this->addEventPort("TaskSpaceWeights",port_w_ts);

  this->addProperty("robot_description",prop_urdf_model);

  m_joint_state.name.assign(5,"0        10       20");
  m_joint_state.position.resize(5);
  m_joint_state.velocity.resize(5);
  m_joint_state.effort.resize(5);

  m_joint_velocities.names.assign(5, "arm_joint");
  m_joint_velocities.velocities.assign(5,0.0);
  m_w_js.assign(8,1.0);
  m_w_ts.assign(6,1);

  port_joint_velocities.write(m_joint_velocities);
  port_base_twist.write(m_ee_twist);

  //Let's assume the base has two translational joints and a
  //rotational joint: Base joints
  m_chain.addSegment(Segment(Joint(Joint::TransX)));
  m_chain.addSegment(Segment(Joint(Joint::TransY)));
  //The base of the arm is located 16 cm in front of the center
  //of the platform
  m_chain.addSegment(Segment(Joint(Joint::RotZ),
  	Frame(Vector(0.143,0.0, 0.096 + 0.046)))); 

  std::cout << "|YoubotKinematics| constructed!" <<std::endl;	  
}

bool Youbot_kinematics::configureHook(){
  KDL::Tree my_tree;
	if (!kdl_parser::treeFromString(prop_urdf_model, my_tree)){
		log(Error)<<"Failed to construct kdl tree"<<endlog();
		return false;
	}
        // Add arm chain from urdf to existing chain
	KDL::Chain arm_chain;
	if(!my_tree.getChain("arm_link_0","gripper_palm_link",arm_chain)){
		log(Error)<< "Failed to construct subchain from urdf model"<<endlog();
		return false;
	}
	if(arm_chain.getNrOfJoints()!= 5){
		log(Error)<<"Nr of joints in urdf model is not correct: "<<arm_chain.getNrOfJoints()<<endlog();
		return false;
	}
	m_chain.addChain(arm_chain);


	jnt_to_pose_solver_.reset(new ChainFkSolverVel_recursive(m_chain));
	pose_to_jnt_solver_.reset(new ChainIkSolverVel_wdls(m_chain,1.0));

	pose_to_jnt_solver_->setLambda(0.5);
	SetToZero(m_jnt_array);
	m_Mq.setIdentity();
	m_My.setIdentity();
	SetToZero(m_twist);
	pose_to_jnt_solver_->setWeightJS(m_Mq);
	pose_to_jnt_solver_->setWeightTS(m_My);
  	
	m_joint_velocities.names[0] = "arm_joint_1";
	m_joint_velocities.names[1] = "arm_joint_2";
	m_joint_velocities.names[2] = "arm_joint_3";
	m_joint_velocities.names[3] = "arm_joint_4";
	m_joint_velocities.names[4] = "arm_joint_5";

  	std::cout << "|YoubotKinematics| configured !" <<std::endl;
  	return true;
}

bool Youbot_kinematics::startHook(){
	port_joint_state.read(m_joint_state);
        //Leave out the base for now, start from 3
	for(unsigned int i=3;i<m_jnt_array.q.rows();i++){
		m_jnt_array.q(i)=m_joint_state.position[i-3];
		m_jnt_array.qdot(i)=m_joint_state.velocity[i-3];
	}

	jnt_to_pose_solver_->JntToCart(m_jnt_array,m_frame_vel);
	tf::PoseKDLToMsg(m_frame_vel.GetFrame(),m_ee_pose);
	tf::TwistKDLToMsg(m_frame_vel.GetTwist(),m_ee_twist_msr);
	port_ee_pose_ros.write(m_ee_pose);
	port_ee_pose_rtt.write(m_frame_vel.GetFrame());
	port_ee_twist_msr.write(m_ee_twist_msr);

	std::cout << "|YoubotKinematics| started !" <<std::endl;
	return true;
}

void Youbot_kinematics::updateHook(){
	if(port_w_ts.read(m_w_ts)==NewData){
		if((int)m_w_ts.size()==m_My.rows()){
			for(unsigned int i=0;i<m_My.rows();i++)
				m_My(i,i)=m_w_ts[i];
			pose_to_jnt_solver_->setWeightTS(m_My);
		}
		else
			log(Error)<<"Data on "<<port_w_ts.getName()<<" has the wrong size."<<endlog();
	}
	if(port_w_js.read(m_w_js)==NewData){
		if((int)m_w_js.size()==m_Mq.rows()){
			for(unsigned int i=0;i<m_Mq.rows();i++)
				m_Mq(i,i)=m_w_js[i];
			pose_to_jnt_solver_->setWeightJS(m_Mq);
		}
		else
			log(Error)<<"Data on "<<port_w_js.getName()<<" has the wrong size."<<endlog();
	}
	bool update_pose=false;
	if(port_odom.read(m_odom)==NewData){
		m_jnt_array.q(0)=m_odom.pose.pose.position.x;
		m_jnt_array.q(1)=m_odom.pose.pose.position.y;
		KDL::Rotation rot = KDL::Rotation::Quaternion(m_odom.pose.pose.orientation.x,
			m_odom.pose.pose.orientation.y,
			m_odom.pose.pose.orientation.z,
			m_odom.pose.pose.orientation.w);
		double roll,pitch,yaw;
		rot.GetRPY(roll,pitch,yaw);
		m_jnt_array.q(2)=yaw;
		m_jnt_array.qdot(0)=m_odom.twist.twist.linear.x;
		m_jnt_array.qdot(1)=m_odom.twist.twist.linear.y;
		m_jnt_array.qdot(3)=m_odom.twist.twist.angular.z;
		update_pose=true;
	}

	if(port_joint_state.read(m_joint_state)==NewData){
            //Leave out the base for now, start from 3
		for(unsigned int i=3;i<m_jnt_array.q.rows();i++){
			m_jnt_array.q(i)=m_joint_state.position[i-3];
			m_jnt_array.qdot(i)=m_joint_state.velocity[i-3];
		}
		update_pose=true;
	}

	if(update_pose){
		jnt_to_pose_solver_->JntToCart(m_jnt_array,m_frame_vel);
		tf::PoseKDLToMsg(m_frame_vel.GetFrame(),m_ee_pose);
		tf::TwistKDLToMsg(m_frame_vel.GetTwist(),m_ee_twist_msr);
		port_ee_pose_ros.write(m_ee_pose);
		port_ee_pose_rtt.write(m_frame_vel.GetFrame());
		port_ee_twist_msr.write(m_ee_twist_msr);
	}

	//if(port_ee_twist_ros.read(m_ee_twist)==NewData)
	//	tf::TwistMsgToKDL(m_ee_twist,m_twist);
	port_ee_twist_rtt.read(m_twist);
	std::stringstream jointName;	
	int ret = pose_to_jnt_solver_->CartToJnt(m_jnt_array.q,m_twist,m_jnt_array.qdot);
	if (ret>=0){
		m_base_twist.linear.x=m_jnt_array.qdot(0);
		m_base_twist.linear.y=m_jnt_array.qdot(1);
		m_base_twist.angular.z=m_jnt_array.qdot(2);
		for(unsigned int i=3;i<m_jnt_array.qdot.rows();i++)
		{
			//jointName.str("");
			//jointName << "arm_joint_" << (i-2);
			//m_joint_velocities.names[i-3]= jointName.str();
			if(m_jnt_array.qdot(i) > 0.3)
				m_jnt_array.qdot(i) = 0.3;

			if(m_jnt_array.qdot(i) < -0.3)
				m_jnt_array.qdot(i) = -0.3;

			m_joint_velocities.velocities[i-3]= m_jnt_array.qdot(i);
		}
	}
	else{
		m_base_twist.linear.x=0;
		m_base_twist.linear.y=0;
		m_base_twist.angular.z=0;
		for(unsigned int i=3;i<m_jnt_array.qdot.rows();i++)
		{
			//jointName.str("");
			//jointName << "arm_joint_" << (i-2);
			//m_joint_velocities.names[i-3]= jointName.str();
			m_joint_velocities.velocities[i-3]= 0.0;
		}
		log(Error)<<"Could not calculate IVK: " << ret <<endlog();
	}
	port_joint_velocities.write(m_joint_velocities);
	port_base_twist.write(m_base_twist);
}

void Youbot_kinematics::stopHook() {
  std::cout << "Youbot_kinematics executes stopping !" <<std::endl;
}

void Youbot_kinematics::cleanupHook() {
  std::cout << "Youbot_kinematics cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Youbot_kinematics)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Youbot_kinematics)
