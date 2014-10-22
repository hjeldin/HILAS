#include "youbot_kinematics-component.hpp"
#include <rtt/Component.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <iostream>
#include <math.h>
#include <fstream>

using namespace KDL;
using namespace RTT; 

YouBot_kinematics::YouBot_kinematics(std::string const& name) : TaskContext(name), m_jnt_array(8){
  this->addEventPort("JointState_in",port_joint_state_in);
  this->addEventPort("BaseOdom_in",port_odom_in);
  this->addPort("EEPose_out",port_ee_pose_ros_out);
  this->addPort("EEPoseRTT_out",port_ee_pose_rtt_out);
  this->addPort("EETwistMsr_out",port_ee_twist_msr_out);
  this->addEventPort("EETwist_in",port_ee_twist_ros_in);
  this->addEventPort("EETwistRTT_in",port_ee_twist_rtt_in);
  this->addPort("JointVelocities_out",port_joint_velocities_out);
  this->addPort("BaseTwist_out",port_base_twist_out);
  this->addEventPort("JointSpaceWeights_in",port_w_js_in);
  this->addEventPort("TaskSpaceWeights_in",port_w_ts_in);

  //this->addProperty("robot_description",prop_urdf_model);

  m_joint_state.name.assign(5,"0        10       20");
  m_joint_state.position.resize(5);
  m_joint_state.velocity.resize(5);
  m_joint_state.effort.resize(5);

  joints_min_limits.assign(5,0);
  joints_max_limits.assign(5,0);

  m_joint_velocities.names.assign(5, "arm_joint");
  m_joint_velocities.velocities.assign(5,0.0);

  m_w_js.assign(8,1.0);
  m_w_ts.assign(6,1);

  joints_norm.assign(5,0);

  //Let's assume the base has two translational joints and a
  //rotational joint: Base joints
  m_chain.addSegment(Segment(Joint(Joint::TransX)));
  m_chain.addSegment(Segment(Joint(Joint::TransY)));
  //The base of the arm is located 16 cm in front of the center
  //of the platform
  m_chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.143,0.0, 0.096 + 0.046)))); 

  std::cout << "|YoubotKinematics| constructed!" <<std::endl;	  
}

bool YouBot_kinematics::configureHook()
{
	std::ifstream file(URDF_PATH_TO_FILE);
	std::cout << "Youbot urdf: " << URDF_PATH_TO_FILE << std::endl;

	file.seekg(0, std::ios::end);
	prop_urdf_model.reserve(file.tellg());
	file.seekg(0, std::ios::beg);
	
	prop_urdf_model.assign((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());

  	KDL::Tree my_tree;
	TiXmlDocument xml_doc;
	TiXmlElement* xml_root;

   	xml_doc.Parse(prop_urdf_model.c_str());
   	xml_root = xml_doc.FirstChildElement("robot");
   	
   	if(!xml_root)
   	{
    	log(Error) << "Failed to get robot from xml document" << endlog();
      	return false;
   	}
   	
   	if (!kdl_parser::treeFromXml(&xml_doc, my_tree))
   	{
    	log(Error) << "Failed to construct kdl tree" << endlog();
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
	
	m_Mq_identity.setIdentity();
	m_My_identity.setIdentity();
	m_Mq_jlc.setIdentity();

	SetToZero(m_twist);
	pose_to_jnt_solver_->setWeightJS(m_Mq_identity);
	pose_to_jnt_solver_->setWeightTS(m_My_identity);
  	
	m_joint_velocities.names[0] = "arm_joint_1";
	m_joint_velocities.names[1] = "arm_joint_2";
	m_joint_velocities.names[2] = "arm_joint_3";
	m_joint_velocities.names[3] = "arm_joint_4";
	m_joint_velocities.names[4] = "arm_joint_5";

	joints_min_limits[0] = 0.01;
	joints_min_limits[1] = 0.01;
	joints_min_limits[2] = -5.0215;
	joints_min_limits[3] = 0.022;
	joints_min_limits[4] = 0.11073;

	joints_max_limits[0] = 5.8343;
	joints_max_limits[1] = 2.61538;
	joints_max_limits[2] = -0.0157;
	joints_max_limits[3] = 3.42577;
	joints_max_limits[4] = 5.63595;	

  	std::cout << "|YoubotKinematics| configured !" <<std::endl;
  	return true;
}

bool YouBot_kinematics::startHook()
{
	port_joint_state_in.read(m_joint_state);
    
    //Leave out the base for now, start from 3
	for(unsigned int i=3;i<m_jnt_array.q.rows();i++)
	{
		m_jnt_array.q(i)=m_joint_state.position[i-3];
		m_jnt_array.qdot(i)=m_joint_state.velocity[i-3];
	}

	jnt_to_pose_solver_->JntToCart(m_jnt_array,m_frame_vel);

	tf::PoseKDLToMsg(m_frame_vel.GetFrame(),m_ee_pose);
	tf::TwistKDLToMsg(m_frame_vel.GetTwist(),m_ee_twist_msr);

  	port_joint_velocities_out.write(m_joint_velocities);
  	port_base_twist_out.write(m_ee_twist);

	port_ee_pose_ros_out.write(m_ee_pose);
	port_ee_pose_rtt_out.write(m_frame_vel.GetFrame());
	port_ee_twist_msr_out.write(m_ee_twist_msr);

	std::cout << "|YoubotKinematics| started !" <<std::endl;
	return true;
}

void YouBot_kinematics::updateHook()
{
	if(port_w_ts_in.read(m_w_ts)==NewData)
	{
		if((int)m_w_ts.size()==m_My_identity.rows())
		{
			for(unsigned int i=0;i<m_My_identity.rows();i++)
			{
				m_My_identity(i,i)=m_w_ts[i];				
			}

			pose_to_jnt_solver_->setWeightTS(m_My_identity);
		}
		else
		{
			log(Error)<<"Data on "<<port_w_ts_in.getName()<<" has the wrong size."<<endlog();
		}
	}

	if(port_w_js_in.read(m_w_js)==NewData)
	{
		if((int)m_w_js.size()==m_Mq_identity.rows())
		{
			for(unsigned int i=0;i<m_Mq_identity.rows();i++)
			{
				m_Mq_identity(i,i)=m_w_js[i];
			}

			pose_to_jnt_solver_->setWeightJS(m_Mq_identity);
		}
		else
		{
			log(Error)<<"Data on "<<port_w_js_in.getName()<<" has the wrong size."<<endlog();			
		}
	}

	bool update_pose=false;
	if(port_odom_in.read(m_odom) == NewData)
	{
		m_jnt_array.q(0)=m_odom.pose.pose.position.x;
		m_jnt_array.q(1)=m_odom.pose.pose.position.y;
		
		KDL::Rotation rot = KDL::Rotation::Quaternion(
			m_odom.pose.pose.orientation.x,
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

	if(port_joint_state_in.read(m_joint_state) == NewData)
	{
        //Leave out the base for now, start from 3
		for(unsigned int i=3;i<m_jnt_array.q.rows();i++)
		{
			m_jnt_array.q(i)=m_joint_state.position[i-3];
			m_jnt_array.qdot(i)=m_joint_state.velocity[i-3];

			//Set joint space weighting matrix --> Joint space limits control
			joints_norm[i-3] = (((m_jnt_array.q(i) - joints_min_limits[i-3]) / (joints_max_limits[i-3] - joints_min_limits[i-3])) * 2) -1;
			//m_Mq_jlc(i,i) = 1 - pow(m_Mq_jlc(i,i),4);
			//std::cout << "joint_state[" << i-2 << "].position =" << m_jnt_array.q(i) << std::endl;
			//std::cout << "joints_norm(" << i << "," << i << ")=" << joints_norm[i-3] << std::endl;
		}
		update_pose = true;
	}

	if(update_pose)
	{
		jnt_to_pose_solver_->JntToCart(m_jnt_array,m_frame_vel);

		tf::PoseKDLToMsg(m_frame_vel.GetFrame(),m_ee_pose);
		tf::TwistKDLToMsg(m_frame_vel.GetTwist(),m_ee_twist_msr);
		port_ee_pose_ros_out.write(m_ee_pose);
		port_ee_pose_rtt_out.write(m_frame_vel.GetFrame());
		port_ee_twist_msr_out.write(m_ee_twist_msr);
	}

	//if(port_ee_twist_ros_in.read(m_ee_twist)==NewData)
	//	tf::TwistMsgToKDL(m_ee_twist,m_twist);
	port_ee_twist_rtt_in.read(m_twist);
	std::stringstream jointName;	
	int ret = pose_to_jnt_solver_->CartToJnt(m_jnt_array.q,m_twist,m_jnt_array.qdot);

	if(ret >= 0)
	{
		for (int i = 0; i < 3; ++i)
		{
			m_Mq_jlc(i,i) = m_Mq_identity(i,i);
		}

		for(unsigned int i=3;i<m_jnt_array.qdot.rows();i++)
		{
			if(joints_norm[i-3] < -0.9 && m_jnt_array.qdot(i) < 0) //joint status: min_limit & counter-clockwise movement
			{
				m_Mq_jlc(i,i) = (m_Mq_identity(i,i) >= 0.0001) ? 1 - pow(joints_norm[i-3],4) : 0.0;
			}

			if(joints_norm[i-3] < -0.9 && m_jnt_array.qdot(i) > 0) //joint status: min_limit & clockwise movement
			{
				m_Mq_jlc(i,i) = (m_Mq_identity(i,i) >= 0.0001) ?  1 : 0.0;
			}

			if(joints_norm[i-3] > 0.9 && m_jnt_array.qdot(i) > 0) //joint status: max_limit & clockwise movement
			{
				m_Mq_jlc(i,i) =  (m_Mq_identity(i,i) >= 0.0001) ? 1 - pow(joints_norm[i-3],4) : 0.0;
			}

			if(joints_norm[i-3] > 0.9 && m_jnt_array.qdot(i) < 0) //joint status: max_limit & counter-clockwise movement
			{
				m_Mq_jlc(i,i) =  (m_Mq_identity(i,i) >= 0.0001) ? 1 : 0.0;
			}
		}

		pose_to_jnt_solver_->setWeightJS(m_Mq_jlc);
		int ret_jlc = pose_to_jnt_solver_->CartToJnt(m_jnt_array.q,m_twist,m_jnt_array.qdot);

		if(ret_jlc>=0)
		{
			// BASE
			for(unsigned int i=0; i < 3; i++)
			{
				if(m_jnt_array.qdot(i) > 0.2)
				{
					m_jnt_array.qdot(i) = 0.2;
				}

				if(m_jnt_array.qdot(i) < -0.2)
				{
					m_jnt_array.qdot(i) = -0.2;
				}				
			}

			m_base_twist.linear.x=m_jnt_array.qdot(0);
			m_base_twist.linear.y=m_jnt_array.qdot(1);
			m_base_twist.angular.z=m_jnt_array.qdot(2);

			// ARM
			for(unsigned int i=3;i<m_jnt_array.qdot.rows();i++)
			{
				//jointName.str("");
				//jointName << "arm_joint_" << (i-2);
				//m_joint_velocities.names[i-3]= jointName.str();

				if(m_jnt_array.qdot(i) > 0.3)
				{
					m_jnt_array.qdot(i) = 0.3;
				}

				if(m_jnt_array.qdot(i) < -0.3)
				{
					m_jnt_array.qdot(i) = -0.3;
				}

				m_joint_velocities.velocities[i-3]= m_jnt_array.qdot(i);
			}
		}
		else
		{
			// BASE
			m_base_twist.linear.x=0;
			m_base_twist.linear.y=0;
			m_base_twist.angular.z=0;
			
			// ARM
			for(unsigned int i=3;i<m_jnt_array.qdot.rows();i++)
			{
				//jointName.str("");
				//jointName << "arm_joint_" << (i-2);
				//m_joint_velocities.names[i-3]= jointName.str();
				m_joint_velocities.velocities[i-3]= 0.0;
			}
			
			log(Error)<<"Could not calculate IVK: " << ret <<endlog();
		}
	}
	else
	{
		// BASE
		m_base_twist.linear.x=0;
		m_base_twist.linear.y=0;
		m_base_twist.angular.z=0;
			
		// ARM
		for(unsigned int i=3;i<m_jnt_array.qdot.rows();i++)
		{
			//jointName.str("");
			//jointName << "arm_joint_" << (i-2);
			//m_joint_velocities.names[i-3]= jointName.str();
			m_joint_velocities.velocities[i-3]= 0.0;
		}
		
		log(Error)<<"Could not calculate IVK: " << ret <<endlog();
	}

	//Reset weight joint space
	pose_to_jnt_solver_->setWeightJS(m_Mq_identity);
	port_joint_velocities_out.write(m_joint_velocities);
	port_base_twist_out.write(m_base_twist);
}

void YouBot_kinematics::stopHook() {
  std::cout << "YouBot_kinematics executes stopping !" <<std::endl;
}

void YouBot_kinematics::cleanupHook() {
  std::cout << "YouBot_kinematics cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(YouBot_kinematics)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(YouBot_kinematics)
