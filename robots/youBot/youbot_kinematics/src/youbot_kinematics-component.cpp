#include "youbot_kinematics-component.hpp"

namespace YouBot
{

YouBotKinematics::YouBotKinematics(std::string const& name):
Hilas::IRobotKinematics(name, 8, 15,YouBot::SIZE_JOINT_NAME_ARRAY,YouBot::JOINT_NAME_ARRAY)
{
    configfile.reset(new youbot::ConfigFile("youbot-base.cfg",CFG_YOUBOT_BASE));
    configfile->readInto(kinematicConfig.rotationRatio, "YouBotKinematic", "RotationRatio");   
    configfile->readInto(kinematicConfig.slideRatio, "YouBotKinematic", "SlideRatio");

    double dummy = 0;
    
    configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontAndRearWheels_[meter]");
    kinematicConfig.lengthBetweenFrontAndRearWheels = dummy * meter;
    configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontWheels_[meter]");
    kinematicConfig.lengthBetweenFrontWheels = dummy * meter;
    configfile->readInto(dummy, "YouBotKinematic", "WheelRadius_[meter]");
    kinematicConfig.wheelRadius = dummy * meter;
    youBotBaseKinematic.setConfiguration(kinematicConfig);

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
}

YouBotKinematics::~YouBotKinematics(){}

void YouBotKinematics::createKinematicChain()
{
	//Let's assume the base has two translational joints and a rotational joint: Base joints
	robot_chain.addSegment(Segment(Joint(Joint::TransX)));
	robot_chain.addSegment(Segment(Joint(Joint::TransY)));
	//The base of the arm is located 16 cm in front of the center of the platform
	robot_chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.143,0.0, 0.096 + 0.046)))); 

    // Add arm chain from urdf to existing chain
	KDL::Chain arm_chain;

	if(!my_tree.getChain("arm_link_0", "gripper_palm_link", arm_chain))
	{
		log(Error) << "[KINE] Failed to construct subchain from urdf model" << endlog();
		return;
	}

	if(arm_chain.getNrOfJoints() != 5)
	{
		log(Error) << "[KINE] Nr of joints in urdf model is not correct: "<<arm_chain.getNrOfJoints() << endlog();
		return;
	}

	robot_chain.addChain(arm_chain);
}

void YouBotKinematics::assignJointToChain()
{
	for(unsigned int i=3; i<robot_joint_array.q.rows(); ++i)
	{
		robot_joint_array.q(i) = robot_joint_state.position[i-3];
		robot_joint_array.qdot(i) = robot_joint_state.velocity[i-3];
	}
}

void YouBotKinematics::forwardKinematic()
{
	bool update_pose = false;

	if(port_odom_in.read(m_odom) == NewData)
	{
		robot_joint_array.q(0) = m_odom.pose.pose.position.x;
		robot_joint_array.q(1) = m_odom.pose.pose.position.y;

		KDL::Rotation rot = KDL::Rotation::Quaternion(
			m_odom.pose.pose.orientation.x,
			m_odom.pose.pose.orientation.y,
			m_odom.pose.pose.orientation.z,
			m_odom.pose.pose.orientation.w);

		double roll,pitch,yaw;
		rot.GetRPY(roll,pitch,yaw);

		robot_joint_array.q(2) = yaw;
		robot_joint_array.qdot(0) = m_odom.twist.twist.linear.x;
		robot_joint_array.qdot(1) = m_odom.twist.twist.linear.y;
		robot_joint_array.qdot(3) = m_odom.twist.twist.angular.z;

		update_pose = true;
	}

	if(port_joint_state_in.read(robot_joint_state) == NewData)
	{
        //Leave out the base for now, start from 3
		for(unsigned int i=3; i<robot_joint_array.q.rows(); ++i)
		{
			robot_joint_array.q(i) = robot_joint_state.position[i-3];
			robot_joint_array.qdot(i) = robot_joint_state.velocity[i-3];

			joints_norm[i-3] = (((robot_joint_array.q(i) - joints_min_limits[i-3]) / (joints_max_limits[i-3] - joints_min_limits[i-3])) * 2) -1;
		}
		update_pose = true;
	}

	if(update_pose)
	{
		int ret = jnt_to_pose_solver->JntToCart(robot_joint_array,m_frame_vel);

		if(ret < 0)
		{
			log(Error)<<"[KINE] Could not calculate FK: " << ret <<endlog();
		}

		tf::PoseKDLToMsg(m_frame_vel.GetFrame(),m_ee_pose);
		port_ee_pose_out.write(m_ee_pose);
	}
}

void YouBotKinematics::twistToJointVelocities(quantity<si::velocity> l,quantity<si::velocity> t, quantity<si::angular_velocity> a)
{
	std::vector<quantity<angular_velocity> > wheelVelocities;
	youBotBaseKinematic.cartesianVelocityToWheelVelocities(l, t, a, wheelVelocities);

	m_joint_velocities.velocities[5] = wheelVelocities[0].value();
	m_joint_velocities.velocities[6] = wheelVelocities[1].value();
	m_joint_velocities.velocities[7] = wheelVelocities[2].value();
	m_joint_velocities.velocities[8] = wheelVelocities[3].value();		
}

void YouBotKinematics::differentialKinematic()
{
	port_ee_twist_in.read(m_twist);

	std::stringstream jointName;	
	int ret = pose_to_jnt_solver->CartToJnt(robot_joint_array.q,m_twist,robot_joint_array.qdot);

	if(ret >= 0)
	{
		for (int i = 0; i < 3; ++i)
		{
			m_Mq_jlc(i,i) = m_Mq_identity(i,i);
		}

		for(unsigned int i=3; i<robot_joint_array.qdot.rows(); ++i)
		{
			if(joints_norm[i-3] < -0.9 && robot_joint_array.qdot(i) < 0) //joint status: min_limit & counter-clockwise movement
			{
				m_Mq_jlc(i,i) = (m_Mq_identity(i,i) >= 0.0001) ? 1 - pow(joints_norm[i-3],4) : 0.0;
			}

			if(joints_norm[i-3] < -0.9 && robot_joint_array.qdot(i) > 0) //joint status: min_limit & clockwise movement
			{
				m_Mq_jlc(i,i) = (m_Mq_identity(i,i) >= 0.0001) ?  1 : 0.0;
			}

			if(joints_norm[i-3] > 0.9 && robot_joint_array.qdot(i) > 0) //joint status: max_limit & clockwise movement
			{
				m_Mq_jlc(i,i) =  (m_Mq_identity(i,i) >= 0.0001) ? 1 - pow(joints_norm[i-3],4) : 0.0;
			}

			if(joints_norm[i-3] > 0.9 && robot_joint_array.qdot(i) < 0) //joint status: max_limit & counter-clockwise movement
			{
				m_Mq_jlc(i,i) =  (m_Mq_identity(i,i) >= 0.0001) ? 1 : 0.0;
			}
		}

		pose_to_jnt_solver->setWeightJS(m_Mq_jlc);
		int ret_jlc = pose_to_jnt_solver->CartToJnt(robot_joint_array.q,m_twist,robot_joint_array.qdot);

		if(ret_jlc >= 0)
		{
			// BASE
			for(unsigned int i=0; i < 3; i++)
			{
				if(robot_joint_array.qdot(i) > 0.2)
				{
					robot_joint_array.qdot(i) = 0.2;
				}

				if(robot_joint_array.qdot(i) < -0.2)
				{
					robot_joint_array.qdot(i) = -0.2;
				}				
			}

			twistToJointVelocities(
				robot_joint_array.qdot(0) * meter_per_second,
				robot_joint_array.qdot(1) * meter_per_second,
				robot_joint_array.qdot(2) * radian_per_second);

			// ARM
			for(unsigned int i=3; i<robot_joint_array.qdot.rows(); ++i)
			{
				if(robot_joint_array.qdot(i) > 0.3)
				{
					robot_joint_array.qdot(i) = 0.3;
				}

				if(robot_joint_array.qdot(i) < -0.3)
				{
					robot_joint_array.qdot(i) = -0.3;
				}

				m_joint_velocities.velocities[i-3] = robot_joint_array.qdot(i);
			}
		}
		else
		{
			for(unsigned int i=0; i<robot_joint_count; ++i)
			{
				m_joint_velocities.velocities[i] = 0.0;
			}
			
			log(Error)<<"[KINE] Could not calculate IVK: " << ret <<endlog();
		}
	}
	else
	{
		for(unsigned int i=0; i<robot_joint_count; ++i)
		{
			m_joint_velocities.velocities[i] = 0.0;
		}
		
		log(Error)<<"[KINE] Could not calculate IVK: " << ret <<endlog();
	}

	//Reset weight joint space
	pose_to_jnt_solver->setWeightJS(m_Mq_identity);
	port_joint_velocities_out.write(m_joint_velocities);
}

}

ORO_CREATE_COMPONENT(YouBot::YouBotKinematics)