#include "puma260_kinematics-component.hpp"

namespace Puma260
{

Puma260Kinematics::Puma260Kinematics(std::string const& name):
Hilas::IRobotKinematics(name, 6, 6,Puma260::SIZE_JOINT_NAME_ARRAY,Puma260::JOINT_NAME_ARRAY)
{
	joints_min_limits[0] = -0.195774;
	joints_min_limits[1] = -4.77369;
	joints_min_limits[2] = -4.401754; 
	joints_min_limits[3] = -8.727639; 
	joints_min_limits[4] = -3.154507; 
	joints_min_limits[5] = -3.123938;
	
	joints_max_limits[0] = 5.366298; 
	joints_max_limits[1] = 0.84636;
	joints_max_limits[2] = 0.761536;
	joints_max_limits[3] = 2.425219; 
	joints_max_limits[4] = 3.955906; 
	joints_max_limits[5] = 8.313073;
}

Puma260Kinematics::~Puma260Kinematics(){}

void Puma260Kinematics::createKinematicChain()
{
	if(!my_tree.getChain("base_link", "end_effector", robot_chain))
	{
		log(Error) << "[KINE] Failed to construct subchain from urdf model" << endlog();
		return;
	}

	if(robot_chain.getNrOfJoints() != 6)
	{
		log(Error) << "[KINE] Nr of joints in urdf model is not correct: "<< robot_chain.getNrOfJoints() << endlog();
		return;
	}
}

void Puma260Kinematics::forwardKinematic()
{
	bool update_pose = false;

	if(port_joint_state_in.read(robot_joint_state) == NewData)
	{
        //Leave out the base for now, start from 3
		for(unsigned int i=0; i<robot_joint_array.q.rows(); ++i)
		{
			robot_joint_array.q(i) = robot_joint_state.position[i];
			robot_joint_array.qdot(i) = robot_joint_state.velocity[i];

			joints_norm[i] = (((robot_joint_array.q(i) - joints_min_limits[i]) / (joints_max_limits[i] - joints_min_limits[i])) * 2) -1;
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

void Puma260Kinematics::differentialKinematic()
{
	port_ee_twist_in.read(m_twist);

	std::stringstream jointName;	
	int ret = pose_to_jnt_solver->CartToJnt(robot_joint_array.q,m_twist,robot_joint_array.qdot);

	if(ret >= 0)
	{
		for(unsigned int i=0; i<robot_joint_array.qdot.rows(); ++i)
		{
			if(joints_norm[i] < -0.9 && robot_joint_array.qdot(i) < 0) //joint status: min_limit & counter-clockwise movement
			{
				m_Mq_jlc(i,i) = (m_Mq_identity(i,i) >= 0.0001) ? 1 - pow(joints_norm[i],4) : 0.0;
			}

			if(joints_norm[i] < -0.9 && robot_joint_array.qdot(i) > 0) //joint status: min_limit & clockwise movement
			{
				m_Mq_jlc(i,i) = (m_Mq_identity(i,i) >= 0.0001) ?  1 : 0.0;
			}

			if(joints_norm[i] > 0.9 && robot_joint_array.qdot(i) > 0) //joint status: max_limit & clockwise movement
			{
				m_Mq_jlc(i,i) =  (m_Mq_identity(i,i) >= 0.0001) ? 1 - pow(joints_norm[i],4) : 0.0;
			}

			if(joints_norm[i] > 0.9 && robot_joint_array.qdot(i) < 0) //joint status: max_limit & counter-clockwise movement
			{
				m_Mq_jlc(i,i) =  (m_Mq_identity(i,i) >= 0.0001) ? 1 : 0.0;
			}
		}

		pose_to_jnt_solver->setWeightJS(m_Mq_jlc);
		int ret_jlc = pose_to_jnt_solver->CartToJnt(robot_joint_array.q,m_twist,robot_joint_array.qdot);

		if(ret_jlc >= 0)
		{
			for(unsigned int i=0; i<robot_joint_array.qdot.rows(); ++i)
			{
				if(robot_joint_array.qdot(i) > 0.3)
				{
					robot_joint_array.qdot(i) = 0.3;
				}

				if(robot_joint_array.qdot(i) < -0.3)
				{
					robot_joint_array.qdot(i) = -0.3;
				}

				m_joint_velocities.velocities[i] = robot_joint_array.qdot(i);
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

ORO_CREATE_COMPONENT(Puma260::Puma260Kinematics)