#include "wam7dof_kinematics-component.hpp"

namespace Wam7dof
{

Wam7dofKinematics::Wam7dofKinematics(std::string const& name):
Hilas::IRobotKinematics(name, 8, 8,Wam7dof::SIZE_JOINT_NAME_ARRAY,Wam7dof::JOINT_NAME_ARRAY)
{
	joints_min_limits[1] = -2.6;
	joints_min_limits[2] = -1.985;
	joints_min_limits[3] = -2.8;
	joints_min_limits[4] = -0.9; 
	joints_min_limits[5] = -4.55; 
	joints_min_limits[6] = -1.5707; 
	joints_min_limits[7] = -3.0;

	joints_min_limits[0] = 0;
	joints_min_limits[8] = 0;


	joints_max_limits[1] = 2.6; 
	joints_max_limits[2] = 1.985;
	joints_max_limits[3] = 2.8;
	joints_max_limits[4] = 3.14159265359;
	joints_max_limits[5] = 1.25; 
	joints_max_limits[6] = 1.5707; 
	joints_max_limits[7] = 3.0;
	
	joints_max_limits[0] = 0;
	joints_max_limits[8] = 0;

}

Wam7dofKinematics::~Wam7dofKinematics(){}

void Wam7dofKinematics::createKinematicChain()
{
	if(!my_tree.getChain("base_link", "wrist_palm_link", robot_chain))
	{
		log(Error) << "[KINE] Failed to construct subchain from urdf model" << endlog();
		return;
	}

	if(robot_chain.getNrOfJoints() != 7)
	{
		log(Error) << "[KINE] Nr of joints in urdf model is not correct: "<< robot_chain.getNrOfJoints() << endlog();
		return;
	}
}

void Wam7dofKinematics::forwardKinematic()
{
	bool update_pose = false;

	if(port_joint_state_in.read(robot_joint_state) == NewData)
	{
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

void Wam7dofKinematics::differentialKinematic()
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

ORO_CREATE_COMPONENT(Wam7dof::Wam7dofKinematics)