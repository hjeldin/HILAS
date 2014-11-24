#include "IRobotKinematics.hpp"

namespace Hilas
{

IRobotKinematics::IRobotKinematics(std::string const& name, int kine_joint_count, int robot_joint_count):
TaskContext(name), chain_joint_count(kine_joint_count)
{
	this->addEventPort("JointState_in",port_joint_state_in);
	this->addEventPort("BaseOdom_in",port_odom_in);
	this->addEventPort("EETwist_in",port_ee_twist_in);
	this->addEventPort("JointSpaceWeights_in",port_w_js_in);
	this->addEventPort("TaskSpaceWeights_in",port_w_ts_in);

	this->addPort("EEPose_out",port_ee_pose_out);
	this->addPort("JointVelocities_out",port_joint_velocities_out);

	robot_joint_count = robot_joint_count;

	robot_joint_state.name.assign(robot_joint_count,"");
  	robot_joint_state.position.resize(robot_joint_count);
  	robot_joint_state.velocity.resize(robot_joint_count);
  	robot_joint_state.effort.resize(robot_joint_count);

	joints_min_limits.assign(robot_joint_count, 0.0);
	joints_max_limits.assign(robot_joint_count, 0.0);

	m_joint_velocities.names.assign(robot_joint_count,"");
	m_joint_velocities.velocities.assign(robot_joint_count, 0.0);

	joints_norm.assign(robot_joint_count, 0.0);

	m_Mq_identity = Eigen::MatrixXd::Identity(chain_joint_count, chain_joint_count);
	m_Mq_jlc = Eigen::MatrixXd::Identity(chain_joint_count, chain_joint_count);

	m_My_identity = Eigen::MatrixXd::Identity(TASK_SPACE_SIZE, TASK_SPACE_SIZE);
	m_My_jlc = Eigen::MatrixXd::Identity(TASK_SPACE_SIZE, TASK_SPACE_SIZE);

	m_w_js.assign(chain_joint_count, 1.0);
	m_w_ts.assign(TASK_SPACE_SIZE, 1.0);

	std::ifstream file(URDF_PATH_TO_FILE);
	std::cout << "[KINE] Robot urdf loaded from: " << URDF_PATH_TO_FILE << std::endl;

	file.seekg(0, std::ios::end);
	prop_urdf_model.reserve(file.tellg());
	file.seekg(0, std::ios::beg);
	
	prop_urdf_model.assign((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());

	TiXmlDocument xml_doc;
	TiXmlElement* xml_root;

   	xml_doc.Parse(prop_urdf_model.c_str());
   	xml_root = xml_doc.FirstChildElement("robot");
   	
   	if(!xml_root)
   	{
    	log(Error) << "[KINE] Failed to get robot from xml document" << endlog();
      	return;
   	}
   	
   	if(!kdl_parser::treeFromXml(&xml_doc, my_tree))
   	{
    	log(Error) << "[KINE] Failed to construct kdl tree" << endlog();
      	return;
   	}	
}

IRobotKinematics::~IRobotKinematics(){}

void IRobotKinematics::modifyDefaultChain(){}

bool IRobotKinematics::configureHook()
{
	modifyDefaultChain();

	jnt_to_pose_solver.reset(new ChainFkSolverVel_recursive(robot_chain));
	pose_to_jnt_solver.reset(new ChainIkSolverVel_wdls(robot_chain,1.0));

	pose_to_jnt_solver->setLambda(0.5);
	SetToZero(robot_joint_array);
	SetToZero(m_twist);

	pose_to_jnt_solver->setWeightJS(m_Mq_identity);
	pose_to_jnt_solver->setWeightTS(m_My_identity);

  	return true;
}

void IRobotKinematics::assignJointToChain()
{
	for(unsigned int i = 0; i < robot_joint_array.q.rows(); ++i)
	{
		robot_joint_array.q(i) = robot_joint_state.position[i];
		robot_joint_array.qdot(i) = robot_joint_state.velocity[i];
	}
}

bool IRobotKinematics::startHook()
{
	port_joint_state_in.read(robot_joint_state);
    
    assignJointToChain();

	jnt_to_pose_solver->JntToCart(robot_joint_array, m_frame_vel);

	tf::PoseKDLToMsg(m_frame_vel.GetFrame(), m_ee_pose);
  	port_joint_velocities_out.write(m_joint_velocities);
	port_ee_pose_out.write(m_ee_pose);

	return true;
}

void IRobotKinematics::updateHook()
{
	if(port_w_ts_in.read(m_w_ts) == NewData)
	{
		if((int)m_w_ts.size() == m_My_identity.rows())
		{
			for(unsigned int i=0; i<m_My_identity.rows(); ++i)
			{
				m_My_identity(i,i) = m_w_ts[i];				
			}

			pose_to_jnt_solver->setWeightTS(m_My_identity);
		}
		else
		{
			log(Error)<<"[KINE] Data on "<<port_w_ts_in.getName()<<" has the wrong size."<<endlog();
		}
	}

	if(port_w_js_in.read(m_w_js) == NewData)
	{
		if((int)m_w_js.size() == m_Mq_identity.rows())
		{
			for(unsigned int i=0; i<m_Mq_identity.rows(); ++i)
			{
				m_Mq_identity(i,i) = m_w_js[i];
			}

			pose_to_jnt_solver->setWeightJS(m_Mq_identity);
		}
		else
		{
			log(Error)<<"[KINE] Data on "<<port_w_js_in.getName()<<" has the wrong size."<<endlog();			
		}
	}

	forwardKinematic();
	differentialKinematic();
}

void IRobotKinematics::stopHook(){}

void IRobotKinematics::cleanupHook(){}

}