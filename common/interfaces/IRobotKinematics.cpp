#include "IRobotKinematics.hpp"

namespace Hilas
{

IRobotKinematics::IRobotKinematics(std::string const& name, int kine_joint_count, int robot_joint_count):
TaskContext(name), robot_jnt_array(kine_joint_count), 
{
	this->addEventPort("JointState_in",port_joint_state_in);
	this->addEventPort("BaseOdom_in",port_odom_in);
	this->addEventPort("EETwist_in",port_ee_twist_ros_in);
	this->addEventPort("EETwistRTT_in",port_ee_twist_rtt_in);
	this->addEventPort("JointSpaceWeights_in",port_w_js_in);
	this->addEventPort("TaskSpaceWeights_in",port_w_ts_in);

	this->addPort("EEPose_out",port_ee_pose_ros_out);
	this->addPort("EEPoseRTT_out",port_ee_pose_rtt_out);
	this->addPort("EETwistMsr_out",port_ee_twist_msr_out);
	this->addPort("JointVelocities_out",port_joint_velocities_out);

	m_joint_state.name.assign(robot_joint_count,"");
  	m_joint_state.position.resize(robot_joint_count);
  	m_joint_state.velocity.resize(robot_joint_count);
  	m_joint_state.effort.resize(robot_joint_count);

	joints_min_limits.assign(robot_joint_count, 0.0);
	joints_max_limits.assign(robot_joint_count, 0.0);

	m_joint_velocities.names.assign(robot_joint_count,"");
	m_joint_velocities.velocities.assign(robot_joint_count, 0.0);

	joints_norm.assign(robot_joint_count, 0.0);

	m_Mq_identity = MatrixXd::Identity(kine_joint_count, kine_joint_count);
	m_Mq_jlc = MatrixXd::Identity(kine_joint_count, kine_joint_count);

	m_My_identity = MatrixXd::Identity(TASK_SPACE_SIZE, TASK_SPACE_SIZE);
	m_My_jlc = MatrixXd::Identity(TASK_SPACE_SIZE, TASK_SPACE_SIZE);

	m_w_js.assign(kine_joint_count, 1.0);
	m_w_ts.assign(TASK_SPACE_SIZE, 1.0);

	std::ifstream file(URDF_PATH_TO_FILE);
	std::cout << "[KINE] Robot urdf loaded from: " << URDF_PATH_TO_FILE << std::endl;

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
    	log(Error) << "[KINE] Failed to get robot from xml document" << endlog();
      	return;
   	}
   	
   	if(!kdl_parser::treeFromXml(&xml_doc, my_tree))
   	{
    	log(Error) << "[KINE] Failed to construct kdl tree" << endlog();
      	return;
   	}	
}

void modifyDefaultChain(){}

bool IRobotKinematics::configureHook()
{
	modifyDefaultChain();

	jnt_to_pose_solver_.reset(new ChainFkSolverVel_recursive(robot_chain));
	pose_to_jnt_solver_.reset(new ChainIkSolverVel_wdls(robot_chain,1.0));

	pose_to_jnt_solver_->setLambda(0.5);
	SetToZero(robot_jnt_array);
	
	pose_to_jnt_solver_->setWeightJS(m_Mq_identity);
	pose_to_jnt_solver_->setWeightTS(m_My_identity);

  	return true;
}

bool IRobotKinematics::startHook()
{
	port_joint_state_in.read(m_joint_state);
    
    //Leave out the base for now, start from 3
	for(unsigned int i=3;i<robot_jnt_array.q.rows();i++)
	{
		robot_jnt_array.q(i)=m_joint_state.position[i-3];
		robot_jnt_array.qdot(i)=m_joint_state.velocity[i-3];
	}

	jnt_to_pose_solver_->JntToCart(robot_jnt_array,m_frame_vel);

	tf::PoseKDLToMsg(m_frame_vel.GetFrame(),m_ee_pose);
	tf::TwistKDLToMsg(m_frame_vel.GetTwist(),m_ee_twist_msr);

  	port_joint_velocities_out.write(m_joint_velocities);
  	port_base_twist_out.write(m_ee_twist);

	port_ee_pose_ros_out.write(m_ee_pose);
	port_ee_pose_rtt_out.write(m_frame_vel.GetFrame());
	port_ee_twist_msr_out.write(m_ee_twist_msr);

	return true;
}

void IRobotKinematics::stopHook(){}

void IRobotKinematics::cleanupHook(){}