#include "TSimYouBotStatePublisher.hpp"

//#include <rtt/types/SequenceTypeInfo.hpp>

/*
 * @brief Adapter to connect 20Sim to the YouBot OODL.
 */
namespace YouBot
{
using namespace RTT;

TSimYouBotStatePublisher::TSimYouBotStatePublisher(std::string const& name) :
				TaskContext(name, PreOperational), topic_name("youbot_robot_state"), m_dimension(
						0), wheel(0)
{
	this->addOperation("addHfeed", &TSimYouBotStatePublisher::addHfeed, this,
			OwnThread);
	this->addPort("joints_angles", joints_angles).doc(
			"this port is sync for the joint states republished in rviz");
	this->addPort("wheels_angles", wheels_angles).doc(
			"this port is sync for the wheels_angles republished in rviz");
	this->addPort("odometry", odometry).doc(
			"this port is sync for the odometry republished in rviz. Not implemented");
}

TSimYouBotStatePublisher::~TSimYouBotStatePublisher()
{

}

bool TSimYouBotStatePublisher::addHfeed(std::string Hchild, std::string Hname)
{
	if (this->isConfigured())
	{
		log(Error)
						<< "This operation is only accessible before system is configured"
						<< endlog();
		return false;
	}
	SH_feed temp;
	temp.Hchild = Hchild;
	temp.Hname = Hname;
	temp.H_port = new RTT::InputPort<flat_matrix_t>;
	this->addPort(temp.Hname, *(temp.H_port)).doc(temp.Hname);
	H_feeds.push_back(temp);
	return true;
}

bool TSimYouBotStatePublisher::configureHook()
{

	for (std::vector<SH_feed>::iterator it = H_feeds.begin();
			it != H_feeds.end(); ++it)
	{
		if (!it->H_port->connected())
		{
			log(Error) << "The port " << it->Hname << " is not connected"
					<< endlog();
		}
	}
	if (!joints_angles.connected())
	{
		log(Error) << "The port joints_angles is not connected" << endlog();

	}
	if (!wheels_angles.connected())
	{
		log(Error) << "The port wheels angles is not connected" << endlog();

	}
	if (odometry.connected())
	{
		log(Error) << "This port republishing is not implemented" << endlog();

	}
	joint_pub = node_handle.advertise<sensor_msgs::JointState>(
			"youbot_robot_state", 1);
	return TaskContext::configureHook();
}
bool TSimYouBotStatePublisher::startHook()
{

	return TaskContext::startHook();
}

void TSimYouBotStatePublisher::updateHook()
{
	flat_matrix_t input_temp;
	for (std::vector<SH_feed>::iterator it = H_feeds.begin();
			it != H_feeds.end(); ++it)
	{
		if (it->H_port->read(input_temp) == NewData)
		{
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			toTransformMatrix(input_temp.data, transform);
			br.sendTransform(
					tf::StampedTransform(transform, ros::Time::now(),
							"base_link", topic_name));
		}
	}
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.assign(JOINT_NAME_ARRAY,JOINT_NAME_ARRAY+SIZE_JOINT_NAME_ARRAY);
	joint_state.position.resize(SIZE_JOINT_NAME_ARRAY,0);
	if (joints_angles.read(input_temp) == NewData)
	{
		joint_state.position[0] = input_temp.data[0];
		joint_state.position[1] = input_temp.data[1];
		joint_state.position[2] = input_temp.data[2];
		joint_state.position[3] = input_temp.data[3];
		joint_state.position[4] = input_temp.data[4];
	}
	if (wheels_angles.read(input_temp) == NewData)
	{
		joint_state.position[5] = input_temp.data[0];
		joint_state.position[6] = input_temp.data[1];
		joint_state.position[7] = input_temp.data[2];
		joint_state.position[8] = input_temp.data[3];
	}
//	if (odometry.read(input_temp) == NewData)
	{
		joint_state.position[9] = 0.01;
		joint_state.position[10] = 0.01;
		//send the joint state and transform
	}
	joint_pub.publish(joint_state);
	TaskContext::updateHook();
}
// supplementary function
void toTransformMatrix(const std::vector<double> & tf, tf::Transform& trans)
{
	trans.setOrigin(tf::Vector3(tf[3], tf[7], tf[11]));
	btMatrix3x3 rotMatrix(tf[0], tf[1], tf[2], tf[4], tf[5], tf[6], tf[8],
			tf[9], tf[10]);
	tf::Quaternion quat;
	rotMatrix.getRotation(quat);
	trans.setRotation(quat);

}
}

ORO_CREATE_COMPONENT( YouBot::TSimYouBotStatePublisher)
