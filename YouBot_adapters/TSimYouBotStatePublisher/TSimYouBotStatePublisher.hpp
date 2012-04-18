#pragma once

#include <rtt/RTT.hpp>

#include <YouBotTypes.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/typekit/Types.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <ocl/Component.hpp>

namespace YouBot
{
using namespace RTT;
using namespace std;
typedef std_msgs::Float64MultiArray flat_matrix_t;
static const string JOINT_NAME_ARRAY[] =
{"j0","j1","j2","j3","j4","w1","w2","w3","w4","f1","f2"};
//{ "arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br", "f1", "f2" };
static const unsigned int SIZE_JOINT_NAME_ARRAY=11;


void toTransformMatrix(const std::vector<double> & tf, tf::Transform& trans);
typedef struct
{
	InputPort<flat_matrix_t>* H_port;
	std::string Hchild;
	std::string Hname;
} SH_feed;
class TSimYouBotStatePublisher: public RTT::TaskContext
{
public:
	TSimYouBotStatePublisher(std::string const& name);
	~TSimYouBotStatePublisher();
	bool addHfeed(std::string Hchild, std::string Hname);
	bool startHook();
	void updateHook();
	bool configureHook();
	std::string topic_name;
private:
	InputPort<flat_matrix_t> joints_angles;
	InputPort<flat_matrix_t> wheels_angles;
	InputPort<flat_matrix_t> odometry;
	std::vector<SH_feed> H_feeds;
	ros::NodeHandle node_handle;
	ros::Publisher joint_pub;
	unsigned int m_dimension;
	double wheel;
};

}

