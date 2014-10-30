#include <map>
#include <string>
#include <algorithm>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <cstdlib>

robot_state_publisher::RobotStatePublisher* rsp;
std::string robot_name;

std::string my_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::map<std::string, double> robot_map;

    for(int i = 0; i < msg->position.size(); ++i)
    {
        robot_map[msg->name[i]] = msg->position[i];
    }

    rsp->publishTransforms(robot_map, ros::Time::now(),my_tolower(robot_name));
    rsp->publishFixedTransforms(my_tolower(robot_name));
}

void odomStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped odometryTransform;
    
    odometryTransform.header.stamp = ros::Time::now();
    odometryTransform.header.frame_id = my_tolower(robot_name) + std::string("/odom");
    odometryTransform.child_frame_id = my_tolower(robot_name) + std::string("/base_footprint");
    odometryTransform.transform.translation.x = msg->pose.pose.position.x;
    odometryTransform.transform.translation.y = msg->pose.pose.position.y;
    odometryTransform.transform.translation.z = msg->pose.pose.position.z;
    odometryTransform.transform.rotation = msg->pose.pose.orientation;

    br.sendTransform(odometryTransform);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_state_publisher");
    ros::NodeHandle n;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/hilas.ini", pt);

    robot_name = pt.get<std::string>("robot.name");
    std::string prop_urdf_model;

    std::ifstream file((std::string(getenv("HILAS_HOME")) + "/robots/" + robot_name + "/"+ my_tolower(robot_name) + "_description/robots/" + my_tolower(robot_name) + ".urdf").c_str());

    file.seekg(0, std::ios::end);
    prop_urdf_model.reserve(file.tellg());
    file.seekg(0, std::ios::beg);
    
    prop_urdf_model.assign((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
    ros::param::set("robot_description", prop_urdf_model);

    KDL::Tree my_tree;
    TiXmlDocument xml_doc;
    TiXmlElement* xml_root;

    xml_doc.Parse(prop_urdf_model.c_str());
    xml_root = xml_doc.FirstChildElement("robot");
    
    if(!xml_root)
    {
        return false;
    }
    
    if (!kdl_parser::treeFromXml(&xml_doc, my_tree))
    {
        return false;
    }

    rsp = new robot_state_publisher::RobotStatePublisher(my_tree);
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1000, jointStateCallback);

    ros::Subscriber sub_odom;

    if(pt.get<int>("robot.baseCount") > 0)
    {
        sub_odom = n.subscribe<nav_msgs::Odometry>("/odom", 1000, odomStateCallback);        
    }

    ros::spin();

    delete rsp;

    return 0;
}