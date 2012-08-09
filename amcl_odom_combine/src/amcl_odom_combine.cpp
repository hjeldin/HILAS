#include "amcl_odom_combine.hpp"
#include <tf/transform_listener.h>
#include <ocl/Component.hpp>

#include <vector>

using namespace RTT;
using namespace std;

amcl_odom_combine::amcl_odom_combine(const string& name) :
    TaskContext(name, PreOperational)
{
  m_H_base_0.data.resize(16, 0.0);

  H_base_0.setDataSample(m_H_base_0);
 
  this->addPort("amcl_pose", amcl_pose).doc("");
  this->addPort("T", T).doc("");
  this->addPort("H_base_0", H_base_0).doc("combined odom");
}

amcl_odom_combine::~amcl_odom_combine()
{
}

bool amcl_odom_combine::configureHook()
{
  tf::TransformListener listener;
  listener.waitForTransform("/map", "/odom", ros::Time::now(), ros::Duration(5.0));
  
  try{
    listener.lookupTransform("/map", "/odom", ros::Time(0), m_transform_map_odom);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
}
  
  return TaskContext::configureHook();
}

bool amcl_odom_combine::startHook()
{
  if (!amcl_pose.connected())
  {
    log(Error) << "amcl_pose not connected." << endlog();
    return false;
  }

  if (!T.connected())
  {
    log(Error) << "T not connected." << endlog();
    return false;
  }

  if (!H_base_0.connected())
  {
    log(Error) << "H_base_0 not connected." << endlog();
    return false;
  }

  return TaskContext::startHook();
}

void makeHMatrixFromQuaternion(vector<double>& H, geometry_msgs::Quaternion& quat, geometry_msgs::Point& pose);

void amcl_odom_combine::updateHook()
{
  TaskContext::updateHook();
 
  if(amcl_pose.read(m_amcl_pose) == NewData)
  {
    makeHMatrixFromQuaternion(m_H_base_0.data, m_amcl_pose.pose.pose.orientation, m_amcl_pose.pose.pose.position);
  }
  //m_T
  //m_transform_map_odom
  
  // magic done by JAN! ! ! ! 

  //output
  H_base_0.write(m_H_base_0);
}

void amcl_odom_combine::stopHook()
{
  

  TaskContext::stopHook();
}

void amcl_odom_combine::cleanupHook()
{
  TaskContext::cleanupHook();
}

void makeHMatrixFromQuaternion(vector<double>& H, geometry_msgs::Quaternion& quat, geometry_msgs::Point& pose)
{
    assert(H.size() == 16);

    double qw, qx, qy, qz;
    qw = quat.w;
    qx = quat.x;
    qy = quat.y;
    qz = quat.z;

    H[0] = pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2);
    H[1] = 2*qx*qy - 2*qz*qw;
    H[2] = 2*qx*qz + 2*qy*qw;

    H[4] = 2*qx*qy + 2*qz*qw;
    H[5] = pow(qw,2) - pow(qx,2) + pow(qy,2) - pow(qz,2);
    H[6] = 2*qy*qz - 2*qx*qw;

    H[8] = 2*qx*qz - 2*qy*qw;
    H[9] = 2*qy*qz + 2*qx*qw;
    H[10] = pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2);

    H[3] = pose.x;
    H[7] = pose.y;
    H[11] = pose.z;

    H[12] = 0;
    H[13] = 0;
    H[14] = 0;
    H[15] = 1;
}

ORO_CREATE_COMPONENT( amcl_odom_combine )
