#include "amcl_odom_combine.hpp"
#include <tf/transform_listener.h>
#include <ocl/Component.hpp>

#include <vector>
#include "xxmatrix.hpp"

using namespace RTT;
using namespace std;
using namespace xxmatrix;

amcl_odom_combine::amcl_odom_combine(const string& name) :
    TaskContext(name, PreOperational)
{
  // Inputs
  m_T_base_00.data.resize(6, 0.0);

  // Outputs
  m_H_base_0.data.assign(EYE4, EYE4 + SIZE_H);
  m_H_base_baseprev.data.assign(EYE4, EYE4 + SIZE_H);

  H_base_0.setDataSample(m_H_base_0);
  H_base_baseprev.setDataSample(m_H_base_baseprev);
 
  this->addPort("amcl_pose", amcl_pose).doc("H_base_0 update based on extra sensory information.");
  this->addPort("T_base_00", T_base_00).doc("Base Twist from sensors");
  this->addPort("H_base_0", H_base_0).doc("Combined base pose");
  this->addPort("H_base_baseprev", H_base_baseprev).doc("");

  m_H_amcl.data.assign(EYE4, EYE4 + SIZE_H);
  m_H.data.assign(EYE4, EYE4 + SIZE_H);
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

  if (!T_base_00.connected())
  {
    log(Error) << "T not connected." << endlog();
    return false;
  }

  if (!H_base_0.connected())
  {
    log(Error) << "H_base_0 not connected." << endlog();
    return false;
  }

  if (!H_base_baseprev.connected())
  {
    log(Error) << "H_base_baseprev not connected." << endlog();
    return false;
  }

  m_sampletime = TaskContext::getPeriod();

  return TaskContext::startHook();
}

void makeHMatrixFromQuaternion(vector<double>& H, geometry_msgs::Quaternion& quat, geometry_msgs::Point& pose);

void amcl_odom_combine::updateHook()
{
  TaskContext::updateHook();
 
  //  if update then
  if(amcl_pose.read(m_amcl_pose) == NewData)
  {
    // H_base_baseprev=eye(4);
    m_H_base_baseprev.data.assign(EYE4, EYE4+SIZE_H);
    // input  from amcl
    makeHMatrixFromQuaternion(m_H_amcl.data, m_amcl_pose.pose.pose.orientation, m_amcl_pose.pose.pose.position);
  }

  T_base_00.read(m_T_base_00);

//  H_base_baseprev=aldo*H; // output to amcl at any point in time
//  aldo=H_base_baseprev;
  computeFiniteTwist(m_H.data, m_T_base_00.data, m_sampletime);
  mulMatrixMatrixSquare(m_H_base_baseprev.data, m_H_base_baseprev.data, m_H.data, 4);

//  H_base_0=Hamcl*H_base_baseprev; //output to controller
  mulMatrixMatrixSquare(m_H_base_0.data, m_H_amcl.data, m_H_base_baseprev.data, 4);

  //output
  H_base_0.write(m_H_base_0);
  H_base_baseprev.write(m_H_base_baseprev);
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
