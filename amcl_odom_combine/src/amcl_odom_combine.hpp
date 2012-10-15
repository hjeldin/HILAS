#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <tf/tf.h>

using namespace RTT;
using namespace std;
class amcl_odom_combine: public TaskContext
{
  public:
    amcl_odom_combine(const string& name);
    virtual ~amcl_odom_combine();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:
    InputPort<geometry_msgs::PoseWithCovarianceStamped> amcl_pose;
    InputPort<std_msgs::Float64MultiArray> T_base_00;
    OutputPort<std_msgs::Float64MultiArray> H_base_0;
    OutputPort<std_msgs::Float64MultiArray> H_base_baseprev;

    geometry_msgs::PoseWithCovarianceStamped m_amcl_pose;
    std_msgs::Float64MultiArray m_T_base_00;
    std_msgs::Float64MultiArray m_H_base_0;
    std_msgs::Float64MultiArray m_H_base_baseprev;

    //temps
    std_msgs::Float64MultiArray m_H_amcl;
    std_msgs::Float64MultiArray m_H;
    std_msgs::Float64MultiArray m_aldo;

    tf::StampedTransform m_transform_map_odom;

    double m_sampletime;
};
