#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include <std_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>

#include <generic/Units.hpp>

#include "YouBotSIM.hpp"
#include "YouBotTypes.hpp"

#include "youbot/YouBotBase.hpp"
#include "generic/ConfigFile.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp"

#include "YouBotHelpers.hpp"

#define VREP_JOINT_CONTROL_POSITION_IP 2001

namespace YouBot
{
    using namespace RTT;
    using namespace std;
    using namespace boost;
    using namespace KDL;

class YouBotBaseService: public Service
{

  public:
    YouBotBaseService(const string& name, TaskContext* parent,
        unsigned int min_slave_nr, long i_clientID);
    virtual ~YouBotBaseService();

    void setControlModesAll(int mode);
    void setControlModes(vector<ctrl_modes>& all);
    void getControlModes(vector<ctrl_modes>& all);
    void setsim_mode(int mode);
    void displayMotorStatuses();

    void clearControllerTimeouts();

  protected:
    OutputPort<sensor_msgs::JointState> joint_state;
    OutputPort<nav_msgs::Odometry> odometry_state;

    InputPort<motion_control_msgs::JointVelocities> joint_velocity_command;
    InputPort<motion_control_msgs::JointPositions> joint_position_command;
    InputPort<motion_control_msgs::JointEfforts> joint_effort_command;

    InputPort<geometry_msgs::Twist> cmd_twist;

    OutputPort<std::string> events;

    /* Acquire Data
       HW -> read from YuoBot_OODL
       SIM -> read from VREP
    */

    InputPort<sensor_msgs::JointState> in_joint_state;
    InputPort<nav_msgs::Odometry> in_odometry_state;

  private:
    void setupComponentInterface();

    bool calibrate();
    bool start();
    void update();
    void cleanup();
    void stop();

    void readJointStates();
    void readOdometry();

    void setJointSetpoints();
    void setTwistSetpoints();

    void checkMotorStatuses();

    /* Simulator data */
    long m_clientID;
    simxInt all_robot_handle;
    vector<simxInt> vrep_joint_handle;
    bool is_in_visualization_mode;

    std::vector<double> joint_base_position_prev;
    std::vector<quantity<plane_angle> > odom_wheelPositions;

    youbot::FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;
    boost::scoped_ptr<youbot::ConfigFile> configfile;
    youbot::FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;

    motion_control_msgs::JointVelocities m_joint_velocity_command;
    motion_control_msgs::JointPositions m_joint_position_command;
    motion_control_msgs::JointEfforts m_joint_effort_command;

    sensor_msgs::JointState m_joint_state;
    nav_msgs::Odometry m_odometry_state;
    geometry_msgs::Twist m_cmd_twist;

    std::string m_events;

    vector<ctrl_modes> m_joint_ctrl_modes;

    bool m_overcurrent[NR_OF_BASE_SLAVES];
    bool m_undervoltage[NR_OF_BASE_SLAVES];
    bool m_overvoltage[NR_OF_BASE_SLAVES];
    bool m_overtemperature[NR_OF_BASE_SLAVES];
    bool m_connectionlost[NR_OF_BASE_SLAVES];
    bool m_i2texceeded[NR_OF_BASE_SLAVES];
    bool m_timeout[NR_OF_BASE_SLAVES];

    bool m_calibrated;

    const unsigned int m_min_slave_nr;

    YouBotSIM* m_VREP;
};

}
