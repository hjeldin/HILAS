#pragma once

#include <IRobotBaseService.hpp>

#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotJoint.hpp>
#include <youbot/ProtocolDefinitions.hpp>
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>

#include "youbot/YouBotBase.hpp"
#include "generic/ConfigFile.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp"

#include <generic/Units.hpp>
#include <youBot.hpp>

namespace YouBot
{

using namespace RTT;
using namespace RTT::types;
using namespace std;
using namespace youbot;

class YouBotBaseService: public Hilas::IRobotBaseService
{

  public:

    YouBotBaseService(const string& name, TaskContext* parent, long clientID);
    ~YouBotBaseService();

    void displayMotorStatuses();
    void clearControllerTimeouts();
    void setControlModesAll(int mode);
    
  private:

    bool calibrate();
    void cleanup();
    void stop();
    void update();

    void readJointStates();
    void readOdometry();
    void setJointSetpoints();
    void setTwistSetpoints();
    void checkMotorStatuses();

    long m_clientID;
    simxInt all_robot_handle;
    vector<simxInt> vrep_joint_handle;

    std::vector<double> joint_base_position_prev;
    std::vector<quantity<plane_angle> > odom_wheelPositions;

    youbot::FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;
    boost::scoped_ptr<youbot::ConfigFile> configfile;
    youbot::FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;

    const unsigned int m_min_slave_nr;
};

}
