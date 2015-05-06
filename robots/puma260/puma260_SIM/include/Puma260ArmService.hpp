#pragma once

#include <IRobotArmService.hpp>

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <puma260.hpp>

namespace Puma260
{

using namespace RTT;
using namespace RTT::types;
using namespace std;

class Puma260ArmService: public Hilas::IRobotArmService
{
  public:
    
    Puma260ArmService(const string& name, TaskContext* parent, long i_clientID);
    ~Puma260ArmService();

    void displayMotorStatuses();
    void clearControllerTimeouts();
    void setControlModesAll(int mode);
 
  private:
    
    bool calibrate();
    void cleanup();
    void stop();
    void update();

    void readJointStates();
    void updateJointSetpoints();
    void checkMotorStatuses();

    vector<simxInt> vrep_joint_handle;
    const unsigned int m_min_slave_nr;
};

}
