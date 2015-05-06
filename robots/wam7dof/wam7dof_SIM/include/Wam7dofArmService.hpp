#pragma once

#include <IRobotArmService.hpp>

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <wam7dof.hpp>

namespace Wam7dof
{

using namespace RTT;
using namespace RTT::types;
using namespace std;

class Wam7dofArmService: public Hilas::IRobotArmService
{
  public:
    
    Wam7dofArmService(const string& name, TaskContext* parent, long i_clientID);
    ~Wam7dofArmService();

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
