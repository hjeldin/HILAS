#pragma once

#include <IRobotArmService.hpp>

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>
#include <stdio.h>
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
    
    Wam7dofArmService(const string& name, TaskContext* parent);
    ~Wam7dofArmService();

    void displayMotorStatuses();
    void clearControllerTimeouts();

  private:
    
    bool calibrate();
    void cleanup();
    void stop();

    void readJointStates();
    void updateJointSetpoints();
    void checkMotorStatuses();

    const unsigned int m_min_slave_nr;
};

}
