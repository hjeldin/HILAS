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

#include <puma260.hpp>

namespace Puma260
{

using namespace RTT;
using namespace RTT::types;
using namespace std;

class Puma260ArmService: public Hilas::IRobotArmService
{

  public:
    
    Puma260ArmService(const string& name, TaskContext* parent);
    ~Puma260ArmService();

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
