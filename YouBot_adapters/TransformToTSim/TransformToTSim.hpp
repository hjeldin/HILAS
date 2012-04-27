#pragma once

#include <rtt/RTT.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/typekit/Types.h>

using namespace RTT;
using namespace std;
class TransformToTSim : public RTT::TaskContext
{
  public:
    TransformToTSim(string const& name);
    ~TransformToTSim();

    bool startHook() ;
    void updateHook() ;

  private:
    OutputPort<std_msgs::Float64MultiArray > output_transform;

    InputPort< geometry_msgs::TransformStamped > input_transform;

    std_msgs::Float64MultiArray m_output_transform;

    geometry_msgs::TransformStamped m_input_transform;
};


