#pragma once

#include <rtt/RTT.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/typekit/Types.h>

using namespace RTT;
using namespace std;
class TSimToWrench : public RTT::TaskContext
{
  public:
    TSimToWrench(string const& name);
    ~TSimToWrench();

    bool startHook() ;
    void updateHook() ;

  private:
    InputPort<std_msgs::Float64MultiArray > input_wrench;

    OutputPort< geometry_msgs::Wrench > output_wrench;

    std_msgs::Float64MultiArray m_input_wrench;

    geometry_msgs::Wrench m_output_wrench;
};


