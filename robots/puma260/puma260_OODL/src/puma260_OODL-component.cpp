#include "puma260_OODL-component.hpp"

namespace Puma260
{

Puma260OODL::Puma260OODL(const string& name):
Hilas::IRobot(name,"puma260",-1)
{
	for (int i = 0; i < arm_count; ++i)
	{
		arm_a.push_back(Service::shared_ptr(new Puma260ArmService(std::string("Arm")+boost::lexical_cast<std::string>(i+1),this)));
		grip_a.push_back(Service::shared_ptr(new Puma260GripperService(std::string("Gripper")+boost::lexical_cast<std::string>(i+1),this)));	    	
	}
}

Puma260OODL::~Puma260OODL() {}

bool Puma260OODL::configureHook()
{
	Hilas::IRobot::configureHook();
}

void Puma260OODL::updateHook()
{
    // Time measurements to confirm period
	Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince( timestamp );
	ca = ca + (elapsed - ca) / (++ca_counter);
    timestamp = RTT::os::TimeService::Instance()->getTicks(); //used cumulative average, because timestamps can overflow for long operation times.

	Hilas::IRobot::updateHook();
}

}
ORO_CREATE_COMPONENT(Puma260::Puma260OODL)