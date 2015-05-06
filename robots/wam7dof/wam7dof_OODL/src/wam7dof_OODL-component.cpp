#include "wam7dof_OODL-component.hpp"

namespace Wam7dof
{

Wam7dofOODL::Wam7dofOODL(const string& name):
Hilas::IRobot(name,"Wam7dof",-1)
{
	for (int i = 0; i < arm_count; ++i)
	{
		arm_a.push_back(Service::shared_ptr(new Wam7dofArmService(std::string("Arm")+boost::lexical_cast<std::string>(i+1),this)));
		grip_a.push_back(Service::shared_ptr(new Wam7dofGripperService(std::string("Gripper")+boost::lexical_cast<std::string>(i+1),this)));	    	
	}
}

Wam7dofOODL::~Wam7dofOODL() {}

bool Wam7dofOODL::configureHook()
{
	Hilas::IRobot::configureHook();
}

void Wam7dofOODL::updateHook()
{
    // Time measurements to confirm period
	Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince( timestamp );
	ca = ca + (elapsed - ca) / (++ca_counter);
    timestamp = RTT::os::TimeService::Instance()->getTicks(); //used cumulative average, because timestamps can overflow for long operation times.

	Hilas::IRobot::updateHook();
}

}
ORO_CREATE_COMPONENT(Wam7dof::Wam7dofOODL)