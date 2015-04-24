#pragma once

#include <interfaces/IRobotStateRepublisher.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <wam7dof.hpp>

namespace Wam7dof
{

using namespace RTT;
using namespace std;

class Wam7dofStateRepublisher : public Hilas::IRobotStateRepublisher
{

public:
	
	Wam7dofStateRepublisher(std::string const& name);
	~Wam7dofStateRepublisher();	
	void updateHook();
};
}