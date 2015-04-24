#pragma once

#include <interfaces/IRobotCmdDemux.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <wam7dof.hpp>

namespace Wam7dof
{

using namespace RTT;
using namespace std;

class Wam7dofCmdDemux: public Hilas::IRobotCmdDemux
{

public:
	Wam7dofCmdDemux(std::string const& name);
 	~Wam7dofCmdDemux();
 	void updateHook();
};

}