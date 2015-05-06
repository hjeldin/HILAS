#pragma once

#include <interfaces/IRobotCmdDemux.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <puma260.hpp>

namespace Puma260
{

using namespace RTT;
using namespace std;

class Puma260CmdDemux: public Hilas::IRobotCmdDemux
{

public:
	Puma260CmdDemux(std::string const& name);
 	~Puma260CmdDemux();
 	void updateHook();
};

}