#pragma once

#include <interfaces/IRobotCmdDemux.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <youBot.hpp>

namespace YouBot
{
 	using namespace RTT;
 	using namespace std;

class YouBotCmdDemux: public Hilas::IRobotCmdDemux
{

public:
	YouBotCmdDemux(std::string const& name);
 	~YouBotCmdDemux();
 	void updateHook();
};

}