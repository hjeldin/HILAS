#pragma once

#include <interfaces/IRobotStateRepublisher.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <youBot.hpp>

namespace YouBot
{
 	using namespace RTT;
 	using namespace std;

class YouBotStateRepublisher: public Hilas::IRobotStateRepublisher
{

public:
	YouBotStateRepublisher(std::string const& name);
 	~YouBotStateRepublisher();
 	void updateHook();
};

}