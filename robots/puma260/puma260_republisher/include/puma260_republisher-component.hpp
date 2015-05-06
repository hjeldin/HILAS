#pragma once

#include <interfaces/IRobotStateRepublisher.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

#include <puma260.hpp>

namespace Puma260
{

using namespace RTT;
using namespace std;

class Puma260StateRepublisher : public Hilas::IRobotStateRepublisher
{

public:
	
	Puma260StateRepublisher(std::string const& name);
	~Puma260StateRepublisher();	
	void updateHook();
};
}