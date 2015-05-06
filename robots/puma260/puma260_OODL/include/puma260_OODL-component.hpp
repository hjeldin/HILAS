#pragma once

#include "Puma260ArmService.hpp"
#include "Puma260GripperService.hpp"
#include <IRobot.hpp>

namespace Puma260
{

using namespace RTT;
using namespace RTT::types;	
using namespace std;

class Puma260OODL: public Hilas::IRobot
{

public:

	Puma260OODL(const string& name);
	~Puma260OODL();

protected:

	bool configureHook();
	void updateHook();

private:

	RTT::os::TimeService::ticks timestamp;
	RTT::Seconds ca;
	unsigned int ca_counter;

	friend class Puma260ArmService;
	friend class Puma260GripperService;
};

}
