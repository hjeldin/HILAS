#pragma once

#include "Wam7dofArmService.hpp"
#include "Wam7dofGripperService.hpp"
#include <IRobot.hpp>

namespace Wam7dof
{

using namespace RTT;
using namespace RTT::types;	
using namespace std;

class Wam7dofOODL: public Hilas::IRobot
{

public:

	Wam7dofOODL(const string& name);
	~Wam7dofOODL();

protected:

	bool configureHook();
	void updateHook();

private:

	RTT::os::TimeService::ticks timestamp;
	RTT::Seconds ca;
	unsigned int ca_counter;

	friend class Wam7dofArmService;
	friend class Wam7dofGripperService;
};

}
