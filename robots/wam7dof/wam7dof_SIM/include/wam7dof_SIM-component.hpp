#pragma once

#include "Wam7dofArmService.hpp"
#include "Wam7dofGripperService.hpp"
#include <IRobot.hpp>

namespace Wam7dof
{

using namespace RTT;
using namespace RTT::types;
using namespace std;

class Wam7dofSIM: public Hilas::IRobot
{

public:

	Wam7dofSIM(const string& name);
	~Wam7dofSIM();

private:

	friend class Wam7dofArmService;
	friend class Wam7dofGripperService;
};
}