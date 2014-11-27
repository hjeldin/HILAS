#pragma once

#include "Puma260ArmService.hpp"
#include "Puma260GripperService.hpp"
#include <IRobot.hpp>

namespace Puma260
{

using namespace RTT;
using namespace RTT::types;
using namespace std;

class Puma260SIM: public Hilas::IRobot
{

public:

	Puma260SIM(const string& name);
	~Puma260SIM();

private:

	friend class Puma260ArmService;
	friend class Puma260GripperService;
};
}