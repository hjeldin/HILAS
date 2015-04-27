#pragma once

#include "YouBotBaseService.hpp"
#include "YouBotArmService.hpp"
#include "YouBotGripperService.hpp"
#include <IRobot.hpp>

namespace YouBot
{

using namespace RTT;
using namespace RTT::types;
using namespace std;

class YouBotSIM: public Hilas::IRobot
{

public:

	YouBotSIM(const string& name);
	~YouBotSIM();

private:

	friend class YouBotArmService;
	friend class YouBotBaseService;
	friend class YouBotGripperService;
};
}