#pragma once

#include "YouBotBaseService.hpp"
#include "YouBotArmService.hpp"
#include "YouBotGripperService.hpp"
#include <IRobot.hpp>

#include <youbot/ProtocolDefinitions.hpp>
#include <youbot/EthercatMaster.hpp>
#include <youbot/EthercatMasterWithoutThread.hpp>

namespace YouBot
{

using namespace RTT;
using namespace RTT::types;	
using namespace std;

class YouBotOODL: public Hilas::IRobot
{

public:

	YouBotOODL(const string& name);
	~YouBotOODL();

protected:

	bool configureHook();
	void updateHook();

private:

	youbot::EthercatMasterInterface* m_ec_master;
	unsigned int m_communication_errors;
	unsigned int m_max_communication_errors;

	RTT::os::TimeService::ticks timestamp;
	RTT::Seconds ca;
	unsigned int ca_counter;

	friend class YouBotArmService;
	friend class YouBotBaseService;
	friend class YouBotGripperService;
};

}
