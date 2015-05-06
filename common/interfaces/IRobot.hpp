#pragma once

#include "IRobotBaseService.hpp"
#include "IRobotArmService.hpp"
#include "IRobotGripperService.hpp"

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <SSimulator.hpp>

namespace Hilas
{

using namespace RTT;
using namespace RTT::types;
using namespace std;

class IRobot: public RTT::TaskContext
{

public:
	IRobot(const string& name, std::string robot_name, long sim_client_id);
	~IRobot();
	void setsim_mode(int mode);

protected:

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

   	vector<OperationCaller<void(int)> > sim_mode_ops;
	vector<OperationCaller<bool(void)> > calibrate_ops;
	vector<OperationCaller<bool(void)> > start_ops;
	vector<OperationCaller<void(void)> > update_ops;
	vector<OperationCaller<void(void)> > stop_ops;
	vector<OperationCaller<void(void)> > cleanup_ops;

	vector<Service::shared_ptr> arm_a;
	vector<Service::shared_ptr> base_a;
	vector<Service::shared_ptr> grip_a;

	RTT::os::TimeService::ticks timestamp;
	RTT::Seconds ca;
	unsigned int ca_counter;
	unsigned int nr_slaves;
	std::string robot_name;
	long sim_client_id;

	int arm_count;
	int base_count;
};

}