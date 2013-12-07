#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>

#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include "YouBotTypes.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	class YouBotVREP: public TaskContext
	{
		public:
			YouBotVREP(const string& name);
			virtual ~YouBotVREP();

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

		private:
      vector<OperationCaller<bool(void)> > calibrate_ops;
      vector<OperationCaller<bool(void)> > start_ops;
      vector<OperationCaller<void(void)> > update_ops;
      vector<OperationCaller<void(void)> > stop_ops;
      vector<OperationCaller<void(void)> > cleanup_ops;

      RTT::os::TimeService::ticks timestamp;
      RTT::Seconds ca;
      unsigned int ca_counter;

      friend class YouBotArmService;
      friend class YouBotBaseService;
      friend class YouBotGripperService;
	};

}
