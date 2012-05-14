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
#include <rtt/Time.hpp>

#include <youbot/EthercatMasterWithoutThread.hpp>

#include "YouBotTypes.hpp"

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	class YouBotOODL: public TaskContext
	{
		public:
			YouBotOODL(const string& name);
			virtual ~YouBotOODL();

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

			bool noWatchdog();

		private:
      vector<OperationCaller<bool(void)> > calibrate_ops;
      vector<OperationCaller<bool(void)> > start_ops;
      vector<OperationCaller<void(void)> > update_ops;
      vector<OperationCaller<void(void)> > stop_ops;
      vector<OperationCaller<void(void)> > cleanup_ops;

      youbot::EthercatMasterInterface* m_ec_master;
      unsigned int m_communication_errors;
      unsigned int m_max_communication_errors;

      bool m_use_watchdog;

      friend class YouBotArmService;
      friend class YouBotBaseService;
      friend class YouBotGripperService;
	};

}
