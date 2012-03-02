#pragma once

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>

#include "YouBotOODL.hpp"

#include <watchdog/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;
	using namespace std;
	using namespace youbot;

	/**
	 * @brief youBot watchdog Service, listens to the WatchdogPublisher.
	 * When the publisher and Service are disconnected, the TaskContext will be stopped.
	 */
  class WatchdogService : public Service {

		public:
      WatchdogService(const string& name, TaskContext* parent);
			virtual ~WatchdogService();

			void setTimeout(double timeout);

		private:
			InputPort<watchdog::watchdogMsg> watchdog_msg;

			void setupComponentInterface();

			bool calibrate();
			bool start();
			void update();
			void cleanup();
			void stop();

			watchdog::watchdogMsg m_watchdog_msg;
			ros::Time m_last;
      ros::Duration m_timeout;
  };

}
