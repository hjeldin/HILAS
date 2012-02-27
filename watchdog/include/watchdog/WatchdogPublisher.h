#pragma once

#include "ros/ros.h"
#include "watchdog/watchdogMsg.h"

/**
 * Watchdog keep-a-life-generator.
 */
class WatchdogPublisher {
	private: 
		/**
		 * The ROS Node Handle
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Watchdog Reset Publisher.
		 */
		ros::Publisher _watchdogPublisher;
	
		/**
		 * The name of the watchdog.
		 */
		std::string _name;
		
    /**
     * The actual timer watching the wdts
     */
    ros::Timer _wdtTimer;

		/**
		 * A boolean variable indicating if the wdt is enabled.
		 */
		bool _started;

	public:
		/**
		 * @param watchDogName the name of the watchdog
		 * @param period the period between keep-alive messages.
		 */
		WatchdogPublisher(std::string watchDogName, ros::Duration period);

		/**
		 * @brief Destructor.
		 */
		~WatchdogPublisher();

		/**
		 * @brief Send keep-a-life message.
		 */
		void kick(const ros::TimerEvent&);
};

