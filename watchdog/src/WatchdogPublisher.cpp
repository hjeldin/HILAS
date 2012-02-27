#include "watchdog/WatchdogPublisher.h"

WatchdogPublisher::WatchdogPublisher(std::string wdtName, ros::Duration period) {
	_name = ros::this_node::getName() + "/" + wdtName;
	_started = false;
	
	_watchdogPublisher = _nh.advertise<watchdog::watchdogMsg>("/watchdog/keepalive", 100);
	
	_wdtTimer = _nh.createTimer(ros::Duration(period),&WatchdogPublisher::kick,this);
	ROS_INFO("WatchdogPublisher %s started.", _name.c_str());
}

WatchdogPublisher::~WatchdogPublisher() {
  ROS_INFO("WatchdogPublisher %s stopped.", _name.c_str());
}

void WatchdogPublisher::kick(const ros::TimerEvent&)
{
  watchdog::watchdogMsg msg;
  msg.kick = ros::Time::now();
  _watchdogPublisher.publish(msg);
}

static double default_period = 0.05; // 20 Hz

int main(int argc, char **argv) {
  ros::init(argc,argv,"WatchdogPublisher");

  double period(default_period);
  if(argc == 2)
  {
    period = strtod(argv[1], NULL);
    if(period <= 0.0)
      period = default_period;
  }

  WatchdogPublisher wdp("WatchdogPublisher", ros::Duration(period));
  ros::spin();
}
