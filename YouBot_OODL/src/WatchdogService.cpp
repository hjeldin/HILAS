#include "WatchdogService.hpp"

#include <cassert>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

	WatchdogService::WatchdogService(const string& name, TaskContext* parent) :
		Service(name,parent), m_timeout((double)0.5)
	{
		setupComponentInterface();
	}

	WatchdogService::~WatchdogService()
	{
	}

  void WatchdogService::setTimeout(double timeout)
  {
    m_timeout = ros::Duration(timeout);
  }

	void WatchdogService::setupComponentInterface()
	{
    this->addOperation("start",&WatchdogService::start,this);
    this->addOperation("update",&WatchdogService::update,this);
    this->addOperation("calibrate",&WatchdogService::calibrate,this);
    this->addOperation("stop",&WatchdogService::stop,this);
    this->addOperation("cleanup",&WatchdogService::cleanup,this);

    this->addOperation("setTimeout", &WatchdogService::setTimeout, this).doc("Set timeout time.");

    this->addPort("watchdog_msg",watchdog_msg).doc("Watchdog kicks");
	}

	bool WatchdogService::start()
	{
    if(!watchdog_msg.connected())
    {
      log(Error) << "WatchdogService NOT connected." << endlog();
      return false;
    }

    m_last = ros::Time::now();
    return true;
	}

	void WatchdogService::update()
	{
    if(watchdog_msg.read(m_watchdog_msg) == RTT::NewData)
    {
      m_last = ros::Time::now();
      return;
    }
    if((ros::Time::now() - m_last) > m_timeout)
    {
      log(Error) << "Watchdog timed out! Stopping TaskContext!" << endlog();
      // shutdown
      this->getOwner()->stop();
    }
	}

	bool WatchdogService::calibrate()
	{
		return true;
	}

	void WatchdogService::stop()
	{ }

	void WatchdogService::cleanup()
	{ }

}
