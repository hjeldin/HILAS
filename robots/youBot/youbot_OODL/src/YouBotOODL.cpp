#include "YouBotOODL.hpp"

namespace YouBot
{

YouBotOODL::YouBotOODL(const string& name):
Hilas::IRobot(name,"youBot",-1)
{
	try
	{
		//arm_a.push_back(Service::shared_ptr(new YouBotArmService(std::string("Arm1"),this)));
		
		//grip_a.push_back(Service::shared_ptr(new YouBotGripperService(std::string("Gripper1"),this)));
		//base_a.push_back(Service::shared_ptr(new YouBotBaseService(std::string("Base1"),this)));
		for (int i = 0; i < arm_count; ++i)
		{
			arm_a.push_back(Service::shared_ptr(new YouBotArmService(std::string("Arm")+boost::lexical_cast<std::string>(i+1),this)));
			grip_a.push_back(Service::shared_ptr(new YouBotGripperService(std::string("Gripper")+boost::lexical_cast<std::string>(i+1),this)));	    	
		}

		for (int i = 0; i < base_count; ++i)
		{
			base_a.push_back(Service::shared_ptr(new YouBotBaseService(std::string("Base")+boost::lexical_cast<std::string>(i+1),this)));
		}
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

	youbot::Logger::logginLevel = youbot::fatal;
}

YouBotOODL::~YouBotOODL() {}

bool YouBotOODL::configureHook()
{
	unsigned int nr_slaves = 0;

	try
	{
		m_ec_master = &EthercatMaster::getInstance("/youbot-ethercat.cfg", OODL_YOUBOT_CONFIG_DIR, false);

		if(m_ec_master == NULL || (m_ec_master != NULL && !m_ec_master->isEtherCATConnectionEstablished()))
		{
			log(Error) << "No EtherCat connection." << endlog();
			this->error();
			return false;
		}

		if(m_ec_master->isThreadActive())
		{
			log(Error) << "EtherCat thread detected, programming error." << endlog();
			this->error();
			return false;
		}

		nr_slaves = m_ec_master->getNumberOfSlaves();

		if(nr_slaves != 4 && nr_slaves != 9 && nr_slaves != 14)
		{
			log(Error) << "Not a proper amount of Ethercat slaves, got:" << nr_slaves << endlog();
			this->error();
			return false;
		}
	}
	catch (std::exception& e)
	{
		log(Error) << e.what() << endlog();
		this->exception();
		return false;
	}

	Hilas::IRobot::configureHook();
}

void YouBotOODL::updateHook()
{
    // Time measurements to confirm period
	Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince( timestamp );
	ca = ca + (elapsed - ca) / (++ca_counter);
    timestamp = RTT::os::TimeService::Instance()->getTicks(); //used cumulative average, because timestamps can overflow for long operation times.

    if(!m_ec_master->receiveProcessData())
    {
    	++m_communication_errors;
    	if(m_communication_errors > m_max_communication_errors)
    	{
    		log(Error) << "Lost EtherCAT connection";
    		this->error();
    	}
    }
    else
    {
    	m_communication_errors = 0;
    }

    if(m_ec_master->isErrorInSoemDriver())
    {
    	log(Error) << "Error in the SOEM driver." << endlog();
    	this->error();
    }

	Hilas::IRobot::updateHook();

    if(!m_ec_master->sendProcessData())
    {
    	++m_communication_errors;
    	if(m_communication_errors > m_max_communication_errors)
    	{
    		log(Error) << "Lost EtherCAT connection";
    		this->error();
    	}
    }
    else
    {
    	m_communication_errors = 0;
    }
}

}
ORO_CREATE_COMPONENT(YouBot::YouBotOODL)