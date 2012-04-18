#include "YouBot_configurator.hpp"

#include <stdlib.h>
#include <stdio.h>

//#include <cmath>
//#include <limits>
//#include <iostream>
//#include <vector>
//#include <signal.h>
//#include "youbot/YouBotJointParameter.hpp"
//#include "youbot/YouBotGripper.hpp"
//#include "boost/date_time/posix_time/posix_time.hpp"
//#include <vector>
//#include <sstream>
//#include <boost/limits.hpp>
//#include "generic/Logger.hpp"
//#include "generic/Units.hpp"
//#include "generic/Time.hpp"
//#include "generic/Exceptions.hpp"
//#include "generic-joint/JointParameter.hpp"
//#include "youbot/YouBotJointParameterReadOnly.hpp"

#include <YouBotTypes.hpp>
#include <JointConfigurator.hpp>

#include <ros/package.h>

using namespace RTT;
using namespace std;
using namespace youbot;

namespace YouBot
{

YouBot_configurator::YouBot_configurator(const string& name) :
	TaskContext(name, PreOperational), m_manipulator(NULL), m_base(NULL),current_state(NOT_RUNNABLE),joint(NULL)
{
	this->addOperation("setDefinedConfiguration",&YouBot_configurator::setDefinedConfiguration,this).doc("The function will set parameters from config files to youBot firmaware. It cann't be called if youbott drivers are working");
	this->addOperation("pidTunning",&YouBot_configurator::pidTunning,this).doc("");
}

YouBot_configurator::~YouBot_configurator()
{
}
bool YouBot_configurator::configureHook(){
	// MUST BE THE FIRST ONE TO CALL getInstance!!!
	unsigned int nr_slaves = 0;

	try
	{
		EthercatMaster* ec_master = &(EthercatMaster::getInstance("/youbot-ethercat.cfg", OODL_YOUBOT_CONFIG_DIR));

		nr_slaves = ec_master->getNumberOfSlaves();
	}
	catch (std::exception& e)
	{
		log(Error) << e.what() << endlog();
		this->error();
		return false;
	}

	if(nr_slaves != (NR_OF_BASE_SLAVES + NR_OF_ARM_SLAVES) && nr_slaves != (NR_OF_BASE_SLAVES + 2*NR_OF_ARM_SLAVES) &&
			nr_slaves !=(NR_OF_BASE_SLAVES))
	{
		log(Error) << "Not a proper amount of Ethercat slaves, got:" << nr_slaves << endlog();
		return false;
	}

	try
	{
		m_manipulator = new YouBotManipulator("/youbot-manipulator", OODL_YOUBOT_CONFIG_DIR);
		if(m_manipulator == NULL)
		{
			log(Error) << "Could not create the YouBotManipulator." << endlog();
			//return false;
		}

		m_base = new YouBotBase("/youbot-base", OODL_YOUBOT_CONFIG_DIR);
		if(m_base == NULL)
		{
			log(Error) << "Could not create the YouBotBase." << endlog();
			//return false;
		}
	}
	catch (std::exception& e)
	{
		log(Error) << e.what();
		m_manipulator = NULL;
		this->error();
		return false;
	}
	return TaskContext::configureHook();

}
double YouBot_configurator::pidTunning(double signalLevel, int slaveNumber)
{
	if (current_state==NOT_RUNNABLE)
	{
		step=signalLevel*si::ampere;
		current_state=TEST_SIGNAL_LOW;

		PParameterSecondParametersCurrentControl Pparam2;
		joint->getConfigurationParameter(Pparam2);
		IParameterSecondParametersCurrentControl Iparam2;
		joint->getConfigurationParameter(Iparam2);
		DParameterSecondParametersCurrentControl Dparam2;
		joint->getConfigurationParameter(Dparam2);
		IClippingParameterSecondParametersCurrentControl Iclip2;
		joint->getConfigurationParameter(Iclip2);
		// DO NOT FORGET TO (RE)STORE THE PREVIOUS SETTINGS!!!
		Iclip2.getParameter(oldPID[0]);
		Pparam2.getParameter(oldPID[1]);
		Iparam2.getParameter(oldPID[2]);
		Dparam2.getParameter(oldPID[3]);
		log(Info)<< "Second current control (high current) Old:" << endlog();
		log(Info) << "Pparam: " << oldPID[0] << endlog();
		log(Info) << "Iparam: " << oldPID[1] << endlog();
		log(Info) << "Dparam: " << oldPID[2] << endlog();
		log(Info) << "Iclip: " << oldPID[3] << endlog();
	}

	return false;
}

void  YouBot_configurator::changePIDParameters(int P,int I, int D){

}
double YouBot_configurator::runTest(int P,int I, int D){

}

double YouBot_configurator::stepResponseWithError()
{
	JointCurrentSetpoint setPointCurrent;
	setPointCurrent.current = 0 * si::ampere;
	joint->setData(setPointCurrent);

	double int_err = 0.0;
	double sensed_current = 0;
	double tu = 0.0;
	double t = 0.0;

	for (int i = 0; i < 500; i++) {
		statusRecoder->recordStatus((double) setPointCurrent.current.value());
		SLEEP_MILLISEC(1);
	}
	setPointCurrent.current = step;
	joint->setData(setPointCurrent);
	for (int i = 0; i < 2000; i++) {
		statusRecoder->recordStatus((double) setPointCurrent.current.value());
		sensed_current = statusRecoder->getLastMeasurement().current;
		if(tu == 0.0 && sensed_current >= step.value()) //rise time
		{
			tu = t;
		}
		else
		{
			t += 0.001;
		}
		int_err = int_err + (w1*pow(sensed_current - step.value(), 2) + w2*pow(sensed_current,2)) * 0.001; // Euler integration
		statusRecoder->getLastMeasurement().error = int_err + w3 * tu;
		SLEEP_MILLISEC(1);
	}

	setPointCurrent.current = 0 * si::ampere;
	joint->setData(setPointCurrent);
	SLEEP_MILLISEC(10);

	return int_err + w3*tu;
}

bool YouBot_configurator::setDefinedConfiguration()
{


	string path = ros::package::getPath("YouBot_configurator");
	stringstream arm_path;
	arm_path << path << "/config/arm";
	stringstream base_path;
	base_path << path << "/config/base";

	for(unsigned int i = 1; i <= NR_OF_ARM_SLAVES; ++i)
	{
		stringstream name;
		name << "/arm-" << i << "-parameter.cfg";

		stringstream protected_name;
		protected_name << "/protected-arm-" << i << "-parameter.cfg";

		try
		{
			JointConfigurator jc(&(m_manipulator->getArmJoint(i)), arm_path.str(), name.str(), protected_name.str());
			jc.readParameters();
			jc.setParametersToJoint();
		}
		catch (std::exception& e)
		{
			log(Error) << e.what() << endlog();
		}
	}

	for(unsigned int i = 1; i <= NR_OF_BASE_SLAVES; ++i)
	{
		stringstream name;
		name << "/base-" << i << "-parameter.cfg";

		stringstream protected_name;
		protected_name << "/protected-base-" << i << "-parameter.cfg";

		try
		{
			JointConfigurator jc(&(m_base->getBaseJoint(i)), base_path.str(), name.str(), protected_name.str());
			jc.readParameters();
			jc.setParametersToJoint();
		}
		catch (std::exception& e)
		{
			log(Error) << e.what() << endlog();
		}
	}

	return true;
}

bool YouBot_configurator::startHook()
{
	if(current_state==NOT_RUNNABLE){
		log(Error) << "The configurator component is normally runnable. Use pid tunning function" << endlog();
		return false;
	}

	return true;
}

void YouBot_configurator::updateHook()
{
	switch (current_state)
	{
	case TEST_SIGNAL_HIGH:
	{
		JointCurrentSetpoint setPointCurrent;
		setPointCurrent.current = step;
		joint->setData(setPointCurrent);
		statusRecoder->recordStatus((double) setPointCurrent.current.value());
		double sensed_current = statusRecoder->getLastMeasurement().current;
		if(tu == 0.0 && sensed_current >= step.value()) //rise time
		{
				tu = t;
		}
		else
		{
				t += this->getPeriod();
		}
		int_err += (w1*pow(sensed_current - step.value(), 2) + w2*pow(sensed_current,2)) * this->getPeriod(); // Euler integration
		statusRecoder->getLastMeasurement().error = int_err + w3 * tu;
		if (step_counter>2000)
		{
			current_state=PARAMETER_CHANGE;
			JointCurrentSetpoint setPointCurrent;
			setPointCurrent.current = 0 * si::ampere;
			joint->setData(setPointCurrent);
			int_err += w3*tu;
		}
		break;
	}
	case TEST_SIGNAL_LOW:
	{
		JointCurrentSetpoint setPointCurrent;
		setPointCurrent.current = 0 * si::ampere;
		joint->setData(setPointCurrent);
		statusRecoder->recordStatus((double) setPointCurrent.current.value());
		step_counter++;
		if (step_counter>500)
			current_state=TEST_SIGNAL_HIGH;
		break;
	}
	case PARAMETER_CHANGE:
	{
		break;
	}
	case NOT_RUNNABLE:
	{
		this->stop();
		break;
	}
	default:
	{
		log(Error) << "Unusual state making exception" << endlog();
		this->error();
		return;
	}
}
}


}

ORO_CREATE_COMPONENT( YouBot::YouBot_configurator)
