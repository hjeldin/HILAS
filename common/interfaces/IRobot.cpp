#include "IRobot.hpp"

namespace Hilas
{

	std::string SIMCom::scene = "";

IRobot::IRobot(const string& name, std::string robot_name, long sim_client_id = -1):
TaskContext(name, PreOperational), sim_client_id(sim_client_id), robot_name(robot_name)
{
	try{
		RTT::Logger* ins = RTT::Logger::Instance();
		ins->setLogLevel(RTT::Logger::Info);
	    boost::property_tree::ptree pt;
	   	std::transform(robot_name.begin(), robot_name.end(), robot_name.begin(), ::tolower);
	    boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/"+robot_name+".ini", pt);
	    arm_count = pt.get<int>("robot.armCount");
	    base_count = pt.get<int>("robot.baseCount");
		this->addOperation("setSimMode", &IRobot::setsim_mode,this).doc("Set simulation mode.");    
	} catch(std::exception &e)
	{
		log(Error) << e.what() << endlog();
	}
}

IRobot::~IRobot(){}

void IRobot::setsim_mode(int mode)
{
	simxInt handle;
	simxGetObjectHandle(sim_client_id, robot_name.c_str(), &handle, simx_opmode_oneshot_wait);

	switch(mode)
	{
		case 1:
			simxSetModelProperty(sim_client_id, handle, sim_modelproperty_not_dynamic, simx_opmode_oneshot);
			for(unsigned int i=0; i<sim_mode_ops.size(); ++i)
    		{
				sim_mode_ops[i](1);
			}

		break;
		
		case 2:
			simxInt prop;
			simxGetModelProperty(sim_client_id, handle,&prop, simx_opmode_oneshot);
			simxSetModelProperty(sim_client_id, handle,(prop & 0xFFDF), simx_opmode_oneshot);			
			for(unsigned int i=0; i<sim_mode_ops.size(); ++i)
    		{
				sim_mode_ops[i](2);
			}
			
		break;
		
		default: break;
	}
}

bool IRobot::configureHook()
{
    for (int i = 0; i < base_count; ++i)
    {
		this->provides()->addService(Service::shared_ptr(base_a[i]));
		update_ops.push_back(this->provides("Base" +boost::lexical_cast<std::string>(i+1))->getOperation("update"));
		calibrate_ops.push_back(this->provides("Base" +boost::lexical_cast<std::string>(i+1))->getOperation("calibrate"));
		start_ops.push_back(this->provides("Base" +boost::lexical_cast<std::string>(i+1))->getOperation("start"));
		stop_ops.push_back(this->provides("Base" +boost::lexical_cast<std::string>(i+1))->getOperation("stop"));
		cleanup_ops.push_back(this->provides("Base" +boost::lexical_cast<std::string>(i+1))->getOperation("cleanup"));
		sim_mode_ops.push_back(this->provides("Base" +boost::lexical_cast<std::string>(i+1))->getOperation("setSimMode"));
		log(Info) << "Detected robot base, loading Base service" << endlog();
    }

    for (int i = 0; i < arm_count; ++i)
    {
		this->provides()->addService(Service::shared_ptr(arm_a[i]));
		update_ops.push_back(this->provides("Arm" +boost::lexical_cast<std::string>(i+1))->getOperation("update"));
		calibrate_ops.push_back(this->provides("Arm" +boost::lexical_cast<std::string>(i+1))->getOperation("calibrate"));
		start_ops.push_back(this->provides("Arm" +boost::lexical_cast<std::string>(i+1))->getOperation("start"));
		stop_ops.push_back(this->provides("Arm" +boost::lexical_cast<std::string>(i+1))->getOperation("stop"));
		cleanup_ops.push_back(this->provides("Arm" +boost::lexical_cast<std::string>(i+1))->getOperation("cleanup"));
		sim_mode_ops.push_back(this->provides("Arm" +boost::lexical_cast<std::string>(i+1))->getOperation("setSimMode"));
		log(Info) << "Detected robot arm, loading Arm service" << endlog();

		this->provides()->addService(Service::shared_ptr(grip_a[i]));
		update_ops.push_back(this->provides("Gripper" +boost::lexical_cast<std::string>(i+1))->getOperation("update"));
		calibrate_ops.push_back(this->provides("Gripper" +boost::lexical_cast<std::string>(i+1))->getOperation("calibrate"));
		start_ops.push_back(this->provides("Gripper" +boost::lexical_cast<std::string>(i+1))->getOperation("start"));
		stop_ops.push_back(this->provides("Gripper" +boost::lexical_cast<std::string>(i+1))->getOperation("stop"));
		cleanup_ops.push_back(this->provides("Gripper" +boost::lexical_cast<std::string>(i+1))->getOperation("cleanup"));
		sim_mode_ops.push_back(this->provides("Gripper" +boost::lexical_cast<std::string>(i+1))->getOperation("setSimMode"));

		log(Info) << "Detected robot gripper, loading Gripper service" << endlog();
    }    

	// invoke all calibration operations
	bool proper_calibration(true);
	for(unsigned int i=0; i<calibrate_ops.size(); ++i)
	{
		if(calibrate_ops[i].ready())
		{
			if(!calibrate_ops[i]())
			{
				proper_calibration = false;
				log(Warning) << "Calibration failed on one Service." << endlog();
				break;
			}
		}
	}

	// Break off when a part of the robot couldn't calibrate.
	if(!proper_calibration)
	{
		cleanupHook();
		update_ops.clear();
		calibrate_ops.clear();
		start_ops.clear();
		stop_ops.clear();
		cleanup_ops.clear();
		this->error();

		return false;
	}
	else
	{
		return TaskContext::configureHook();
	}
}

bool IRobot::startHook()
{
	// invoke all starts
	bool fully_started(true);

	for(unsigned int i=0;i<start_ops.size();++i)
	{
		if(start_ops[i].ready())
		{
			if(!start_ops[i]())
			{
				fully_started = false;
				log(Warning) << "Start failed on one Service." << endlog();
				break;
			}
		}
	}

	ca = 0;
	ca_counter = 0;
	timestamp = RTT::os::TimeService::Instance()->getTicks();

	return fully_started ? TaskContext::startHook() : false;
}

void IRobot::updateHook()
{
    // Time measurements to confirm period
	Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince(timestamp);
	ca = ca + (elapsed - ca) / (++ca_counter);
    timestamp = RTT::os::TimeService::Instance()->getTicks(); //used cumulative average, because timestamps can overflow for long operation times.

	// The mailbox messages are send/received immediatly
    for(unsigned int i=0;i<update_ops.size();++i)
    {
    	if(update_ops[i].ready())
    	{
    		update_ops[i]();
    	}
    }

    TaskContext::updateHook();
}

void IRobot::stopHook()
{
	for(unsigned int i=0;i<stop_ops.size();++i)
	{
		if(stop_ops[i].ready())
		{
			stop_ops[i]();
		}
	}

	log(Info) << "Robot cumulative average updateHook period: " << ca << endlog();

	TaskContext::stopHook();
}

void IRobot::cleanupHook()
{
	for(unsigned int i=0;i<cleanup_ops.size();++i)
	{
		if(cleanup_ops[i].ready())
		{
			cleanup_ops[i]();
		}
	}

	TaskContext::cleanupHook();
}

}