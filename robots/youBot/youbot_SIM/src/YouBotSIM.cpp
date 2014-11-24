#include "YouBotSIM.hpp"
#include "YouBotBaseService.hpp"
#include "YouBotArmService.hpp"
#include "YouBotHelpers.hpp"
#include "YouBotGripperService.hpp"

#include <ocl/Component.hpp>

#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <vector>

#include <generic/Logger.hpp>
#include <rtt/Logger.hpp>

namespace YouBot
{
	using namespace RTT;
	using namespace RTT::types;
	using namespace std;

//	unsigned int non_errors = ::MOTOR_HALTED | ::PWM_MODE_ACTIVE | ::VELOCITY_MODE | ::POSITION_MODE | ::TORQUE_MODE | ::POSITION_REACHED | ::INITIALIZED;

	YouBotSIM::YouBotSIM(const string& name) :
	TaskContext(name, PreOperational)
	{
		youbot::Logger::logginLevel = youbot::fatal;
		RTT::Logger* ins = RTT::Logger::Instance();
		ins->setLogLevel(RTT::Logger::Info);
	 	
	 	this->addOperation("setSimMode", &YouBotSIM::setSimMode,this, OwnThread).doc("Set simulation mode.");
	}

	YouBotSIM::~YouBotSIM() {}

	void YouBotSIM::setSimMode(int mode)
	{
		simxInt handle;
		simxGetObjectHandle(c_id,"youBot",&handle, simx_opmode_oneshot_wait);

		switch(mode)
		{
			case 1:
				simxSetModelProperty(c_id,handle,sim_modelproperty_not_dynamic,simx_opmode_oneshot);
				for(unsigned int i=0; i<sim_mode_ops.size(); ++i)
	    		{
					sim_mode_ops[i](1);
				}
			break;
			
			case 2:
				simxInt prop;
				simxGetModelProperty(c_id,handle,&prop, simx_opmode_oneshot);
				simxSetModelProperty(c_id,handle,(prop & 0xFFDF), simx_opmode_oneshot);			
				for(unsigned int i=0; i<sim_mode_ops.size(); ++i)
	    		{
					sim_mode_ops[i](2);
				}
			break;
			
			default: break;
		}
	}

	bool YouBotSIM::configureHook()
	{
		// MUST BE THE FIRST ONE TO CALL getInstance!!!
		unsigned int nr_slaves = 9;

		//@todo getNumSlaves from VREP (manage multi-robot scenario)

		// Base
		c_id = VREPComm::getInstance().clientID;
		
		this->provides()->addService(Service::shared_ptr(new YouBotBaseService("Base",this, 1, c_id)));
		update_ops.push_back(this->provides("Base")->getOperation("update"));
		calibrate_ops.push_back(this->provides("Base")->getOperation("calibrate"));
		start_ops.push_back(this->provides("Base")->getOperation("start"));
		stop_ops.push_back(this->provides("Base")->getOperation("stop"));
		cleanup_ops.push_back(this->provides("Base")->getOperation("cleanup"));
		sim_mode_ops.push_back(this->provides("Base")->getOperation("setsim_mode"));
		log(Info) << "[VREP] Detected youbot base, loading Base service" << endlog();

		if(nr_slaves == NR_OF_BASE_SLAVES + NR_OF_ARM_SLAVES) // Arm 2
		{
			// Arm 1
			this->provides()->addService(Service::shared_ptr(new YouBotArmService("Arm1",this, 1, c_id)));
			update_ops.push_back(this->provides("Arm1")->getOperation("update"));
			calibrate_ops.push_back(this->provides("Arm1")->getOperation("calibrate"));
			start_ops.push_back(this->provides("Arm1")->getOperation("start"));
			stop_ops.push_back(this->provides("Arm1")->getOperation("stop"));
			cleanup_ops.push_back(this->provides("Arm1")->getOperation("cleanup"));
			sim_mode_ops.push_back(this->provides("Arm1")->getOperation("setsim_mode"));			
			log(Info) << "[VREP] Detected youbot arm, loading Arm1 service" << endlog();

			// Gripper 1
			this->provides()->addService(Service::shared_ptr(new YouBotGripperService("Gripper1",this, c_id)));
			update_ops.push_back(this->provides("Gripper1")->getOperation("update"));
			calibrate_ops.push_back(this->provides("Gripper1")->getOperation("calibrate"));
			start_ops.push_back(this->provides("Gripper1")->getOperation("start"));
			stop_ops.push_back(this->provides("Gripper1")->getOperation("stop"));
			cleanup_ops.push_back(this->provides("Gripper1")->getOperation("cleanup"));
			sim_mode_ops.push_back(this->provides("Gripper1")->getOperation("setsim_mode"));
			log(Info) << "[VREP] Detected youbot gripper, loading Gripper1 service" << endlog();
		}

		if(nr_slaves == NR_OF_BASE_SLAVES + 2*NR_OF_ARM_SLAVES) // Arm 2
		{
			this->provides()->addService(Service::shared_ptr(new YouBotArmService("Arm2",this, 1 + NR_OF_ARM_SLAVES, c_id)));
			update_ops.push_back(this->provides("Arm2")->getOperation("update"));
			calibrate_ops.push_back(this->provides("Arm2")->getOperation("calibrate"));
			start_ops.push_back(this->provides("Arm2")->getOperation("start"));
			stop_ops.push_back(this->provides("Arm2")->getOperation("stop"));
			cleanup_ops.push_back(this->provides("Arm2")->getOperation("cleanup"));
			log(Info) << "[VREP] Detected youbot arm, loading Arm2 service" << endlog();
		}

		Seconds period = this->getPeriod();
		log(Info) << "[VREP] thread period: " << period << endlog();

		// invoke all calibration operations
		bool proper_calibration(true);
		for(unsigned int i=0; i<calibrate_ops.size(); ++i)
		{
			if(calibrate_ops[i].ready())
			{
				if(!calibrate_ops[i]())
				{
					proper_calibration = false;
					log(Warning) << "[VREP] Calibration failed on one Service." << endlog();
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
			sim_mode_ops.clear();
			this->error();
			return false;
		}
		else
		{
			return TaskContext::configureHook();
		}
	}

	bool YouBotSIM::startHook()
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

	void YouBotSIM::updateHook()
	{
        // Time measurements to confirm period
		Seconds elapsed = RTT::os::TimeService::Instance()->secondsSince( timestamp );
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

	void YouBotSIM::stopHook()
	{
		for(unsigned int i=0;i<stop_ops.size();++i)
		{
			if(stop_ops[i].ready())
			{
				stop_ops[i]();
			}
		}

		log(Info) << "[VREP] YouBotSIM cumulative average updateHook period: " << ca << endlog();

		TaskContext::stopHook();
	}

	void YouBotSIM::cleanupHook()
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

ORO_CREATE_COMPONENT( YouBot::YouBotSIM )
