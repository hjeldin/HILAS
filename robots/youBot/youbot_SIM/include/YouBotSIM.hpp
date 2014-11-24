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

extern "C"
{
	#define MAX_EXT_API_CONNECTION 255
	#include "extApi.h"
}

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	class YouBotSIM: public TaskContext
	{
		public:
			YouBotSIM(const string& name);
			virtual ~YouBotSIM();

		    void setSimMode(int mode);			

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

		private:
	      	vector<OperationCaller<void(int)> > sim_mode_ops;
	      	vector<OperationCaller<bool(void)> > calibrate_ops;
	      	vector<OperationCaller<bool(void)> > start_ops;
	      	vector<OperationCaller<void(void)> > update_ops;
	      	vector<OperationCaller<void(void)> > stop_ops;
	      	vector<OperationCaller<void(void)> > cleanup_ops;

	      	RTT::os::TimeService::ticks timestamp;
	      	RTT::Seconds ca;
	      	unsigned int ca_counter;
	      	long c_id;

	      	friend class YouBotArmService;
	      	friend class YouBotBaseService;
	      	friend class YouBotGripperService;
	};

	class VREPComm
	{
	public:
		static VREPComm& getInstance()
        {
            static VREPComm instance; 		// Guaranteed to be destroyed. Instantiated on first use.
            return instance;
        }

        static std::string server_ip;
        static int server_port;

       	int clientID;

        ~VREPComm(){}
	protected:

	private:
		bool connected;
		VREPComm():
			connected(false),
			clientID(-1)
		{
			clientID = simxStart(SIM_ADDRESS,SIM_PORT,10,0,1000,5);
			if(clientID == -1){
				log(Error) << "[ERROR] connection to v-rep refused"<< endlog();
			} else {
				connected = true;
				log(Info) << "[OK] Connected to v-rep with clientID: " << clientID << endlog();
			}
			simxStartSimulation(clientID,simx_opmode_oneshot);
		}

		VREPComm(VREPComm const&) = delete; 			//< Don't Implement
        void operator=(VREPComm const&); 				//< Don't implement
	};

}
