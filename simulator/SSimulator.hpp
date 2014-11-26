extern "C"
{
	#define MAX_EXT_API_CONNECTION 255
	#include "extApi.h"
}

namespace Hilas
{

class SIMCom
{

public:

	static SIMCom& getInstance()
    {
        static SIMCom instance; 		// Guaranteed to be destroyed. Instantiated on first use.
        return instance;
    }

    static std::string server_ip;
    static int server_port;
   	int clientID;

    ~SIMCom(){}

private:

	bool connected;
	SIMCom(): connected(false), clientID(-1)
	{
		clientID = simxStart(SIM_ADDRESS,SIM_PORT,10,0,1000,5);
		
		if(clientID == -1)
		{
			log(Error) << "[ERROR] connection to v-rep refused"<< endlog();
		}
		else
		{
			connected = true;
			log(Info) << "[OK] Connected to v-rep with clientID: " << clientID << endlog();
		}

		simxStartSimulation(clientID,simx_opmode_oneshot);
	}

	SIMCom(SIMCom const&); // Don't Implement
    void operator = (SIMCom const&); // Don't implement
};

}