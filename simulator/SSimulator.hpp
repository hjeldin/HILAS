extern "C"
{
	#define MAX_EXT_API_CONNECTION 255
	#include "extApi.h"
}

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

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

		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/hilas.ini", pt);		

		clientID = simxStart((pt.get<std::string>("simulator.socketaddr")).c_str(),pt.get<int>("simulator.socketport"),10,0,1000,5);
		
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