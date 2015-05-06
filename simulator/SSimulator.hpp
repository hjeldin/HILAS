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
	static std::string scene;
   	int clientID;

    ~SIMCom(){}

private:

	bool connected;
	SIMCom(): connected(false), clientID(-1)
	{

		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/hilas.ini", pt);		

		clientID = simxStart((pt.get<std::string>("simulator.socketaddr")).c_str(),pt.get<int>("simulator.socketport"),10,0,1000,5);
		std::string client_scene = pt.get<std::string>("simulator.clientscene");
		std::string server_scene = pt.get<std::string>("simulator.serverscene");
		
		if(clientID == -1)
		{
			log(Error) << "[ERROR] connection to v-rep refused"<< endlog();
		}
		else
		{
			connected = true;
			log(Info) << "[OK] Connected to v-rep with clientID: " << clientID << endlog();
		}

		if(client_scene.empty() && server_scene.empty()){
			log(Info) << "No scene specified to load. Will only connect to simulator." << endlog();
		} else {
			if(!client_scene.empty() && !server_scene.empty())
			{
				log(Info) << "Both client and server scenes specified. Fallback on client scene." << endlog();
			}
			//specifies scene location. true on client, false on server. 
			bool isSceneOnClient = false;
			if(client_scene.empty()){
				scene = server_scene;
			} else { 
				isSceneOnClient = true;
				scene = client_scene;
			}
			simxLoadScene(clientID,scene.c_str(),(simxUChar)isSceneOnClient, simx_opmode_oneshot_wait);
		}

		simxStartSimulation(clientID,simx_opmode_oneshot);
	}

	SIMCom(SIMCom const&); // Don't Implement
    void operator = (SIMCom const&); // Don't implement
};

}
