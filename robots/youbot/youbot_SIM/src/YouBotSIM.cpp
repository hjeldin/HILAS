#include "YouBotSIM.hpp"

namespace YouBot
{

YouBotSIM::YouBotSIM(const string& name):
Hilas::IRobot(name,"youBot",-1)
{
	boost::property_tree::ptree pt;
	std::transform(robot_name.begin(), robot_name.end(), robot_name.begin(), ::tolower);
	boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/"+robot_name+".ini", pt);

	sim_client_id = Hilas::SIMCom::getInstance().clientID;

	for (int i = 0; i < pt.get<int>("robot.armCount"); ++i)
	{
		arm_a.push_back(Service::shared_ptr(new YouBotArmService(std::string("Arm")+boost::lexical_cast<std::string>(i+1),this,sim_client_id)));
		grip_a.push_back(Service::shared_ptr(new YouBotGripperService(std::string("Gripper")+boost::lexical_cast<std::string>(i+1),this,sim_client_id)));	    	
	}

	for (int i = 0; i < pt.get<int>("robot.baseCount"); ++i)
	{
		base_a.push_back(Service::shared_ptr(new YouBotBaseService(std::string("Base")+boost::lexical_cast<std::string>(i+1),this,sim_client_id)));
	}
}

YouBotSIM::~YouBotSIM(){}

}

ORO_CREATE_COMPONENT(YouBot::YouBotSIM)