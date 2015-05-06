#include "puma260_SIM-component.hpp"

namespace Puma260
{

Puma260SIM::Puma260SIM(const string& name):
Hilas::IRobot(name,"puma260",-1)
{
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/"+robot_name+".ini", pt);

	sim_client_id = Hilas::SIMCom::getInstance().clientID;

	for (int i = 0; i < pt.get<int>("robot.armCount"); ++i)
	{
		arm_a.push_back(Service::shared_ptr(new Puma260ArmService(std::string("Arm")+boost::lexical_cast<std::string>(i+1),this,sim_client_id)));
		grip_a.push_back(Service::shared_ptr(new Puma260GripperService(std::string("Gripper")+boost::lexical_cast<std::string>(i+1),this,sim_client_id)));	    	
	}
}

Puma260SIM::~Puma260SIM(){}

}

ORO_CREATE_COMPONENT(Puma260::Puma260SIM)