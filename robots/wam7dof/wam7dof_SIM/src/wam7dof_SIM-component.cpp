#include "wam7dof_SIM-component.hpp"

namespace Wam7dof
{

Wam7dofSIM::Wam7dofSIM(const string& name):
Hilas::IRobot(name,"Wam7dof",-1)
{
	try{
		boost::property_tree::ptree pt;
		std::transform(robot_name.begin(), robot_name.end(), robot_name.begin(), ::tolower);
		boost::property_tree::ini_parser::read_ini(std::string(getenv("HILAS_HOME")) + "/hilas/config/"+robot_name+".ini", pt);

		sim_client_id = Hilas::SIMCom::getInstance().clientID;

		for (int i = 0; i < pt.get<int>("robot.armCount"); ++i)
		{
			arm_a.push_back(Service::shared_ptr(new Wam7dofArmService(std::string("Arm")+boost::lexical_cast<std::string>(i+1),this,sim_client_id)));
			grip_a.push_back(Service::shared_ptr(new Wam7dofGripperService(std::string("Gripper")+boost::lexical_cast<std::string>(i+1),this,sim_client_id)));	    	
		}
	} catch (std::exception & e)
	{
		log(Error) << e.what() << endlog();
	}
}

Wam7dofSIM::~Wam7dofSIM(){}

}

ORO_CREATE_COMPONENT(Wam7dof::Wam7dofSIM)