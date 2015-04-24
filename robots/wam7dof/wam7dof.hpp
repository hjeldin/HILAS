#pragma once

#include <map>
#include <string>

namespace Wam7dof
{
	static std::map<std::string,int> create_map()
	{
		std::map<std::string,int> m;

		m["joint_1"] = 0;
		m["joint_2"] = 1;
		m["joint_3"] = 2;
		m["joint_4"] = 3;
		m["joint_5"] = 4;
		m["joint_6"] = 5;
		m["joint_7"] = 6;

	  	return m;
	}

	static std::map<std::string,int> JOINT_MAP = create_map();

 	static const std::string JOINT_NAME_ARRAY[] =
 	{
 		"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"
 	};

	static const unsigned int SIZE_JOINT_NAME_ARRAY = 7;

	static const std::string JOINT_ARM_NAME_ARRAY[] =
	{
		"joint_1", "joint_2", "joint_3", "joint_4", "joint_5","joint_6", "joint_7"
	};

	static const std::string* A_JOINT_ARM_NAME_ARRAY[] = {JOINT_ARM_NAME_ARRAY};

	static const unsigned int NR_OF_ARM_SLAVES = 7;
}