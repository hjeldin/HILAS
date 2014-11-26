#pragma once

#include <map>
#include <string>

namespace YouBot
{
	static std::map<std::string,int> create_map()
	{
		std::map<std::string,int> m;

		m["arm_joint_1"] = 0;
		m["arm_joint_2"] = 1;
		m["arm_joint_3"] = 2;
		m["arm_joint_4"] = 3;
		m["arm_joint_5"] = 4;
	 	m["wheel_joint_fl"] = 5;
		m["wheel_joint_fr"] = 6;
		m["wheel_joint_bl"] = 7;
		m["wheel_joint_br"] = 8;
	 	m["caster_joint_fl"] = 9;
		m["caster_joint_fr"] = 10;
		m["caster_joint_bl"] = 11;
		m["caster_joint_br"] = 12;
	 	m["gripper_finger_joint_l"] = 13;
		m["gripper_finger_joint_r"] = 14;

	  return m;
	}

	static std::map<std::string,int> JOINT_MAP = create_map();

 	static const std::string JOINT_NAME_ARRAY[] =
 	{
 		"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5",
 		"wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br",
 		"caster_joint_fl","caster_joint_fr", "caster_joint_bl", "caster_joint_br",
 		"gripper_finger_joint_l", "gripper_finger_joint_r"
 	};

	static const unsigned int SIZE_JOINT_NAME_ARRAY = 15;

	static const std::string JOINT_ARM_NAME_ARRAY[] =
	{
		"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"
	};

	static const std::string* A_JOINT_ARM_NAME_ARRAY[] = {JOINT_ARM_NAME_ARRAY};

	static const unsigned int NR_OF_ARM_SLAVES = 5;

	static const std::string JOINT_BASE_NAME_ARRAY[] =
	{
		"wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br"
	};

	static const std::string* A_JOINT_BASE_NAME_ARRAY[] = {JOINT_BASE_NAME_ARRAY};

	static const unsigned int NR_OF_BASE_SLAVES = 4;	

}