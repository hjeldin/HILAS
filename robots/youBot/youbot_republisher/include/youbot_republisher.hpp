#pragma once

#include <interfaces/IRobotStateRepublisher.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <tf/tf.h>

namespace YouBot
{
 	using namespace RTT;
 	using namespace std;

 	static const std::string JOINT_NAME_ARRAY[] =
 	{"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5",
 	"wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br",
 	"caster_joint_fl","caster_joint_fr", "caster_joint_bl", "caster_joint_br",
 	"gripper_finger_joint_l", "gripper_finger_joint_r"};

	static const unsigned int SIZE_JOINT_NAME_ARRAY = 15;

class YouBotStateRepublisher: public Hilas::IRobotStateRepublisher
{

public:
	YouBotStateRepublisher(std::string const& name);
 	~YouBotStateRepublisher();
 	void updateHook();
};

}