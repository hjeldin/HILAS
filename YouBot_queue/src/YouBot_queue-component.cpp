#include "YouBot_queue-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

YouBot_queue::YouBot_queue(std::string const& name) : TaskContext(name)
{
	isinloading = true;

    this->addOperation("setIsInLoading", &YouBot_queue::setQueueMode, this);

    this->addPort("ros_arm_joint_position_command", ros_arm_joint_position_command).doc("ROS Command joint angles");
    this->addPort("ros_arm_joint_velocity_command", ros_arm_joint_velocity_command).doc("ROS Command joint velocities");
    this->addPort("ros_arm_joint_effort_command", ros_arm_joint_effort_command).doc("ROS Command joint torques");	

    this->addPort("ros_base_cmd_twist", ros_base_cmd_twist).doc("ROS Command base twist");
    this->addPort("ros_gripper_joint_position_command", ros_gripper_joint_position_command).doc("ROS Command the gripper position");    

    this->addPort("orocos_arm_joint_position_command", orocos_arm_joint_position_command).doc("Orocos Command joint angles");
    this->addPort("orocos_arm_joint_velocity_command", orocos_arm_joint_velocity_command).doc("Orocos Command joint velocities");
    this->addPort("orocos_arm_joint_effort_command", orocos_arm_joint_effort_command).doc("Orocos Command joint torques");	

    this->addPort("orocos_base_cmd_twist", orocos_base_cmd_twist).doc("Orocos Command base twist");
    this->addPort("orocos_gripper_joint_position_command", orocos_gripper_joint_position_command).doc("Orocos Command the gripper position");    

    this->addPort("out_arm_joint_position_command", out_arm_joint_position_command).doc("Out command joint angles");
    this->addPort("out_arm_joint_velocity_command", out_arm_joint_velocity_command).doc("Out command joint velocities");
    this->addPort("out_arm_joint_effort_command", out_arm_joint_effort_command).doc("Out command joint torques");	

    this->addPort("out_base_cmd_twist", out_base_cmd_twist).doc("Out command base twist");
    this->addPort("out_gripper_cmd_position", out_gripper_joint_position_command).doc("Out command the gripper position");
}

void YouBot_queue::setQueueMode(bool mode)
{
	isinloading = mode;
}

bool YouBot_queue::configureHook()
{
	return true;
}

bool YouBot_queue::startHook()
{
	return true;
}

void YouBot_queue::updateHook()
{
	if(isinloading)
	{
		if(ros_arm_joint_position_command.read(ros_arm_joint_position_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(ros_arm_joint_position_command_data,timestamp));
		}
		
		if(ros_arm_joint_velocity_command.read(ros_arm_joint_velocity_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(ros_arm_joint_velocity_command_data,timestamp));
		}
		
		if(ros_arm_joint_effort_command.read(ros_arm_joint_effort_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(ros_arm_joint_effort_command_data,timestamp));		
		}

		if(ros_base_cmd_twist.read(ros_base_cmd_twist_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(ros_base_cmd_twist_data,timestamp));		
		}

		if(ros_gripper_joint_position_command.read(ros_gripper_joint_position_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(ros_gripper_joint_position_command_data,3,timestamp));		
		}

		if(orocos_arm_joint_position_command.read(orocos_arm_joint_position_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(orocos_arm_joint_position_command_data,timestamp));		
		}

		if(orocos_arm_joint_velocity_command.read(orocos_arm_joint_velocity_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(orocos_arm_joint_velocity_command_data,timestamp));		
		}

		if(orocos_arm_joint_effort_command.read(orocos_arm_joint_effort_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(orocos_arm_joint_effort_command_data,timestamp));		
		}

		if(orocos_base_cmd_twist.read(orocos_base_cmd_twist_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(orocos_base_cmd_twist_data,timestamp));		
		}

		if(orocos_gripper_joint_position_command.read(orocos_gripper_joint_position_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			queue.push_back(queue_item(orocos_gripper_joint_position_command_data,3,timestamp));	
		}
	}
	else
	{
		if(queue.empty())
		{
			return;
		}

		struct queue_item tmp = queue.back();
		queue.pop_back();

		switch(tmp.mode_index)
		{
			case 0:
				out_arm_joint_position_command.write(tmp.arm_pos);
				break;

			case 1:
				out_arm_joint_velocity_command.write(tmp.arm_vel);
				break;

			case 2:
				out_arm_joint_effort_command.write(tmp.arm_eff);			
				break;			

			case 3:
				out_base_cmd_twist.write(tmp.base_twist);
				break;			

			case 4:
				out_gripper_joint_position_command.write(tmp.gripper_pos);			
				break;

			default:
				break;
		}
	}
}

void YouBot_queue::stopHook()
{

}

void YouBot_queue::cleanupHook()
{

}

ORO_CREATE_COMPONENT(YouBot_queue)