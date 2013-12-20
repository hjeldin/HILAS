#include "YouBot_queue-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

YouBot_queue::YouBot_queue(std::string const& name) : TaskContext(name)
{
	isinloading = true;
	from_planner_goal_data = true;
	time_of_the_last = 0;

    this->addOperation("setIsInLoading", &YouBot_queue::setQueueMode, this);

    //this->addPort("from_hw_odom", from_hw_odom).doc("Odometry status from HW");
    this->addPort("from_planner_goal", from_planner_goal).doc("Planner status from HW");    
   	this->addPort("from_cartesian_status", from_cartesian_status).doc("Cartesian status from HW"); 
    //this->addPort("from_hw_arm_joints", from_hw_arm_joints).doc("Arm joints status from HW");

    this->addPort("ros_arm_joint_position_command", ros_arm_joint_position_command).doc("ROS Command joint angles");
    this->addPort("ros_arm_joint_velocity_command", ros_arm_joint_velocity_command).doc("ROS Command joint velocities");
    this->addPort("ros_arm_joint_effort_command", ros_arm_joint_effort_command).doc("ROS Command joint torques");	

    this->addPort("ros_base_cmd_twist", ros_base_cmd_twist).doc("ROS Command base twist");
    this->addPort("ros_gripper_joint_position_command", ros_gripper_joint_position_command).doc("ROS Command the gripper position");    

    this->addPort("ros_planner_command", ros_planner_command).doc("ROS planner command");
    this->addPort("ros_cartesian_command", ros_cartesian_command).doc("ROS cartesian command");    

    this->addPort("orocos_arm_joint_position_command", orocos_arm_joint_position_command).doc("Orocos Command joint angles");
    this->addPort("orocos_arm_joint_velocity_command", orocos_arm_joint_velocity_command).doc("Orocos Command joint velocities");
    this->addPort("orocos_arm_joint_effort_command", orocos_arm_joint_effort_command).doc("Orocos Command joint torques");	

    this->addPort("orocos_base_cmd_twist", orocos_base_cmd_twist).doc("Orocos Command base twist");
    this->addPort("orocos_gripper_joint_position_command", orocos_gripper_joint_position_command).doc("Orocos Command the gripper position");    

    this->addPort("out_arm_joint_position_command", out_arm_joint_position_command).doc("Out command joint angles");
    this->addPort("out_arm_joint_velocity_command", out_arm_joint_velocity_command).doc("Out command joint velocities");
    this->addPort("out_arm_joint_effort_command", out_arm_joint_effort_command).doc("Out command joint torques");	

    this->addPort("out_base_cmd_twist", out_base_cmd_twist).doc("Out command base twist");
    this->addPort("out_gripper_joint_position_command", out_gripper_joint_position_command).doc("Out command the gripper position");

    this->addPort("out_ros_planner_command", out_ros_planner_command).doc("Out ROS planner command");
    this->addPort("out_ros_cartesian_command", out_ros_cartesian_command).doc("Out ROS cartesian command");    
}

void YouBot_queue::setQueueMode(bool mode)
{
	if(!mode)
	{
		my_time = ros::Time::now().toNSec();
	}

	isinloading = mode;
}

void YouBot_queue::my_push_front(const queue_item& q)
{
	if(queue.size() == 0)
	{
		time_of_the_last = q.timestamp;
		last_command_submitted = q;
	}
	queue.push_front(q);
}

double getDist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));
}

bool YouBot_queue::lastCartesianPoseIsReached()
{
    KDL::Rotation last_cmd_orient =  KDL::Rotation::Quaternion(
    												last_command_submitted.cartesian.orientation.x,
    												last_command_submitted.cartesian.orientation.y,
                                                    last_command_submitted.cartesian.orientation.z,
                                                    last_command_submitted.cartesian.orientation.w);

    KDL::Rotation setpoint_orient =  KDL::Rotation::Quaternion(
    												from_cartesian_status_data.orientation.x,
    												from_cartesian_status_data.orientation.y,
                                                    from_cartesian_status_data.orientation.z,
                                                    from_cartesian_status_data.orientation.w);

	KDL::Rotation diff_orient = setpoint_orient * last_cmd_orient.Inverse();

	double diff_roll, diff_pitch, diff_yaw, diff_dist;

	diff_orient.GetRPY(diff_roll,diff_pitch,diff_yaw);
	diff_dist = getDist(last_command_submitted.cartesian.position,from_cartesian_status_data.position);

	if(diff_dist <= POS_DISTANCE && diff_roll <= ORIENT_OFFSET && diff_pitch <= ORIENT_OFFSET && diff_yaw <= ORIENT_OFFSET)
		return true;
	else
		return false;
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
			my_push_front(queue_item(ros_arm_joint_position_command_data,timestamp));
		}
		
		if(ros_arm_joint_velocity_command.read(ros_arm_joint_velocity_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(ros_arm_joint_velocity_command_data,timestamp));
		}
		
		if(ros_arm_joint_effort_command.read(ros_arm_joint_effort_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(ros_arm_joint_effort_command_data,timestamp));		
		}

		if(ros_base_cmd_twist.read(ros_base_cmd_twist_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(ros_base_cmd_twist_data,timestamp));		
		}

		if(ros_gripper_joint_position_command.read(ros_gripper_joint_position_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(ros_gripper_joint_position_command_data,0,timestamp));		
		}

		if(ros_planner_command.read(ros_planner_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(ros_planner_command_data,timestamp));		
		}

		if(ros_cartesian_command.read(ros_cartesian_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(ros_cartesian_command_data,timestamp));		
		}

		if(orocos_arm_joint_position_command.read(orocos_arm_joint_position_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(orocos_arm_joint_position_command_data,timestamp));		
		}

		if(orocos_arm_joint_velocity_command.read(orocos_arm_joint_velocity_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(orocos_arm_joint_velocity_command_data,timestamp));		
		}

		if(orocos_arm_joint_effort_command.read(orocos_arm_joint_effort_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(orocos_arm_joint_effort_command_data,timestamp));		
		}

		if(orocos_base_cmd_twist.read(orocos_base_cmd_twist_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(orocos_base_cmd_twist_data,timestamp));		
		}

		if(orocos_gripper_joint_position_command.read(orocos_gripper_joint_position_command_data) == NewData)
		{
			uint64_t timestamp = ros::Time::now().toNSec();
			my_push_front(queue_item(orocos_gripper_joint_position_command_data,3,timestamp));	
		}
	}
	else
	{
		from_planner_goal.read(from_planner_goal_data);
		from_cartesian_status.read(from_cartesian_status_data);

		// if(from_planner_goal_data)
		// {
		// 	std::cout << "Planner on goal\n";
		// }
		// else
		// {
		// 	std::cout << "Planner not on goal\n";			
		// }

		// if(lastCartesianPoseIsReached())
		// {
		// 	std::cout << "Cartesian on goal\n";
		// }
		// else
		// {
		// 	std::cout << "Cartesian not on goal\n";			
		// }

		// if(queue.empty())
		// {
		// 	std::cout << "Queue empty\n";
		// }

		if(queue.empty() || ((last_command_submitted.mode_index == PLANNER) && !from_planner_goal_data) || 
							((last_command_submitted.mode_index == CARTESIAN) && !lastCartesianPoseIsReached()))
		{
			return;
		}

		uint64_t now = ros::Time::now().toNSec();
		struct queue_item tmp_now = queue.back();		
		uint64_t delta = tmp_now.timestamp - time_of_the_last;

		if((now - my_time) >= delta)
		{
			switch(tmp_now.mode_index)
			{
				case 0:
					out_arm_joint_position_command.write(tmp_now.arm_pos);
					break;

				case 1:
					out_arm_joint_velocity_command.write(tmp_now.arm_vel);
					break;

				case 2:
					out_arm_joint_effort_command.write(tmp_now.arm_eff);			
					break;			

				case 3:
					out_gripper_joint_position_command.write(tmp_now.gripper_pos);				
					break;			

				case 4:
					out_base_cmd_twist.write(tmp_now.base_twist);			
					break;

				case 5:
					out_ros_planner_command.write(tmp_now.planner);
					break;

				case 6:
					out_ros_cartesian_command.write(tmp_now.cartesian);
					break;

				default:
					break;
			}

			my_time = now;
			time_of_the_last = tmp_now.timestamp;
			last_command_submitted = tmp_now;
			queue.pop_back();		
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