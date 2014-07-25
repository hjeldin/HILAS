#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;

int sockfd, n;
struct sockaddr_in servaddr, cliaddr;
char* deployer_ip;
char* port;

void command_callback(const std_msgs::StringConstPtr& msg)
{
	sendto(sockfd, (*msg).data.c_str(), strlen((*msg).data.c_str()), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));	
    cout << "Command: " << (*msg).data << endl;
}

int main(int argc, char ** argv)
{
	cout << "Available commands:" << endl << endl;
	cout << "1 - Arm command" << endl;
	cout << "VREP_ARM_MODE_POS" << "\tHW_ARM_MODE_POS" << endl;
	cout << "VREP_ARM_MODE_VEL" << "\tHW_ARM_MODE_VEL" << endl;
	cout << "VREP_ARM_MODE_TOR" << "\tHW_ARM_MODE_TOR" << endl;
	cout << "VREP_ARM_MODE_STOP" << "\tHW_ARM_MODE_STOP" << endl;
	cout << endl << "2 - Base command" << endl;
	cout << "VREP_BASE_MODE_POS" << "\tHW_BASE_MODE_POS" << endl;
	cout << "VREP_BASE_MODE_VEL" << "\tHW_BASE_MODE_VEL" << endl;
	cout << "VREP_BASE_MODE_TOR" << "\tHW_BASE_MODE_TOR" << endl;
	cout << "VREP_BASE_MODE_STOP" << "\tHW_BASE_MODE_STOP" << endl;
	cout << "VREP_BASE_MODE_TWIST" << "\tHW_BASE_MODE_TWIST" << endl;
	cout << endl << "3 - Cartesian controller command:" << endl;
	cout << "CARTESIAN_START" << "\t\tCARTESIAN_STOP" << endl;
	cout << endl << "4 - Switch commands:" << endl;	
	cout << "SWITCH_TO_HW" << "\t\tSWITCH_TO_VREP" << endl;
	cout << endl << "5 - Block Position (fix arm pose, baseTwist set to zero" << endl;
	cout << "VREP_BLOCK_YOUBOT_POSITION" << "\tHW_BLOCK_YOUBOT_POSITION" << endl;
	cout << endl << "6 - Deployer exit" << endl;
	cout << "DEPLOYER_EXIT" << endl;

	ros::init(argc,argv,"deployer_commander");
	ros::NodeHandle nh;
	
	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	
	if(argc == 3)
	{
  	  deployer_ip = argv[1];
	  port = argv[2];
	} 
	else 
	{
	  std::cout << "./deployer_command <hostname> <port>" << std::endl;
	  exit(0);
	}

   	servaddr.sin_family = AF_INET;
   	servaddr.sin_addr.s_addr = inet_addr(deployer_ip);
   	servaddr.sin_port = htons(atoi(port));	

	cout << endl << "Socket up! " << deployer_ip << ":" << port << endl << std::flush;

	ros::Subscriber sub = nh.subscribe <std_msgs::String> ("/orocos/command",1,command_callback);

	ros::spin();
	return 0;
}
