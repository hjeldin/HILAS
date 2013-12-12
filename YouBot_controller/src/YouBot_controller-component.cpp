#include "YouBot_controller-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

YouBot_controller::YouBot_controller(std::string const& name) : TaskContext(name){}

void YouBot_controller::manage_msg(const char* json_msg)
{
	rapidjson::Document d;
	d.Parse<0>(json_msg);
	
	printf("%s\n", d["hello"].GetString());
}

bool YouBot_controller::configureHook(){

	RTT::Seconds period = this->getPeriod();

	tv.tv_sec = period;
	tv.tv_usec = (period/1e6)/2;

	/* socket creation */
	FD_ZERO(&readfds);
	sd = socket(AF_INET, SOCK_DGRAM, 0);
	FD_SET(sd, &readfds);

	if(sd < 0)
	{
		printf("Cannot open socket\n");
		exit(1);
	}

	/* bind local server port */
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons(LOCAL_SERVER_PORT);
	
	rc = bind (sd, (struct sockaddr *) &servAddr,sizeof(servAddr));

	if(rc < 0)
	{
		printf("Cannot bind port number %d\n", LOCAL_SERVER_PORT);
		return false;
	}

	printf("Waiting for data on port UDP %u\n", LOCAL_SERVER_PORT);

	return true;
}

bool YouBot_controller::startHook()
{
	return true;
}

void YouBot_controller::updateHook(){
	
	if (select(sd+1, &readfds, NULL, NULL, &tv) >= 0)
	{
		/* init buffer */
		memset(msg,0x0,MAX_MSG);

		/* receive message */
		cliLen = sizeof(cliAddr);
		n = recvfrom(sd, msg, MAX_MSG, 0,(struct sockaddr *) &cliAddr, &cliLen);

		if(n < 0)
		{
			printf("Cannot receive data \n");
		}

		/* print received message */
		printf("From %s : UDP%u : %s \n", inet_ntoa(cliAddr.sin_addr), ntohs(cliAddr.sin_port),msg);
		manage_msg(msg);
	}
}

void YouBot_controller::stopHook()
{

}

void YouBot_controller::cleanupHook()
{

}

ORO_CREATE_COMPONENT(YouBot_controller)