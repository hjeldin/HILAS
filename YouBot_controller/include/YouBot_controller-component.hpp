#ifndef OROCOS_YOUBOT_CONTROLLER_COMPONENT_HPP
#define OROCOS_YOUBOT_CONTROLLER_COMPONENT_HPP

#include <rtt/RTT.hpp>

extern "C"{
	#include <sys/select.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <unistd.h> /* close() */
	#include <string.h> /* memset() */
	#include <errno.h>
}

#include <rapidjson/document.h>

#define LENGTH 512
#define LOCAL_SERVER_PORT 12345
#define MAX_MSG 100

class YouBot_controller : public RTT::TaskContext{
  public:
    YouBot_controller(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
	void manage_msg(const char* msg);

	struct timeval tv;
	int sd, rc,nsockfd;
	unsigned int n,cliLen;
	struct sockaddr_in cliAddr, servAddr;
	char msg[MAX_MSG];
	char buf[512],sbuf[LENGTH];
	fd_set readfds;
};
#endif