#include <pointcloud_save.h>
#include "vrep_common/simRosCreateDummy.h"
#include "vrep_common/simRosSetObjectPosition.h"
#include <nav_msgs/OccupancyGrid.h>
#include "vrep_common/simRosImportCubeMap.h"
#include <geometry_msgs/Point.h>
void mapCallback(const nav_msgs::OccupancyGrid map)
{
	ROS_INFO("%d", map.header.seq);
	//get step x/z from map params
	float resolution = map.info.resolution;
	ROS_INFO("LASER RESOLUTION %f", resolution);
	//OMP foreach if data > 80 place a block
	ros::ServiceClient client = nh->serviceClient<vrep_common::simRosImportCubeMap>("/vrep/simRosImportCubeMap",true);
	vrep_common::simRosImportCubeMap cm;
	cm.request.grid = map;
	if(client.call(cm)){
		ROS_INFO("called vrep plugin");
	}
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "map_builder");
	nh = new ros::NodeHandle();
	/*ros::ServiceClient client = node.serviceClient<nav_msgs::OccupacyGrid>("service",true);
	if(client){
		nav_msgs::OccupacyGrid og;
		if(client.exists())
		{
			client.call(og);
		}
	} else {
		ROS_ERROR("Could not connect to grid service");
	}*/
	ros::ServiceClient client = nh->serviceClient<vrep_common::simRosCreateDummy>("/vrep/simRosCreateDummy");
    vrep_common::simRosCreateDummy srv;
    srv.request.size = 0.01f;
    srv.request.colors = std::vector<signed char>();
    if (client.call(srv))
    {
        ROS_INFO("dummy created with id %d",srv.response.dummyHandle);
    }

    ros::Subscriber mapTopic = nh->subscribe<nav_msgs::OccupancyGrid>("/map",1,mapCallback);
    ros::spin();
	return 0;
}