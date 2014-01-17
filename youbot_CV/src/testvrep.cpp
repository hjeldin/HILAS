#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/gp3.h>
#include <dlfcn.h>
#include "v_repLib.h"

LIBRARY vrepLib;
ros::NodeHandle *nh;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"vreptest");
	nh = new ros::NodeHandle();
	vrepLib=loadVrepLibrary("/home/hjeldin/DEV/youbot-stack/youbot_CV/lib/libv_rep.so");

	if(vrepLib == NULL)
	{
		std::cout << dlerror() << std::endl;
		exit(1);
	}
	ros::spin();
	return 0;
}