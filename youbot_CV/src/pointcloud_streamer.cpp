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

ros::NodeHandle *nh;

ros::Subscriber cameraSubscriber;
ros::Subscriber startAcquisition;
ros::Publisher voxelizedPC;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

void acquisitionCamera(const std_msgs::Bool msg)
{
	if(msg.data == true)
	{
		ROS_INFO("Start pointcloud publishing");
		cameraSubscriber = nh->subscribe <sensor_msgs::PointCloud2> ("/camera/depth_registered/points",1,cloud_cb);
	} else {
		ROS_INFO("Stop pointcloud publishing");
		cameraSubscriber.shutdown();
	}
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	sensor_msgs::PointCloud2::Ptr cloud_downsampled (new sensor_msgs::PointCloud2 ()); 
	pcl::fromROSMsg(*msg,*cloud);

	// Create the filtering object
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor; 
	sor.setInputCloud (msg);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (*cloud_downsampled);

	//Publish on specified topic
	voxelizedPC.publish(*cloud_downsampled);
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"pcl_stream");
	nh = new ros::NodeHandle();

	startAcquisition = nh->subscribe<std_msgs::Bool>("/camera/startAcquisition",1,acquisitionCamera);
	voxelizedPC = nh->advertise<sensor_msgs::PointCloud2>("/camera/voxelizedPC", 5);
	ros::spin();
	return 0;
}