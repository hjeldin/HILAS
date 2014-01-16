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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
ros::Subscriber cameraSubscriber;
ros::Subscriber startAcquisition;
ros::Publisher voxelizedPC;
time_t timer;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

void acquisitionCamera(const std_msgs::Bool msg)
{
	if(msg.data == true)
	{
		cameraSubscriber = nh->subscribe <sensor_msgs::PointCloud2> ("/camera/voxelizedPC",1,cloud_cb);
	} else {
		timer = time(NULL);

		sensor_msgs::PointCloud2::Ptr tmpcloud(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*accumCloud, *tmpcloud);
		
		
		pcl::VoxelGrid<sensor_msgs::PointCloud2> sor; 
		sor.setInputCloud (tmpcloud);
		sor.setLeafSize (0.01, 0.01, 0.01);
		sor.filter (*tmpcloud);

		pcl::fromROSMsg(*tmpcloud,*accumCloud);

		cameraSubscriber.shutdown();
		pcl::io::savePCDFile("test.pcd",*accumCloud);
		std::cout << "Shutting down subscriber and saving acquisition to :" << std::endl;
	}
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	sensor_msgs::PointCloud2::Ptr cloud_downsampled (new sensor_msgs::PointCloud2 ()); 
	pcl::fromROSMsg(*msg,*cloud);

	std::cout << (*cloud).size() << std::endl;
	// Create the filtering object
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor; 
	sor.setInputCloud (msg);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (*cloud_downsampled);

	pcl::fromROSMsg(*cloud_downsampled, *cloud);
	std::cout << (*cloud).size() << std::endl;

	// NaN points
	std::vector<int> v_nan;
	pcl::removeNaNFromPointCloud(*cloud,*cloud,v_nan);

	// Add acquired pointcloud
	*accumCloud += *cloud;
	std::cout << "Added frame to accumulation point cloud" << std::endl;
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"pcl_save");
	nh = new ros::NodeHandle();
	startAcquisition = nh->subscribe<std_msgs::Bool>("/camera/startAcquisition",1,acquisitionCamera);
	ros::spin();
	return 0;
}