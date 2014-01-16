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

ros::NodeHandle *nh;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
ros::Subscriber cameraSubscriber;
ros::Subscriber startAcquisition;
ros::Publisher voxelizedPC;
time_t timer;

void pcdToMesh()
{
	//normal estimation
	pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> n;

	//openMP normal estimation
	pcl::NormalEstimationOMP<pcl::PointXYZRGB,pcl::Normal> ompn;

	pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );
	pcl::PointCloud<pcl::Normal>::Ptr normals_omp ( new pcl::PointCloud<pcl::Normal> );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	tree->setInputCloud(accumCloud);
	ompn.setInputCloud(accumCloud);
	ompn.setSearchMethod(tree);
	ompn.setKSearch(20);
	std::cout << "Starting normal estimation with openMP" << std::endl;
	ompn.compute(*normals_omp);
	std::cout << "Normals estimated" << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*accumCloud,*normals_omp,*cloud_normals);

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;

	gp3.setSearchRadius(0.025);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	pcl::io::saveOBJFile ("mesh.obj", triangles);

	std::cout << "Mesh saved" << std::endl;	
}

void vrepPlaceMesh()
{
	simxStart();

    simFloat** vertices;
    simInt* verticesSizes;
    simInt** indices;
    simInt* indicesSizes;
    simChar** names;

    simInt elementCount=simImportMesh(0,"./mesh.obj",0,0.0001f,1.0f,&vertices,&verticesSizes,&indices,&indicesSizes,NULL,&names);

    if (elementCount>0)
    {
        const float grey[3]={0.5f,0.5f,0.5f};
        for (int i=0;i<elementCount;i++)
        {
            simInt shapeHandle=simCreateMeshShape(2,20.0f*3.1415f/180.0f,vertices[i],verticesSizes[i],indices[i],indicesSizes[i],NULL);
            simSetObjectName(shapeHandle,names[i]);
            simSetShapeColor(shapeHandle,"",0,grey);
            simReleaseBuffer(names[i]);
            simReleaseBuffer((simChar*)indices[i]);
            simReleaseBuffer((simChar*)vertices[i]);
        }
        simReleaseBuffer((simChar*)names);
        simReleaseBuffer((simChar*)indicesSizes);
        simReleaseBuffer((simChar*)indices);
        simReleaseBuffer((simChar*)verticesSizes);
        simReleaseBuffer((simChar*)vertices);
    }	
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

void acquisitionCamera(const std_msgs::Bool msg)
{
	if(msg.data == true)
	{
		cameraSubscriber = nh->subscribe <sensor_msgs::PointCloud2> ("/camera/voxelizedPC",1,cloud_cb);
	}
	else
	{
		timer = time(NULL);

		sensor_msgs::PointCloud2::Ptr tmpcloud(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*accumCloud, *tmpcloud);
		
		
		pcl::VoxelGrid<sensor_msgs::PointCloud2> sor; 
		sor.setInputCloud (tmpcloud);
		sor.setLeafSize (0.01, 0.01, 0.01);
		sor.filter (*tmpcloud);

		pcl::fromROSMsg(*tmpcloud,*accumCloud);

		cameraSubscriber.shutdown();
		
		pcdToMesh();
		vrepPlaceMesh();

		//pcl::io::savePCDFile("test.pcd",*accumCloud);
		std::cout << "Job complete" << std::endl;
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