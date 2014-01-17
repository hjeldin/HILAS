#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/gp3.h>

int main(int argc, char ** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	ros::init(argc,argv,"pcl_elab");
	//get filename from argv?
	if(argc < 1)
		exit(1);

	std::string fn(argv[1]);
	pcl::io::loadPCDFile(fn,*pointCloud);

	//normal estimation
	pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> n;

	//openMP normal estimation
	pcl::NormalEstimationOMP<pcl::PointXYZRGB,pcl::Normal> ompn;

	pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );
	pcl::PointCloud<pcl::Normal>::Ptr normals_omp ( new pcl::PointCloud<pcl::Normal> );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	tree->setInputCloud(pointCloud);
	n.setInputCloud(pointCloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	std::cout << "Starting normal estimation" << std::endl;
	n.compute(*normals);
	std::cout << "Normals estimated" << std::endl;

	ompn.setInputCloud(pointCloud);
	ompn.setSearchMethod(tree);
	ompn.setKSearch(20);
	std::cout << "Starting normal estimation with openMP" << std::endl;
	ompn.compute(*normals_omp);
	std::cout << "Normals estimated" << std::endl;


	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*pointCloud,*normals_omp,*cloud_normals);

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

	ros::spin();
}