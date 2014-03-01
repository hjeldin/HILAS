#include "pointcloud_save.h"

pcl::PointCloud<pcl::PointXYZRGB> * prevCloud;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_container;

int pcdCounter = 0;
void vrepPlaceMesh()
{
    /*ros::ServiceClient client = nh->serviceClient<vrep_common::simRosImportMesh>("/vrep/simRosImportMesh");
    vrep_common::simRosImportMesh srv;
    srv.request.fileFormat = 0;
    srv.request.fileName = std::string("/home/hjeldin/DEV/youbot-stack/youbot_CV/bin/mesh.obj");
    srv.request.options = 0;
    srv.request.identicalVerticeTolerance = 0.0001f;
    if (client.call(srv))
    {
        ROS_INFO("ahah ok");
    }*/
}

void alignPCLs()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr firstCloud = cloud_container[0];
	// Add acquired pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr totalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = cloud_container.begin(); it != cloud_container.end(); it++)
	{
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr _tmptr (firstCloud);

		icp.setMaxCorrespondenceDistance (0.05);
		icp.setMaximumIterations (50);
		icp.setTransformationEpsilon (1e-8);		
		icp.setEuclideanFitnessEpsilon (1);
		icp.setInputCloud((*it));
		icp.setInputTarget(_tmptr);
		icp.align(*(*it));
		ROS_DEBUG_STREAM("has converged:" << icp.hasConverged() << " " << icp.getFitnessScore());
		if(icp.getFitnessScore() < 0.1f){
	  		Eigen::Matrix4f T = icp.getFinalTransformation();
	  		pcl::transformPointCloud(**it,**it,T);
	  		*(*it) += *accumCloud;
	  		*totalCloud += *(*it);
	  	}
	  	firstCloud = *(it);
	}

	*accumCloud = *totalCloud;
}


void pcdToMesh()
{

	//openMP normal estimation
	pcl::NormalEstimationOMP<pcl::PointXYZRGB,pcl::Normal> ompn;
	pcl::PointCloud<pcl::Normal>::Ptr normals_omp ( new pcl::PointCloud<pcl::Normal> );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	tree->setInputCloud(accumCloud);
	ompn.setInputCloud(accumCloud);
	ompn.setSearchMethod(tree);
	ompn.setKSearch(20);
	ROS_DEBUG("Starting normal estimation with openMP");
	ompn.compute(*normals_omp);
	ROS_DEBUG("Normals estimated");

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

	// Here we should call meshlabserver and wait till it returns

	// Here we rotate the mesh to align to vrep's axis.

	pcl::io::saveOBJFile ("mesh.obj", triangles);

	vrepPlaceMesh();

	ROS_DEBUG("Mesh saved");
}

void acquisitionCamera(const std_msgs::Bool msg)
{
	if(msg.data == true)
	{
		cameraSubscriber = nh->subscribe <sensor_msgs::PointCloud2> ("/camera/voxelizedPC",1,cloud_cb);
		prevActivation = true;
	}
	else
	{
		if(!prevActivation) return;
		
		timer = time(NULL);
		alignPCLs();
		sensor_msgs::PointCloud2::Ptr tmpcloud(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*accumCloud, *tmpcloud);
		
		ROS_INFO("Voxelizing and filtering outliers");
		//Voxelization with 1cm resolution
		pcl::VoxelGrid<sensor_msgs::PointCloud2> vox; 
		vox.setInputCloud (tmpcloud);
		vox.setLeafSize (0.01, 0.01, 0.01);
		vox.filter (*tmpcloud);

		pcl::fromROSMsg(*tmpcloud,*accumCloud);
		pcl::io::savePCDFile("voxelized.pcd",*accumCloud);

		cameraSubscriber.shutdown();
		//pcl::io::savePCDFile("planes.pcd",*planes);
		pcl::io::savePCDFile("test.pcd",*accumCloud);		
		
		pcdToMesh();
		ROS_INFO("Job complete");
		exit(1);
	}
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	sensor_msgs::PointCloud2::Ptr cloud_downsampled (new sensor_msgs::PointCloud2 ()); 
	pcl::fromROSMsg(*msg,*cloud);

	ROS_INFO_STREAM("Received " << (*cloud).size()<< " points. Downsampling...");
	// Create the filtering object
	pcl::VoxelGrid<sensor_msgs::PointCloud2> vox; 
	vox.setInputCloud (msg);
	vox.setLeafSize (0.01, 0.01, 0.01);
	vox.filter (*cloud_downsampled);

	pcl::fromROSMsg(*cloud_downsampled, *cloud);
	ROS_INFO_STREAM("to " << (*cloud).size()<< " points.");

	// NaN points
	std::vector<int> v_nan;
	pcl::removeNaNFromPointCloud(*cloud,*cloud,v_nan);

	//grab only points from y[0.1m,4m], z[0.1,4]
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	// TODO: Limits will be set by messages
	pass.setFilterLimits (0.1, 1.5);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud);
	//pass.setInputCloud(cloud);
	//pass.setFilterFieldName("y");
	//pass.setFilterLimits(0.0,4);
	//pass.filter(*cloud);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud);
	//pcl::io::savePCDFile("sor.pcd",*accumCloud);	

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.05); 
	mls.reconstruct(*cloud);

	st = new tf::StampedTransform();
	tfListener->waitForTransform("/base_link", "/camera_depth_optical_frame",msg->header.stamp, ros::Duration(0.1));
	tfListener->lookupTransform("/base_link","/camera_depth_optical_frame",msg->header.stamp,(*st));	
	Eigen::Matrix4f T; 	
	pcl_ros::transformAsMatrix ((*st), T); 
	delete st;
	pcl::transformPointCloud(*cloud,*cloud,T);
	cloud_container.push_back(cloud);

	std::stringstream ss;
	ss<< pcdCounter<<".pcd";
	pcdCounter++;
	pcl::io::savePCDFile(ss.str().c_str(),*cloud);
	ROS_DEBUG("Added frame to accumulation point cloud");
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"pcl_save");
	nh = new ros::NodeHandle();
	tfListener = new tf::TransformListener();
	ROS_INFO("Waiting for /camera/startAcquisition to be true");
	startAcquisition = nh->subscribe<std_msgs::Bool>("/camera/startAcquisition",1,acquisitionCamera);
	ros::spin();
	return 0;
}
