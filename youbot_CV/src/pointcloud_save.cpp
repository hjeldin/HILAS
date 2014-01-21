#include "pointcloud_save.h"



void vrepPlaceMesh()
{
	ros::ServiceClient client = nh->serviceClient<vrep_common::simRosImportMesh>("/vrep/simRosImportMesh");
    vrep_common::simRosImportMesh srv;
    srv.request.fileFormat = 0;
    srv.request.fileName = std::string("/home/hjeldin/DEV/youbot-stack/youbot_CV/bin/mesh.obj");
    srv.request.options = 0;
    srv.request.identicalVerticeTolerance = 0.0001f;
    if (client.call(srv))
    {
        ROS_INFO("ahah ok");
    }
}

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

	// Here we should call meshlabserver and wait till it returns

	// Here we rotate the mesh to align to vrep's axis.

	pcl::io::saveOBJFile ("mesh.obj", triangles);

	vrepPlaceMesh();

	std::cout << "Mesh saved" << std::endl;	
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
		//Statistical outlier removal
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(accumCloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*accumCloud);
		pcl::io::savePCDFile("sor.pcd",*accumCloud);	

		// TODO: Do we really need planar extraction? YES
		// If we can get a plane, we could simply add a goddamned fucking plane to vrep instead of the whole pointcloud
		// EDIT: actually, no, plane extraction is crappy with kinect sensors since depth extrapolation sucks.
		// 		 so we should rely on triangulation and semplification using quadric error. That if, of course, 
		//		 we can actually manage to grab meaningful pointclouds.
		// SEE:  http://users.csc.calpoly.edu/~zwood/teaching/csc570/final06/jseeba/ or
		//		 http://graphics.stanford.edu/courses/cs468-10-fall/LectureSlides/08_Simplification.pdf

		/*pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		ROS_INFO("Planes segmentation");
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.1);
		seg.setInputCloud(accumCloud);
		seg.segment(*inliers,*coefficients);

		if(inliers->indices.size()==0)
		{
			ROS_INFO("Could not estimate planar model.");
		}

		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes (new pcl::PointCloud<pcl::PointXYZRGB>);
		extract.setInputCloud(accumCloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*planes);

		//Remove planes from accumulation Point Cloud
		extract.setNegative(true);
		extract.filter(*accumCloud);
		*/

		cameraSubscriber.shutdown();
		//pcl::io::savePCDFile("planes.pcd",*planes);
		pcl::io::savePCDFile("test.pcd",*accumCloud);		
		
		pcdToMesh();
		std::cout << "Job complete" << std::endl;
		exit(1);
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


	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	// TODO: Limits will be set by messages
	pass.setFilterLimits (0.0, 3.4);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud);

	st = new tf::StampedTransform();
	//TODO: should subscribe to /tf, save previous /tf, subtract the two and transform each accumulated point cloud accordingly
	tfListener->lookupTransform("/arm_link_5","/base_link",ros::Time(),(*st));
	Eigen::Matrix4f T; 
	pcl_ros::transformAsMatrix ((*st), T); 
	delete st;
	pcl::transformPointCloud(*cloud,*cloud,T);
	// Add acquired pointcloud
	*accumCloud += *cloud;
	std::cout << "Added frame to accumulation point cloud" << std::endl;
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"pcl_save");
	nh = new ros::NodeHandle();
	tfListener = new tf::TransformListener();

	startAcquisition = nh->subscribe<std_msgs::Bool>("/camera/startAcquisition",1,acquisitionCamera);
	ros::spin();
	return 0;
}