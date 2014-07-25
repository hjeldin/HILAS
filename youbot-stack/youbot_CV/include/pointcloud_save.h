#ifndef PCL_SAVE
#define PCL_SAVE

#include <ros/ros.h>
#include <tf/transform_listener.h>
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

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h> 
#include <pcl/filters/passthrough.h>
#include "vrep_common/simRosImportMesh.h"
#include <pcl/filters/bilateral.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/icp.h>

ros::NodeHandle *nh;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
ros::Subscriber cameraSubscriber;
ros::Subscriber startAcquisition;
ros::Publisher voxelizedPC;
time_t timer;
int prevActivation = false;

tf::TransformListener * tfListener;
tf::StampedTransform * st;

void vrepPlaceMesh();
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
void pcdToMesh();

#endif	//PCL_SAVE
