#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
void poseCallback(const nav_msgs::Odometry::ConstPtr& odometry)
{
   //TF odom=> base_link
    
     static tf::TransformBroadcaster odom_broadcaster;
     static geometry_msgs::TransformStamped odometryTransform;
    
    odometryTransform.header.stamp = ros::Time::now();
    odometryTransform.header.frame_id = "odom";
    odometryTransform.child_frame_id = "base_footprint";
    odometryTransform.transform.translation.x = odometry->pose.pose.position.x;
    odometryTransform.transform.translation.y = odometry->pose.pose.position.y;
    odometryTransform.transform.translation.z = odometry->pose.pose.position.z;
    odometryTransform.transform.rotation = odometry->pose.pose.orientation;

    odom_broadcaster.sendTransform(odometryTransform);
 
}



int main(int argc, char** argv){
	ros::init(argc, argv, "odom_hw2TF");
	ros::NodeHandle n;

	ros::Rate r(300);
	
	ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, poseCallback);

	while(n.ok())
	{
	    ros::spinOnce();		
    	    r.sleep();
	}
}	
