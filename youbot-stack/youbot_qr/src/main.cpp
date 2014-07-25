#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <youbot_qr/reader.hpp>
#include <youbot_qr/node.hpp>
using namespace std;


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "barcode_reader");
	qrcode::Node node("/camera/rgb/image_rect");
	ros::spin();
	return 0;
}