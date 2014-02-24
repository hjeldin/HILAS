#include <youbot_qr/node.hpp>

using namespace qrcode;

void Node::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_img_ptr;
	msg->header;

	try
	{
	  cv_img_ptr = cv_bridge::toCvCopy(msg, "mono8");
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR_STREAM("Could not convert ROS image to CV: "<< e.what());
	  std::cout << "error " << std::endl;
	  return;
	}

	static int id = 0;
	reader.get(cv_img_ptr);
	std::vector<qrcode::QRCode> codes = reader.getQRs();
	for (uint i = 0; i < codes.size(); i++)
	{
	  ROS_DEBUG_STREAM("Barcode: " << codes[i].data //
	      << " x:"<<codes[i].x//
	      << " y:"<<codes[i].y);
	  vis_.publish(codes[i].x, codes[i].y, codes[i].z, msg->header.frame_id, id++ % 1000);
	}
	if (msg->header.frame_id == "")
	{
	  ROS_ERROR_THROTTLE(1, "Received image with empty frame_id, would cause tf connectivity issues.");
	}
	object_pub_.publish(codes, msg);
}