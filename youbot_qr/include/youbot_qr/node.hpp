#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <youbot_qr/marker.hpp>
#include <youbot_qr/reader.hpp>
#pragma once

namespace qrcode{

class Node
{
private:
  qrcode::QRCodeReader reader;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub_image;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  Marker vis_;
  TfPublisher object_pub_;
public:
  Node(const char * topicName) :
          it(nh),
          sub_image(it.subscribe(topicName, 1, &Node::imageCallback, this))
  {
    ROS_INFO("Starting qrcode reader.");
    reader.setFOV(60);
  }

};

}