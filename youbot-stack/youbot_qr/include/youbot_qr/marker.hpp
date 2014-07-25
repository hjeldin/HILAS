#pragma once
#include <visualization_msgs/Marker.h>
#include <youbot_qr/reader.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tfMessage.h>
namespace qrcode{

class Marker
{
  ros::NodeHandle nh;
  ros::Publisher pub_marker;
  visualization_msgs::Marker marker;
public:
  Marker() :
          pub_marker(nh.advertise<visualization_msgs::Marker>("vis_marker", 1))
  {
    marker.ns = "qrcodes";
    marker.type = visualization_msgs::Marker::CUBE;
  }
  void publish(float x, float y, float z, const std::string & frame_id, int id);

  void publish(const geometry_msgs::PoseStamped & target, int id);
};

class TfPublisher
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::Publisher pub_tf;
  ros::Publisher pub_tf_private;
public:
  TfPublisher() :
          private_nh("~"),
          pub_tf(nh.advertise<tf::tfMessage>("/tf", 5)),
          pub_tf_private(private_nh.advertise<tf::tfMessage>("tf_objects", 5))
  {
  }
  void publish(std::vector<QRCode> & codes, const sensor_msgs::ImageConstPtr& msg) const;
};

}