#include <youbot_qr/marker.hpp>

using namespace qrcode;

void Marker::publish(float x, float y, float z, const std::string & frame_id, int id)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1;
    publish(pose, id);
}

void Marker::publish(const geometry_msgs::PoseStamped & target, int id)
{
	marker.id = id;
	marker.header.stamp = target.header.stamp;
	marker.header.frame_id = target.header.frame_id;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = target.pose;

	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = 0.7;
	marker.color.g = 0.7;
	marker.color.b = 0.7;
	marker.color.a = 0.1;
	marker.lifetime = ros::Duration(0);
	pub_marker.publish(marker);
}

void TfPublisher::publish(std::vector<qrcode::QRCode> & codes, const sensor_msgs::ImageConstPtr& msg) const
{
	tf::tfMessage tf_msg;
	geometry_msgs::TransformStamped tr;
	tr.header = msg->header;

	for (uint i = 0; i < codes.size(); i++)
	{
	  qrcode::QRCode & code = codes[i];
	  tr.child_frame_id = code.data;
	  tr.transform.rotation.w = 1;
	  tr.transform.translation.x = code.x;
	  tr.transform.translation.y = code.y;
	  tr.transform.translation.z = code.z;
	  tf_msg.transforms.push_back(tr);
	}
	pub_tf.publish(tf_msg);
	pub_tf_private.publish(tf_msg);
}