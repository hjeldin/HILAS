/**********************************************************************
 *
 * Copyright (c) 2010-2013
 * All rights reserved.
 *
 * Robotics and Mechatronics (RaM) group
 * Faculty of Electrical Engineering, Mathematics and Computer Science
 * University of Twente
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author(s):
 * Yury Brodskiy
 *
 * Supervised by: 
 * Jan F. Broenink
 * 
 * The research leading to these results has received funding from the 
 * European Community's Seventh Framework Programme (FP7/2007-2013) 
 * under grant agreement no. FP7-ICT-231940-BRICS (Best Practice in 
 * Robotics).
 * 
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This software is published under a dual-license: GNU Lesser General 
 * Public License LGPL 2.1 and BSD license. The dual-license implies 
 * that users of this code may choose which terms they prefer.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above 
 *       copyright notice, this list of conditions and the following 
 *       disclaimer in the documentation and/or other materials 
 *       provided with the distribution.
 *     * Neither the name of the University of Twente nor the names of 
 *       its contributors may be used to endorse or promote products 
 *       derived from this software without specific prior written 
 *       permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more 
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 **********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

std::string topic_name;


void toTransformMatrix(const std::vector<double> & tf, tf::Transform& trans) {
        trans.setOrigin(tf::Vector3(tf[3], tf[7], tf[11]));
        btMatrix3x3 rotMatrix(tf[0], tf[1], tf[2],
                        tf[4], tf[5], tf[6],
                        tf[8], tf[9], tf[10]);
        btQuaternion quat;
        rotMatrix.getRotation(quat);
        trans.setRotation(quat);

}

void adapterCallback(std_msgs::Float64MultiArrayConstPtr msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  toTransformMatrix(msg->data,transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", topic_name));

}



int main(int argc, char** argv){
  ros::init(argc, argv, "HomogeneousToTF");
  if (argc != 2){ROS_ERROR("need topic name as argument"); return -1;};
  topic_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(topic_name, 10, &adapterCallback);

  ros::spin();
  return 0;
};
