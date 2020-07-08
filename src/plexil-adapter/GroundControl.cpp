// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.


// ow_autonomy
#include "subscriber.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"


void commandsCallback(const std_msgs::String::ConstPtr& cmd)
{
  ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
}

int main(int argc, char **argv)
{
  // Initialize GroundControl ROS node.
  ros::init(argc, argv, "GroundControl");

  ros::NodeHandle nHandle;

  // subscribe to /GroundControl/message topic
  ros::Subscriber sub = nHandle.subscribe("/GroundControl/message", 3, commandsCallback);
  
  ros::Publisher pub = nHandle.advertise<geometry_msgs::Point>
    ("/GroundControl/fwd_link", 3, true);

  geometry_msgs::Point xyz;

  xyz.x = 6;
  xyz.y = 7;
  xyz.z = 0;

  pub.publish(xyz);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}