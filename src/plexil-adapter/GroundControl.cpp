// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.


// ow_autonomy
#include "subscriber.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

// C++
#include <thread>

// ros::NodeHandle* nHandlePtr;
ros::Publisher* pubLinkPtr;
geometry_msgs::Point* xyzPtr;

// ros::Publisher pubLink = nHandlePtr->advertise<geometry_msgs::Point>
//     ("/GroundControl/fwd_link", 3, true);

void commandsCallback(const std_msgs::String::ConstPtr& cmd)
{
  ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
}

void requestCallback(const std_msgs::String::ConstPtr& cmd)
{
  ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
  // Simulate time delay for arrival of message to Lander.
  // 30 seconds.
  std::this_thread::sleep_for(std::chrono::seconds(30));
  pubLinkPtr->publish(*xyzPtr);
}


int main(int argc, char **argv)
{
  // Initialize GroundControl ROS node.
  ros::init(argc, argv, "GroundControl");

  ros::NodeHandle nHandle;
  //nHandlePtr = &nHandle;

  geometry_msgs::Point xyz;
  xyz.x = 6;
  xyz.y = 7;
  xyz.z = 0;
  xyzPtr = &xyz;

  ros::Publisher pubLink = nHandle.advertise<geometry_msgs::Point>
      ("/GroundControl/fwd_link", 3, true);

  pubLinkPtr = &pubLink;

  // subscribe to /GroundControl/message topic
  ros::Subscriber sub = nHandle.subscribe("/GroundControl/message", 3, commandsCallback);

  // subscribe to /GroundControl/request topic to respond to requests from lander.
  ros::Subscriber subRequest = nHandle.subscribe("/GroundControl/request", 3, requestCallback);
  // ros::Subscriber subRequest = nHandle.subscribe("/GroundControl/request", 3, requestCallback =
  //   [] (const std_msgs::String::ConstPtr& req) {
  //     std::this_thread::sleep_for(std::chrono::seconds(30));
  //     ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
  //     pubLink.publish(xyz);
  //   });

  // geometry_msgs::Point xyz;

  // simulate communication delay from earth
  //sleep(10);
  // 50 minutes.
  // std::this_thread::sleep_for(std::chrono::seconds(3000));
  // 1 hour.
  // std::this_thread::sleep_for(std::chrono::seconds(3600));
  // 2 hours.
  // std::this_thread::sleep_for (std::chrono::seconds(7200));
  // 2 minutes.
  // std::this_thread::sleep_for(std::chrono::seconds(120));
  // xyz.x = 6;
  // xyz.y = 7;
  // xyz.z = 0;

  // pubLink.publish(xyz);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}