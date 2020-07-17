// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// A few options for adjusting your simulation with this node. 
// If you want to use ROS time instead of system time, open 
// ow_autonomy/launch/autonomy_node.launch and set the "/use_sim_time"
// parameter to true. If you want to run a full real time simulation 
// (approx. 2 hr 40 min instead of 1 min 36 sec for communications),
// set the "/real_time_sim" parameter to true. If you want the "GroundControl"
// node to use the target set from ground instead of the target aquired 
// onboard the lander, set the "/use_onboard_target" parameter to false.

// ow_autonomy
#include "subscriber.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

// C++
#include <thread>

ros::Publisher* pubLinkPtr;
ros::Publisher* pubUseOnboardTargetPtr;
geometry_msgs::Point* xyzPtr;
std_msgs::String* onboardTargetMessagePtr;
ros::Duration* commsDelayPtr;
ros::Duration* decisionDelayPtr;
bool* useOnboardPtr;

void downlinkCallback(const geometry_msgs::Point::ConstPtr& point)
{
  // Simulate time delay for arrival of message to Lander.
  commsDelayPtr->sleep();
  std::string x = std::to_string(point->x);
  std::string y = std::to_string(point->y);
  std::string z = std::to_string (point->z);
  ROS_INFO("GroundControl: Received target, [x = %s, y = %s, z = %s].", 
    x.c_str(), y.c_str(), z.c_str());
}

void requestCallback(const std_msgs::String::ConstPtr& cmd)
{
  ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
  ROS_INFO("GroundControl: Making Decision.");
  // Simulate time delay for decision making on ground. 
  decisionDelayPtr->sleep();
  ROS_INFO("GroundControl: Sending Decision.");
  if (*useOnboardPtr == false) {
    // Simulate time delay for arrival of message to Lander.
    commsDelayPtr->sleep();
    pubLinkPtr->publish(*xyzPtr);
  }
  else {
    // Simulate time delay for arrival of message to Lander.
    commsDelayPtr->sleep();
    pubUseOnboardTargetPtr->publish(*onboardTargetMessagePtr);
  }
}


int main(int argc, char **argv)
{
  // Initialize GroundControl ROS node.
  ros::init(argc, argv, "GroundControl");

  ros::NodeHandle nHandle;

  // New target from ground.
  geometry_msgs::Point xyz;
  xyz.x = 6;
  xyz.y = 7;
  xyz.z = 0;
  xyzPtr = &xyz;

  // For decision to use onboard target.
  std_msgs::String onboardTargetMessage;
  onboardTargetMessage.data = "Use onboard Target.";
  onboardTargetMessagePtr = &onboardTargetMessage;

  // Decision to use onboard target or target from ground, and decision
  // to use full real time communications simulation or not. 
  // 2 hr 40 min or 1 min 36 sec simulation. Decisions are set in 
  // ow_autonomy/launch/autonomy_node.launch
  bool useOnboard;
  bool realTimeSim;
  nHandle.getParam("/use_onboard_target", useOnboard);
  nHandle.getParam("/real_time_sim", realTimeSim);
  useOnboardPtr = &useOnboard;

  ros::Duration commsDelay;
  ros::Duration decisionDelay;

  if (realTimeSim == true) {
    // 50 minutes delay.
    commsDelay = ros::Duration(3000, 0);
    // 1 hour delay.
    decisionDelay = ros::Duration(3600, 0);
  }
  else {
    // 30 seconds delay.
    commsDelay = ros::Duration(30, 0);
    // 36 seconds delay.
    decisionDelay = ros::Duration(36, 0);
  }

  commsDelayPtr = &commsDelay;
  decisionDelayPtr = &decisionDelay;

  /////////////////////////// Initialize Publishers ////////////////////////////////
  // Publish to /GroundControl/fwd_link when decided to update trench 
  // target to target decided on ground.
  ros::Publisher pubLink = nHandle.advertise<geometry_msgs::Point>
      ("/GroundControl/fwd_link", 3, true);
  
  // Publish to /GroundControl/use_onboard_target when decided to use 
  // target obtained by Lander. 
  ros::Publisher pubUseOnboardTarget = nHandle.advertise<std_msgs::String>
      ("GroundControl/use_onboard_target", 3, true);

  pubLinkPtr = &pubLink;
  pubUseOnboardTargetPtr = &pubUseOnboardTarget;

  /////////////////////////// Initialize Subscribers ////////////////////////////////
  // Subscribe to /GroundControl/message topic.
  ros::Subscriber sub = nHandle.subscribe("/GroundControl/downlink", 3, downlinkCallback);

  // Subscribe to /GroundControl/request topic to respond to requests from lander.
  ros::Subscriber subRequest = nHandle.subscribe("/GroundControl/request", 3, requestCallback);
  
  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}