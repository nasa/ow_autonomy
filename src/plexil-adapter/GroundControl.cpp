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

static ros::Publisher pubLink;
static ros::Publisher pubUseOnboardTarget;
static geometry_msgs::Point xyz;
static std_msgs::String onboardTargetMessage;
static ros::Duration commsDelay;
static ros::Duration decisionDelay;
static bool useOnboard;

void downlinkCallback(const geometry_msgs::Point::ConstPtr& point)
{
  // Simulate time delay for arrival of message to Lander.
  commsDelay.sleep();
  std::string x = std::to_string(point->x);
  std::string y = std::to_string(point->y);
  std::string z = std::to_string (point->z);
  ROS_INFO("GroundControl: Received target, [x = %s, y = %s, z = %s].", 
    x.c_str(), y.c_str(), z.c_str());
}

void requestCallback(const std_msgs::String::ConstPtr& cmd)
{
  commsDelay.sleep();
  ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
  ROS_INFO("GroundControl: Making Decision.");
  // Simulate time delay for decision making on ground. 
  decisionDelay.sleep();
  ROS_INFO("GroundControl: Sending Decision.");
  if (useOnboard == false) {
    // Simulate time delay for arrival of message to Lander.
    commsDelay.sleep();
    pubLink.publish(xyz);
  }
  else {
    // Simulate time delay for arrival of message to Lander.
    commsDelay.sleep();
    pubUseOnboardTarget.publish(onboardTargetMessage);
  }
}


int main(int argc, char **argv)
{
  // Initialize GroundControl ROS node.
  ros::init(argc, argv, "GroundControl");

  ros::NodeHandle nHandle;

  // New target from ground.

  double xCoordinate;
  double yCoordinate;
  double zCoordinate;

  nHandle.getParam("/x_coordinate", xCoordinate);
  nHandle.getParam("/y_coordinate", yCoordinate);
  nHandle.getParam("/z_coordinate", zCoordinate);

  xyz.x = xCoordinate;
  xyz.y = yCoordinate;
  xyz.z = zCoordinate;

  // For decision to use onboard target.
  onboardTargetMessage.data = "Use onboard Target.";

  // For Communications Delay and Decision Duration, set in autonomy_node.launch
  int delay;
  int dDuration;
  nHandle.getParam("/use_onboard_target", useOnboard);
  nHandle.getParam("/communications_delay", delay);
  nHandle.getParam("/decision_duration", dDuration);

  commsDelay = ros::Duration(delay, 0);
  decisionDelay = ros::Duration(dDuration, 0);

  /////////////////////////// Initialize Publishers ////////////////////////////////
  // Publish to /GroundControl/fwd_link when decided to update trench 
  // target to target decided on ground.
  pubLink = nHandle.advertise<geometry_msgs::Point>
      ("/GroundControl/fwd_link", 3, true);
  
  // Publish to /GroundControl/use_onboard_target when decided to use 
  // target obtained by Lander. 
  pubUseOnboardTarget = nHandle.advertise<std_msgs::String>
      ("GroundControl/use_onboard_target", 3, true);

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