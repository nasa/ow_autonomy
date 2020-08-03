// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// GroundControl ROS node.

// ow_autonomy
#include "subscriber.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

// C++
#include <thread>

static ros::Publisher pubNewTarget;
static ros::Publisher pubUseOnboardTarget;
static geometry_msgs::Point xyz;
static geometry_msgs::Point xyzOnboard;
static ros::Duration commsDelay;
static ros::Duration decisionDelay;
static bool useOnboard;


// placeholder. Just receives a string message for now. Eventually needs 
// receive an image. 
void imageCallback(const std_msgs::String::ConstPtr& image)
{
  // Simulate time delay for arrival of messages to Lander.
  commsDelay.sleep();
  ROS_INFO("GroundControl: Received image [%s].", image->data.c_str());
}

void targetCallback(const geometry_msgs::Point::ConstPtr& point)
{
  std::string x = std::to_string(point->x);
  std::string y = std::to_string(point->y);
  std::string z = std::to_string (point->z);
  xyzOnboard.x = point->x;
  xyzOnboard.y = point->y;
  xyzOnboard.z = point->z;
  ROS_INFO("GroundControl: Received target, [x = %s, y = %s, z = %s].", 
    x.c_str(), y.c_str(), z.c_str());
}

void requestCallback(const std_msgs::String::ConstPtr& cmd)
{
  ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
  ROS_INFO("GroundControl: Making Decision.");
  // Simulate time duration for decision making on ground. 
  decisionDelay.sleep();
  ROS_INFO("GroundControl: Sending Decision.");
  if (useOnboard == false) {
    // Simulate time delay for arrival of message to Lander.
    commsDelay.sleep();
    pubNewTarget.publish(xyz);
  }
  else {
    // Simulate time delay for arrival of message to Lander.
    commsDelay.sleep();
    pubUseOnboardTarget.publish(xyzOnboard);
  }
}

int main(int argc, char **argv)
{
  // Initialize GroundControl ROS node.
  ros::init(argc, argv, "GroundControl");

  ros::NodeHandle nHandle;

  // New target from ground. (Set in autonomy_node.launch)
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
  //onboardTargetMessage.data = "Use onboard Target.";

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
  pubNewTarget = nHandle.advertise<geometry_msgs::Point>
      ("/GroundControl/fwd_link", 3, true);
  
  // Publish to /GroundControl/use_onboard_target when decided to use 
  // target obtained by Lander. 
  pubUseOnboardTarget = nHandle.advertise<geometry_msgs::Point>
      ("GroundControl/use_onboard_target", 3, true);

  /////////////////////////// Initialize Subscribers ////////////////////////////////
  // Subscribe to /GroundControl/downlink topic.
  ros::Subscriber subTarget = nHandle.subscribe("/GroundControl/downlink", 3, targetCallback);

  // Subscribe to /GroundControl/request topic to respond to requests from lander.
  ros::Subscriber subRequest = nHandle.subscribe("/GroundControl/request", 3, requestCallback);

  // Subscribe to /GroundControl/downlinkImage topic.
  ros::Subscriber subImage = nHandle.subscribe("/GroundControl/downlinkImage", 3, imageCallback);;
  
  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}