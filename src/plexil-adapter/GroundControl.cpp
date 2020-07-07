// ow_autonomy
#include "subscriber.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"

void commandsCallback(const std_msgs::String::ConstPtr& cmd)
{
  ROS_INFO("GroundControl: Received message, [%s].", cmd->data.c_str());
  //publish ("MessageReceived", "I heard: [%s]", cmd->data.c_str());
  //publish ("MessageReceived", cmd->data.c_str());
}

int main(int argc, char **argv)
{
  // Initialize GroundControl ROS node.
  ros::init(argc, argv, "GroundControl");

  ros::NodeHandle nHandle;

  // subscribe to /GroundControl/message topic
  ros::Subscriber sub = nHandle.subscribe("/GroundControl/message", 3, commandsCallback);
  
  ros::Publisher pub = nHandle.advertise<std_msgs::Float64MultiArray>
    ("/GroundControl/fwd_link", 3, true);

  std_msgs::Float64MultiArray xyz;

  xyz.data.clear();
  xyz.data.push_back(6);
  xyz.data.push_back(7);
  xyz.data.push_back(0);

  pub.publish(xyz);


  //ros::spin();

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}