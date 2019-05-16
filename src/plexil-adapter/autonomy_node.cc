// Autonomy ROS node, which is also a PLEXIL application.

// Under construction!

#include <ros/ros.h>
#include "ExecApplication.hh"

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "autonomy_node");

  //  ros::NodeHandle nh;

  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
