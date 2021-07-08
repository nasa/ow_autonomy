// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ROS node wrapping a PLEXIL executive.

// ROS
#include <ros/ros.h>
#include <ros/package.h>
//Planner
#include "PlexilPlanner.h"

int main(int argc, char* argv[])
{
  // Initializations
  ros::init(argc, argv, "plexil_node");
  PlexilPlanner planner; 
  planner.initialize(); //initialize pubs, subs, etc
  planner.start(); //begin control loop
  ros::spin();
  return 0;  // We never actually get here!
}
