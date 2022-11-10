// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ROS
#include <ros/ros.h>
#include <ros/package.h>
// Terminal plan selection class
#include "TerminalPlanSelection.h"

int main(int argc, char* argv[])
{
  // Initializations
  ros::init(argc, argv, "terminal_selection_node");

  bool plan_given = false;
  std::string plan(argv[1]);
  //checking if there is a plan given
  if(argc == 2 && plan.compare("None") != 0) {
    plan_given = true;
  }
  TerminalPlanSelection terminal_interface;
  terminal_interface.initialize();
  terminal_interface.start(plan_given);
  ros::spin();
  return 0;
}
