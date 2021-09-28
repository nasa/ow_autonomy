// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Plan executive ROS node for the OWLAT simulator.

// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "PlexilPlanSelection.h"
#include "OwExecutive.h"
#include "OwlatInterface.h"

int main(int argc, char* argv[])
{
  // Initializations
  ros::init(argc, argv, "owlat_exec_node");
  std::string initial_plan = "None";

  //checking if there is a plan given
  if(argc >= 2 && std::string(argv[1]).compare("None") != 0) {
    std::string plan(argv[1]);
    initial_plan = plan;
  }
  if (! OwExecutive::instance()->initialize ("owlat_plans/owlat-config.xml")) {
    ROS_ERROR("Could not initialize Plexil executive, shutting down.");
    return 1;
  }
  OwlatInterface::instance()->initialize();
  PlexilPlanSelection plan_selection;
  plan_selection.initialize(initial_plan); //initialize pubs, subs, etc
  plan_selection.start(); //begin control loop
  ros::spin();

  // Never reached, because killing the process is the only way to terminate
  // this program.
  return 0;
}
