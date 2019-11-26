// OW autonomy ROS node.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include "OwExecutive.h"
#include "OwInterface.h"

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "autonomy_node");

  if (! OwExecutive::instance()->initialize()) {
    ROS_ERROR("Could not initialize OW executive, shutting down.");
    return 1;
  }

  OwInterface::instance()->initialize();

  // Run the specified plan

  if (argc == 2) {
    OwExecutive::instance()->runPlan (argv[1]);
  }
  else {
    ROS_ERROR("autonomy got %i args, expected 2", argc);
    return 1;
  }

  // ROS Loop

  // NOTE: Why does this run concurrently with plan?

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  // Is this ever reached?
  ROS_INFO("Autonomy node exiting normally.");
  return 0;
}
