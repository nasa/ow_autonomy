// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// OW autonomy ROS node.

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

  // wait for the first proper clock message before running the plan
  ros::Rate warmup_rate(0.1);
  ros::Time begin = ros::Time::now();
  while (ros::Time::now() - begin == ros::Duration(0.0))
  {
    ros::spinOnce();
    warmup_rate.sleep();
  }
  
  // Run the specified plan
  if (argc == 2) {
    ROS_INFO ("Running plan %s", argv[1]);
    OwExecutive::instance()->runPlan (argv[1]); // asynchronous
  }
  else {
    ROS_ERROR("autonomy_node got %i args, expected 2", argc);
    return 1;
  }

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
	std::string input; 
	// loops until terminated by user, prompts them to enter any additional plans after the
	// previous plan has been run to completion.
  while (ros::ok()) {
    ros::spinOnce();
		if(OwExecutive::instance()->getPlanState()){ // checks to see if previous plan finished
			std::cout << "\nEnter any additional plan to be run: ";
			std::getline(std::cin, input); 
			// checks for empty input and trys to run the plan, if the plan has an error
			// or doesnt exist then error message prints before looping again.
			if(input != "" && OwExecutive::instance()->runPlan(input.c_str())){
					ROS_INFO ("Running plan %s", input.c_str());
    			rate.sleep();
			}
		}
		else{
    	rate.sleep();
		}
  }

  return 0;  // We never actually get here!
}
