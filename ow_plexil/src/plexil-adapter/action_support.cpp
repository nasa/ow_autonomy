// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "action_support.h"

// This is the only non-template function so far, which motivated this .cpp file.

t_action_active_cb default_action_active_cb (const std::string& operation_name)
{
  return [&] () { ROS_INFO ("%s started...", operation_name.c_str()); };
}
