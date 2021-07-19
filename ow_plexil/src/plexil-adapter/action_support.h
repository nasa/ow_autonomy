// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Action_Support_H
#define Action_Support_H

// Generic ROS Action support

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <functional>

const auto ActionServerTimeout = 10.0;  // seconds

template<typename T>
using t_action_done_cb =
  std::function<void (const actionlib::SimpleClientGoalState&,
                      const T& result_ignored)>;

template<typename T>
t_action_done_cb<T> default_action_done_cb (const std::string& operation_name)
{
  return [&](const actionlib::SimpleClientGoalState& state,
             const T& result_ignored) {
    ROS_INFO ("%s finished in state %s", operation_name.c_str(),
              state.toString().c_str());
  };
}

template<typename T>
void action_feedback_cb (const T& feedback_unused)
{
}

std::function<void()> active_cb (const std::string& operation_name);

#endif
