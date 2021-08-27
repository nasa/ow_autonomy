// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Action_Support_H
#define Action_Support_H

// Generic ROS Action support.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <functional>

// Time that the Plexil node waits for each action server to be detected.  It
// typically happens very fast, so this timeout is only to prevent indifinite
// wait when an action server is down for some reason.
//
const auto ACTION_SERVER_TIMEOUT_SECS = 10.0;

// The following action callbacks are essentially stubs that do nothing more
// than print a short status.  They are used as defaults; any action invocation
// can substitute another callback.

using t_action_active_cb = std::function<void ()>;

t_action_active_cb default_action_active_cb (const std::string& operation_name);

template<typename T>
using t_action_feedback_cb = std::function<void (const T& feedback)>;

template<typename T>
t_action_feedback_cb<T>
default_action_feedback_cb (const std::string& operation_name_unused)
{
  // Since feedback is verbose, do nothing by default.
  return [&] (const T& feedback_unused) { };
}

template<typename T>
using t_action_done_cb =
  std::function<void (const actionlib::SimpleClientGoalState&,
                      const T& result_ignored)>;

template<typename T>
t_action_done_cb<T> default_action_done_cb (const std::string& operation_name)
{
  return [&] (const actionlib::SimpleClientGoalState& state,
              const T& result_ignored) {
    ROS_INFO ("%s finished in state %s", operation_name.c_str(),
              state.toString().c_str());
  };
}


#endif
