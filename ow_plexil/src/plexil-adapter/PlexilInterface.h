// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Interface_H
#define Plexil_Interface_H

// Interfacing support between the PLEXIL executive and various Ocean World
// simulators and testbeds.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

//// Generic ROS Action support ////

const auto ActionServerTimeout = 10.0;  // seconds

// The following callbacks do little or nothing besides issuing information.

template<typename T>
using t_action_done_cb = void (*)(const actionlib::SimpleClientGoalState&,
                                  const T& result_ignored,
                                  const std::string& operation_name);

template<typename T>
void default_action_done_cb (const actionlib::SimpleClientGoalState& state,
                             const T& result_ignored,
                             const std::string& operation_name);

template<typename T>
void action_feedback_cb (const T& feedback);

void active_cb (const std::string& operation_name);

template<typename T>
void default_action_done_cb (const actionlib::SimpleClientGoalState& state,
                             const T& result_ignored,
                             const std::string& operation_name);

class PlexilInterface
{
 public:
  PlexilInterface () = default;
  virtual ~PlexilInterface () = 0;
  PlexilInterface (const PlexilInterface&) = delete;
  PlexilInterface& operator= (const PlexilInterface&) = delete;

  // Is the given operation (as named in the subclass) running?
  bool running (const std::string& name) const;

  // Is the given operation name valid?
  bool isLanderOperation (const std::string& name);

  bool markOperationRunning (const std::string& name, int id);
  bool markOperationFinished (const std::string& name, int id);

  // Command feedback
  void setCommandStatusCallback (void (*callback) (int, bool));

 protected:
  void registerLanderOperation (const std::string& name);

  // Map from operation name to its instance ID if it is running, or to IDLE_ID
  // otherwise.  The keys of this map do not change after initialization, and
  // comprise all the valid lander operation names.
  std::map<std::string, int> m_runningOperations;

 private:
  bool operationRunning (const std::string& name) const;
  template <int OpIndex, class ActionClient, class Goal,
            class ResultPtr, class FeedbackPtr>
  void runAction (const std::string& opname,
                  std::unique_ptr<ActionClient>&,
                  const Goal&,
                  int id,
                  t_action_done_cb<OpIndex,ResultPtr> done_cb =
                    default_action_done_cb<OpIndex, ResultPtr>);

  // Callback function in PLEXIL adapter for success/failure of given command.
  // This didn't work, so using conventional pointer.
  //  std::unique_ptr<void*(int,bool)> m_CommandStatusCallback;
  void* m_CommandStatusCallback (int,bool) = nullptr;
};

#endif
