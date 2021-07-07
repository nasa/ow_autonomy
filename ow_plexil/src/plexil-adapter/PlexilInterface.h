// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Interface_H
#define Plexil_Interface_H

// Interfacing support between the PLEXIL executive and various Ocean World
// simulators and testbeds.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// ROS Action support

const auto ActionServerTimeout = 10.0;  // seconds

template<int OpIndex, typename T>
using t_action_done_cb = void (*)(const actionlib::SimpleClientGoalState&,
                                  const T& result_ignored);

template<int OpIndex, typename T>
void default_action_done_cb (const actionlib::SimpleClientGoalState& state,
                             const T& result_ignored);

template<typename T>
void action_feedback_cb (const T& feedback);

template<int OpIndex>
void active_cb ();

template<int OpIndex, typename T>
void default_action_done_cb (const actionlib::SimpleClientGoalState& state,
                             const T& result_ignored);

class PlexilInterface
{
 public:
  PlexilInterface () = default;
  ~PlexilInterface () = default;
  PlexilInterface (const PlexilInterfac&) = delete;
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
  std::unique_ptr<void*(int,bool)> m_CommandStatusCallback;

  // Map from operation name to its instance ID if it is running, or to IDLE_ID
  // otherwise.  This map stays the same size after initialization, thus storing
  // all the valid lander operation names.
  std::map<std::string, int> m_runningOperations;
};

#endif
