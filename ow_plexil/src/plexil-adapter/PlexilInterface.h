// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Interface_H
#define Plexil_Interface_H

// Interfacing support between the PLEXIL executive and various Ocean World
// simulators and testbeds.

#include "action_support.h"

class PlexilInterface
{
 public:
  PlexilInterface ();
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
  template <class ActionClient, class Goal, class ResultPtr, class FeedbackPtr>
    void runAction (const std::string& opname,
                    std::unique_ptr<ActionClient>&,
                    const Goal&,
                    int id,
                    t_action_done_cb<ResultPtr> done_cb =
                      default_action_done_cb<ResultPtr>);

  // Callback function in PLEXIL adapter for success/failure of given command.
  // This didn't work, so using conventional pointer.
  //  std::unique_ptr<void*(int,bool)> m_commandStatusCallback;
  void* (int,bool) m_commandStatusCallback;
};

#endif
