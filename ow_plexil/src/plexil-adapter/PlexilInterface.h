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
  bool operationRunning (const std::string& name) const;
  void registerLanderOperation (const std::string& name);

  // Map from operation name to its instance ID if it is running, or to IDLE_ID
  // otherwise.  The keys of this map do not change after initialization, and
  // comprise all the valid lander operation names.
  std::map<std::string, int> m_runningOperations;

  template <class ActionClient, class Goal, class ResultPtr, class FeedbackPtr>
    void runAction (const std::string& opname,
                    std::unique_ptr<ActionClient>& ac,
                    const Goal& goal,
                    int id)
                    //                   t_action_done_cb<ResultPtr> done_cb =
                    //                      default_action_done_cb<ResultPtr> (opname))
  {
    if (ac) {
      ROS_INFO ("Sending goal to action %s", opname.c_str());
      ac->sendGoal (goal,
                    //                    done_cb,
                    default_action_done_cb<ResultPtr> (opname),
                    //                    [&](){ active_cb (opname); },
                    active_cb (opname),
                    action_feedback_cb<FeedbackPtr>);
      ROS_INFO ("Sent goal to action %s", opname.c_str());
    }
    else {
      ROS_ERROR ("%s action client was null!", opname.c_str());
      return;
    }

    // Wait indefinitely for the action to complete.
    bool finished_before_timeout = ac->waitForResult (ros::Duration (0));
    markOperationFinished (opname, id);
  }

 private:
  // Callback function in PLEXIL adapter for success/failure of given command.
  // This didn't work, so using conventional pointer.
  //  std::unique_ptr<void*(int,bool)> m_commandStatusCallback;
  void (*m_commandStatusCallback)(int,bool);
};

#endif
