// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Interface_H
#define Plexil_Interface_H

// Interfacing support between the PLEXIL executive and various Ocean
// World simulators and testbeds.

#include "action_support.h"
#include <cmath>  // for M_PI, fabs, fmod

// Degree/Radian conversion used in subclasses
constexpr double D2R = M_PI / 180.0 ;
constexpr double R2D = 180.0 / M_PI ;

class PlexilInterface
{
 public:
  static bool anglesEquivalent (double deg1, double deg2, double tolerance);
  
  PlexilInterface ();
  virtual ~PlexilInterface ();
  PlexilInterface (const PlexilInterface&) = delete;
  PlexilInterface& operator= (const PlexilInterface&) = delete;

  // Is the given operation (as named in the subclass) running?
  bool running (const std::string& name) const;

  // Is the given operation name valid?
  bool isLanderOperation (const std::string& name) const;

  bool markOperationRunning (const std::string& name, int id);
  void markOperationFinished (const std::string& name, int id);

  // Command feedback
  void setCommandStatusCallback (void (*callback) (int, bool));
  int actionGoalStatus (const std::string& action_name) const;

 protected:
  bool operationRunning (const std::string& name) const;
  void registerLanderOperation (const std::string& name);

  // Map from operation name to its instance ID if it is running, or to IDLE_ID
  // otherwise.  The keys of this map do not change after initialization, and
  // comprise all the valid lander operation names.
  std::map<std::string, int> m_runningOperations;

  template <class Client, class Goal, class ResultPtr, class FeedbackPtr>
  void runNullaryAction (int id, const std::string& name,
                         std::unique_ptr<Client>& ac)
  {
    Goal goal;
    std::string opname = name;

    runAction<Client, Goal, ResultPtr, FeedbackPtr>
      (opname, ac, goal, id,
       default_action_active_cb (opname),
       default_action_feedback_cb<FeedbackPtr> (opname),
       default_action_done_cb<ResultPtr> (opname));
  }

  template<typename T>
  void connectActionServer (std::unique_ptr<actionlib::SimpleActionClient<T> >& c,
			    const std::string& action)
  {
    if (! c->waitForServer(ros::Duration(10))) {
      // Connection typically happens very fast, so this timeout is
      // only to prevent indefinite wait when an action server is down
      // for some reason.
      ROS_ERROR ("%s action server did not connect!", action.c_str());
    }
    subscribeToActionStatus (action);
  }

  void subscribeToActionStatus (const std::string& action);

  template <class ActionClient, class Goal, class ResultPtr, class FeedbackPtr>
    void runAction (const std::string& opname,
                    std::unique_ptr<ActionClient>& ac,
                    const Goal& goal,
                    int id,
                    t_action_active_cb active_cb,
                    t_action_feedback_cb<FeedbackPtr> feedback_cb,
                    t_action_done_cb<ResultPtr> done_cb)
  {
    if (ac) {
      ROS_DEBUG ("Sending goal to action %s", opname.c_str());
      ac->sendGoal (goal, done_cb, active_cb, feedback_cb);
      ROS_DEBUG ("Sent goal to action %s", opname.c_str());
    }
    else {
      ROS_ERROR ("%s action client was null!", opname.c_str());
      return;
    }

    // Wait indefinitely for the action to complete.
    bool finished_before_timeout = ac->waitForResult (ros::Duration (0));
    markOperationFinished (opname, id);
  }

  // Node handle reused much.
  std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;

  // Subscribers: generic container because the subscribers are not
  // referenced; only their callback functions are of use.
  std::vector<std::unique_ptr<ros::Subscriber>> m_subscribers;

 private:
  // Callback function in PLEXIL adapter for success/failure of given command.
  std::function<void(int, bool)> m_commandStatusCallback;
  void actionGoalStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr&,
                                 const std::string);

};

#endif
