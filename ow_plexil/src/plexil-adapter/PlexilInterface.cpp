// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "PlexilInterface.h"
#include "subscriber.h"
#include <map>

using std::string;
using std::map;
using std::make_unique;

///////////////////////// Utilities /////////////////////////

static double normalize_degrees (double angle)
{
  static double pi = R2D * M_PI;
  static double tau = pi * 2.0;
  double x = fmod(angle + pi, tau);
  if (x < 0) x += tau;
  return x - pi;
}


///////////////////////// Action Goal Status Support /////////////////////////

// Dummy operation ID that signifies idle lander operation.
#define IDLE_ID (-1)

// Duplication of actionlib_msgs/GoalStatus.h with the addition of a
// NOGOAL status for when the action is not running.
// NOTE: the last three are not yet supported in the simulator.
//
enum ActionGoalStatus {
  NOGOAL = -1,
  PENDING = 0,
  ACTIVE = 1,
  PREEMPTED = 2,
  SUCCEEDED = 3,
  ABORTED = 4,
  REJECTED = 5,
  PREEMPTING = 6,
  RECALLING = 7,
  RECALLED = 8,
  LOST = 9
};

static map<string, int> ActionGoalStatusMap { };

static string status_topic (const string& action)
{
  return string("/") + action + "/status";
}


///////////////////////////////// Class Interface ///////////////////////////////

bool PlexilInterface::anglesEquivalent (double deg1, double deg2,
                                        double tolerance)
{
  return fabs(normalize_degrees(deg1 - deg2)) <= tolerance;
}

PlexilInterface::PlexilInterface ()
  : m_commandStatusCallback (nullptr),
    m_genericNodeHandle (make_unique<ros::NodeHandle>())
{ }

PlexilInterface::~PlexilInterface ()
{
  // No memory leak, since memory wasn't allocated.
  m_commandStatusCallback = nullptr;
}

void PlexilInterface::actionGoalStatusCallback
(const actionlib_msgs::GoalStatusArray::ConstPtr& msg, const string& action_name)

// Update ActionGoalStatusMap of action action_name with the status
// from first goal in GoalStatusArray msg. This is based on the
// assumption that no action will have more than one goal in our
// system.
{
  if (msg->status_list.size() == 0) {
    ActionGoalStatusMap[action_name] = NOGOAL;
    ROS_WARN_ONCE ("---- There was no goal");
    publish ("ActionGoalStatus", static_cast<int>(NOGOAL), action_name);
  }
  else { // if (msg->status_list.size() >= 1) {
    ROS_INFO("---- Action status: ", msg->status_list[0].status);
    ActionGoalStatus status =
      static_cast<ActionGoalStatus>(msg->status_list[0].status);
    ROS_WARN_ONCE ("---- There was a goal");
    if (status == SUCCEEDED) ROS_WARN_ONCE ("---- %s SUCCEEDED", action_name.c_str());
    if (status == ABORTED) ROS_WARN_ONCE ("---- %s ABORTED", action_name.c_str());
    if (status == ACTIVE) ROS_WARN_ONCE ("---- %s ACTIVE", action_name.c_str());
    if (status == PENDING) ROS_WARN_ONCE ("---- %s PENDING", action_name.c_str());
    ActionGoalStatusMap[action_name] = status;
    publish ("ActionGoalStatus", static_cast<int>(status), action_name);
  }
}

void PlexilInterface::subscribeToActionStatus (const string& action)
{
  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
      (status_topic(action), 3,
       boost::bind(&PlexilInterface::actionGoalStatusCallback,
                   this, _1, action))));
}

int PlexilInterface::actionGoalStatus (const string& action_name) const
{
  return ActionGoalStatusMap[action_name];
}

bool PlexilInterface::isLanderOperation (const string& name) const
{
  return m_runningOperations.find(name) != m_runningOperations.end();
}

bool PlexilInterface::markOperationRunning (const string& name, int id)
{
  if (m_runningOperations.at (name) != IDLE_ID) {
    ROS_WARN ("%s already running, ignoring duplicate request.", name.c_str());
    return false;
  }
  m_runningOperations.at (name) = id;
  publish ("Running", true, name);
  return true;
}

void PlexilInterface::markOperationFinished (const string& name, int id)
{
  if (! m_runningOperations.at (name) == IDLE_ID) {
    ROS_WARN ("%s was not running. Should never happen.", name.c_str());
  }
  m_runningOperations.at (name) = IDLE_ID;
  publish ("Running", false, name);
  publish ("Finished", true, name);
  if (id != IDLE_ID) {
    if (m_commandStatusCallback) m_commandStatusCallback (id, true);
    else ROS_ERROR ("markOperationFinished: m_commandStatusCallback was null!");
  }
  else ROS_WARN ("markOperationFinished: %s was not running.", name.c_str());
}

bool PlexilInterface::running (const string& name) const
{
  if (isLanderOperation (name)) {
    return operationRunning (name);
  }
  else {
    ROS_ERROR("PlexilInterface::running: unsupported operation: %s", name.c_str());
    return false;
  }
}

bool PlexilInterface::operationRunning (const string& name) const
{
  // Note: check in caller guarantees 'at' to return a valid value.
  return m_runningOperations.at (name) != IDLE_ID;
}

void PlexilInterface::setCommandStatusCallback (void (*callback) (int, bool))
{
  m_commandStatusCallback = callback;
}

void PlexilInterface::registerLanderOperation (const string& name)
{
  m_runningOperations[name] = IDLE_ID;
}
