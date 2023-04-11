// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "PlexilInterface.h"
#include "subscriber.h"
#include <map>

using std::string;
using std::map;
using std::make_unique;

// Dummy operation ID that signifies idle lander operation.
#define IDLE_ID (-1)

///////////////////////// Action Goal Status Support /////////////////////////

// Duplication of actionlib_msgs/GoalStatus.h with the addition of a
// NOGOAL status for when the action is not running.
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

///////////////////////////////// Class Interface ///////////////////////////////

PlexilInterface::PlexilInterface ()
  : m_commandStatusCallback (nullptr)
{ }

PlexilInterface::~PlexilInterface ()
{
  // No memory leak, since memory wasn't allocated.
  m_commandStatusCallback = nullptr;
}

void PlexilInterface::initialize()
{
  m_genericNodeHandle = make_unique<ros::NodeHandle>();
}


void PlexilInterface::actionGoalStatusCallback
(const actionlib_msgs::GoalStatusArray::ConstPtr& msg, const string action_name)

// Update ActionGoalStatusMap of action action_name with the status
// from first goal in GoalStatusArray msg. This is based on the
// assumption that no action will have more than one goal in our
// system.
{
  if (msg->status_list.size() == 0) {
    ActionGoalStatusMap[action_name] = NOGOAL;
  }
  else if (msg->status_list.size() == 1) {
    ActionGoalStatusMap[action_name] = msg->status_list[0].status;
  }
  else {
    ROS_ERROR ("%s had more than one goal: this should never happen.",
               action_name.c_str());
  }
}

void PlexilInterface::subscribeToActionStatus (const string& topic, const string& operation)
{
  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
      (topic, 3,
       boost::bind(&PlexilInterface::actionGoalStatusCallback,
                   this, _1, operation))));
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
