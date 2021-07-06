#include "PlexilInterface.h"
#include "subscriber.h"

template<typename T>
void action_feedback_cb (const T& feedback)
{
}

template<int OpIndex>
void active_cb ()
{
  ROS_INFO ("%s started...", LanderOpNames[OpIndex].c_str());
}

template<int OpIndex, typename T>
void default_action_done_cb (const actionlib::SimpleClientGoalState& state,
                             const T& result_ignored)
{
  ROS_INFO ("%s finished in state %s", LanderOpNames[OpIndex].c_str(),
            state.toString().c_str());
}



bool PlexilInterface::isLanderOperation (const string& name)
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
  publish ("m_runningOperations", true, name);
  return true;
}

bool PlexilInterface::markOperationFinished (const string& name, int id)
{
  if (! m_runningOperations.at (name) == IDLE_ID) {
    ROS_WARN ("%s was not running. Should never happen.", name.c_str());
  }
  m_runningOperations.at (name) = IDLE_ID;
  publish ("m_runningOperations", false, name);
  publish ("Finished", true, name);
  if (id != IDLE_ID) m_CommandStatusCallback (id, true);

}

bool PlexilInterface::running (const string& name) const
{
  if (is_lander_operation (name)) return operationRunning (name);

  ROS_ERROR("PlexilInterface::running: unsupported operation: %s", name.c_str());
  return false;
}

bool PlexilInterface::operationRunning (const string& name) const
{
  // Note: check in caller guarantees 'at' to return a valid value.
  return m_runningOperations.at (name) != IDLE_ID;
}

void PlexilInterface::setCommandStatusCallback (void (*callback) (int, bool))
{
  m_CommandStatusCallback = callback;
}
