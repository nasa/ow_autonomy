// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter as common to all testbeds.

// ow_plexil
#include "PlexilAdapter.h"
#include "adapter_support.h"
#include "subscriber.h"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// PLEXIL API
#include <AdapterConfiguration.hh>
#include <AdapterFactory.hh>
#include <AdapterExecInterface.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
#include <Expression.hh>
using namespace PLEXIL;

void PlexilAdapter::propagateValueChange (const State& state,
                                          const std::vector<Value>& vals) const
{
  if (! isStateSubscribed (state)) {
    debugMsg("PlexilAdapter:propagateValueChange", " ignoring " << state);
    return;
  }
  debugMsg("PlexilAdapter:propagateValueChange", " sending " << state);
  getInterface().handleValueChange (state, vals.front());
  getInterface().notifyOfExternalEvent();
}

bool PlexilAdapter::isStateSubscribed(const State& state) const
{
  return m_subscribedStates.find(state) != m_subscribedStates.end();
}

PlexilAdapter::PlexilAdapter(AdapterExecInterface& execInterface,
                     const pugi::xml_node& configXml)
  : InterfaceAdapter(execInterface, configXml)
{
  debugMsg("PlexilAdapter", " created.");
}

bool PlexilAdapter::initialize()
{
  g_configuration->defaultRegisterAdapter(this);
  g_configuration->registerCommandHandler("log_info", log_info);
  g_configuration->registerCommandHandler("log_warning", log_warning);
  g_configuration->registerCommandHandler("log_error", log_error);
  g_configuration->registerCommandHandler("log_debug", log_debug);
  setSubscriber (receiveBool);
  setSubscriber (receiveInt);
  setSubscriber (receiveString);
  setSubscriber (receiveDouble);
  setSubscriber (receiveBoolFromString);
  setSubscriber (receiveDoubleFromInt);
  setSubscriber (receiveDoubleVector);
  g_adapter = this;
  debugMsg("PlexilAdapter", " initialized.");
  return true;
}

bool PlexilAdapter::start()
{
  debugMsg("PlexilAdapter", " started.");
  return true;
}

void PlexilAdapter::stop()
{
  debugMsg("PlexilAdapter", " stopped.");
}

/*
bool PlexilAdapter::reset()
{
  debugMsg("PlexilAdapter", " reset.");
  return true;
}

bool PlexilAdapter::shutdown()
{
  debugMsg("PlexilAdapter", " shut down.");
  return true;
}
*/

void PlexilAdapter::invokeAbort(Command *cmd)
{
  ROS_ERROR("Cannot abort command %s, not implemented, ignoring.",
            cmd->getName().c_str());
}

void PlexilAdapter::subscribe(const State& state)
{
  debugMsg("PlexilAdapter:subscribe", " to state " << state.name());
  m_subscribedStates.insert(state);
}


void PlexilAdapter::unsubscribe (const State& state)
{
  debugMsg("PlexilAdapter:unsubscribe", " from state " << state.name());
  m_subscribedStates.erase(state);
}
