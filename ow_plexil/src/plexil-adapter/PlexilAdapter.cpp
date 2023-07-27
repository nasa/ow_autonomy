// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter as common to all testbeds.

// ow_plexil
#include "PlexilAdapter.h"
#include "OwExecutive.h"
#include "adapter_support.h"
#include "subscriber.h"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// PLEXIL API
#include <AdapterConfiguration.hh>
#include <AdapterFactory.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
#include <Expression.hh>
using namespace PLEXIL;

void PlexilAdapter::propagateValueChange (const State& state,
                                          const std::vector<Value>& vals)
{
  if (! isStateSubscribed (state)) {
    debugMsg("PlexilAdapter:propagateValueChange", " ignoring " << state);
    return;
  }
  debugMsg("PlexilAdapter:propagateValueChange", " sending " << state);
  OwExecutive::instance()->plexilInterfaceMgr()
    -> handleValueChange (state, vals.front());
  OwExecutive::instance()->plexilInterfaceMgr()
    -> notifyOfExternalEvent();
}

bool PlexilAdapter::isStateSubscribed(const State& state) const
{
  return m_subscribedStates.find(state) != m_subscribedStates.end();
}

PlexilAdapter::PlexilAdapter(AdapterExecInterface& execInterface,
                             PLEXIL::AdapterConf* conf)
  : InterfaceAdapter(execInterface, conf)
{
  debugMsg("PlexilAdapter", " created.");
}

bool PlexilAdapter::initialize(AdapterConfiguration* config)
{
  // Plexil 4.6:
  // config->defaultRegisterAdapter(this);
  // Tried this in Plexil 6:
  //  config->setDefaultCommandHandler(this);

  config->registerCommandHandlerFunction("log_info", log_info);
  config->registerCommandHandlerFunction("log_warning", log_warning);
  config->registerCommandHandlerFunction("log_error", log_error);
  config->registerCommandHandlerFunction("log_debug", log_debug);
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
