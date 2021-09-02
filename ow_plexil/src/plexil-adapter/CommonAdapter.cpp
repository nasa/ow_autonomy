// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter as common to all testbeds.

// ow_plexil
#include "CommonAdapter.h"
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
#include <StateCacheEntry.hh>
using namespace PLEXIL;

void CommonAdapter::propagateValueChange (const State& state,
                                          const std::vector<Value>& vals) const
{
  if (! isStateSubscribed (state)) {
    debugMsg("CommonAdapter:propagateValueChange", " ignoring " << state);
    return;
  }

  debugMsg("CommonAdapter:propagateValueChange", " sending " << state);
  m_execInterface.handleValueChange (state, vals.front());
  m_execInterface.notifyOfExternalEvent();
}

bool CommonAdapter::isStateSubscribed(const State& state) const
{
  return m_subscribedStates.find(state) != m_subscribedStates.end();
}

CommonAdapter::CommonAdapter(AdapterExecInterface& execInterface,
                     const pugi::xml_node& configXml)
  : InterfaceAdapter(execInterface, configXml)
{
  debugMsg("CommonAdapter", " created.");
}

bool CommonAdapter::initialize()
{
  g_configuration->defaultRegisterAdapter(this);
  g_configuration->registerCommandHandler("log_info", log_info);
  g_configuration->registerCommandHandler("log_warning", log_warning);
  g_configuration->registerCommandHandler("log_error", log_error);
  g_configuration->registerCommandHandler("log_debug", log_debug);
  setSubscriber (receiveBool);
  setSubscriber (receiveString);
  setSubscriber (receiveDouble);
  setSubscriber (receiveBoolString);
  setSubscriber (receiveDoubleVector);
  g_adapter = this;
  debugMsg("CommonAdapter", " initialized.");
  return true;
}

bool CommonAdapter::start()
{
  debugMsg("CommonAdapter", " started.");
  return true;
}

bool CommonAdapter::stop()
{
  debugMsg("CommonAdapter", " stopped.");
  return true;
}

bool CommonAdapter::reset()
{
  debugMsg("CommonAdapter", " reset.");
  return true;
}

bool CommonAdapter::shutdown()
{
  debugMsg("CommonAdapter", " shut down.");
  return true;
}

void CommonAdapter::invokeAbort(Command *cmd)
{
  ROS_ERROR("Cannot abort command %s, not implemented, ignoring.",
            cmd->getName().c_str());
}

void CommonAdapter::subscribe(const State& state)
{
  debugMsg("CommonAdapter:subscribe", " to state " << state.name());
  m_subscribedStates.insert(state);
}


void CommonAdapter::unsubscribe (const State& state)
{
  debugMsg("CommonAdapter:unsubscribe", " from state " << state.name());
  m_subscribedStates.erase(state);
}
