// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// Implementation of PLEXIL interface adapter.

// OW
#include "OwAdapter.h"
#include "OwInterface.h"
#include "subscriber.h"

// ROS
#include <ros/ros.h>

// PLEXIL API
#include <AdapterConfiguration.hh>
#include <AdapterFactory.hh>
#include <AdapterExecInterface.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
#include <Expression.hh>
#include <StateCacheEntry.hh>

// C++/C
#include <list>
#include <iostream>
using std::string;
using std::vector;
using std::list;
using std::copy;
using std::cout;
using std::endl;

// Domain
#include "OwSimProxy.h"


///////////////////////////// Conveniences //////////////////////////////////

// A prettier name for the "unknown" value.
static Value const Unknown;

// An empty argument vector.
static vector<Value> const EmptyArgs;

static string log_string (const vector<Value>& args)
{
  std::ostringstream out;
  out << "PLEXIL: ";
  for (vector<Value>::const_iterator iter = args.begin();
       iter != args.end();
       iter++) out << *iter;
  return out.str();
}

static void log_info (const vector<Value>& args)
{
  ROS_INFO("%s", log_string(args).c_str());
}

static void log_warning (const vector<Value>& args)
{
  ROS_WARN("%s", log_string(args).c_str());
}

static void log_error (const vector<Value>& args)
{
  ROS_ERROR("%s", log_string(args).c_str());
}

static void log_debug (const vector<Value>& args)
{
  ROS_DEBUG("%s", log_string(args).c_str());
}


////////////////////// Publish/subscribe support ////////////////////////////

// A localized handle on the adapter, which allows a
// decoupling between the sample system and adapter.
static OwAdapter* TheAdapter;

static State createState (const string& state_name, const vector<Value>& value)
{
  State state(state_name, value.size());
  if (value.size() > 0)
  {
    for(size_t i=0; i<value.size();i++)
    {
      state.setParameter(i, value[i]);
    }
  }
  return state;
}

static void propagate (const State& state, const vector<Value>& value)
{
  TheAdapter->propagateValueChange (state, value);
}

// To do: templatize the following few

static void receiveBool (const string& state_name, bool val)
{
  propagate (createState(state_name, EmptyArgs),
             vector<Value> (1, val));
}

static void receiveDouble (const string& state_name, double val)
{
  propagate (createState(state_name, EmptyArgs),
             vector<Value> (1, val));
}

static void receiveString (const string& state_name, const string& val)
{
  propagate (createState(state_name, EmptyArgs),
             vector<Value> (1, val));
}

static void receiveBoolString (const string& state_name,
                               bool val,
                               const string& arg)
{
  propagate (createState(state_name, vector<Value> (1, arg)),
             vector<Value> (1, val));
}

void OwAdapter::propagateValueChange (const State& state,
                                       const vector<Value>& vals) const
{
  if (!isStateSubscribed(state)) return;

  m_execInterface.handleValueChange (state, vals.front());
  m_execInterface.notifyOfExternalEvent();
}

bool OwAdapter::isStateSubscribed(const State& state) const
{
  return m_subscribedStates.find(state) != m_subscribedStates.end();
}

///////////////////////////// Domain Support //////////////////////////////////

// We keep the domain interface as lightweight as possible!

static OwSimProxy* TheSimProxy = 0;



///////////////////////////// Member functions //////////////////////////////////


OwAdapter::OwAdapter(AdapterExecInterface& execInterface,
                     const pugi::xml_node& configXml) :
  InterfaceAdapter(execInterface, configXml)
{
  TheSimProxy = new OwSimProxy();
  debugMsg("OwAdapter", " created.");
}

OwAdapter::~OwAdapter ()
{
  if (TheSimProxy) delete TheSimProxy;
}

bool OwAdapter::initialize()
{
  g_configuration->defaultRegisterAdapter(this);
  TheAdapter = this;
  setSubscriber (receiveBool);
  setSubscriber (receiveString);
  setSubscriber (receiveDouble);
  setSubscriber (receiveBoolString);
  debugMsg("OwAdapter", " initialized.");
  return true;
}

bool OwAdapter::start()
{
  debugMsg("OwAdapter", " started.");
  return true;
}

bool OwAdapter::stop()
{
  debugMsg("OwAdapter", " stopped.");
  return true;
}

bool OwAdapter::reset()
{
  debugMsg("OwAdapter", " reset.");
  return true;
}

bool OwAdapter::shutdown()
{
  debugMsg("OwAdapter", " shut down.");
  return true;
}

void OwAdapter::invokeAbort(Command *cmd)
{
}


// Sends a command (as invoked in a Plexil command node) to the system and sends
// the status, and return value if applicable, back to the executive.
//
void OwAdapter::executeCommand(Command *cmd)
{
  string name = cmd->getName();
  debugMsg("OwAdapter:executeCommand", " for " << name);

  // Arguments
  vector<Value> argv(12);
  const vector<Value>& args = cmd->getArgValues();
  copy (args.begin(), args.end(), argv.begin());

  // Argument value holders
  double double_arg;

  // Return values
  Value retval = Unknown;

  // Utility commands
  if (name == "log_info") log_info (cmd->getArgValues());
  else if (name == "log_warning") log_warning (cmd->getArgValues());
  else if (name == "log_error") log_error (cmd->getArgValues());
  else if (name == "log_debug") log_debug (cmd->getArgValues());
  // "Demos"
  else if (name == "StartPlanning") OwInterface::instance()->startPlanningDemo();
  else if (name == "MoveGuarded") OwInterface::instance()->moveGuardedDemo();
  else if (name == "PublishTrajectory") {
    OwInterface::instance()->publishTrajectoryDemo();
  }
  // Operations
  else if (name == "DigTrenchOp") {
    double x, y, z, depth, length, width, pitch, yaw, dumpx, dumpy, dumpz;
    args[0].getValue(x);
    args[1].getValue(y);
    args[2].getValue(z);
    args[3].getValue(depth);
    args[4].getValue(length);
    args[5].getValue(width);
    args[6].getValue(pitch);
    args[7].getValue(yaw);
    args[8].getValue(dumpx);
    args[9].getValue(dumpy);
    args[10].getValue(dumpz);
    OwInterface::instance()->digTrench(x, y, z, depth, length, width,
                                       pitch, yaw, dumpx, dumpy, dumpz);
  }
  else if (name == "tilt_antenna") {
    args[0].getValue(double_arg);
    OwInterface::instance()->tiltAntenna (double_arg);
  }
  else if (name == "pan_antenna") {
    args[0].getValue(double_arg);
    OwInterface::instance()->panAntenna (double_arg);
  }
  else if (name == "take_picture") {
    OwInterface::instance()->takePicture();
  }
  else ROS_ERROR("Invalid command %s", name.c_str());

  m_execInterface.handleCommandAck(cmd, COMMAND_SENT_TO_SYSTEM);
  m_execInterface.handleCommandAck(cmd, COMMAND_SUCCESS);
  if (retval != Unknown) m_execInterface.handleCommandReturn(cmd, retval);
  m_execInterface.notifyOfExternalEvent();
  debugMsg("OwAdapter:executeCommand", " " << name << " complete");
}


///////////////////////////// State support //////////////////////////////////

void OwAdapter::lookupNow (const State& state, StateCacheEntry& entry)
{
  debugMsg("OwAdapter:lookupNow", " called on " << state.name() << " with "
           << state.parameters().size() << " arguments");

  Value retval = Unknown;  // the value of the queried state

  if (! TheSimProxy->lookup(state.name(), state.parameters(), retval)) {
    ROS_ERROR("PLEXIL Adapter: Invalid lookup name: %s", state.name().c_str());
  }
  entry.update(retval);
}


void OwAdapter::subscribe(const State& state)
{
  debugMsg("OwAdapter:subscribe", " to state " << state.name());
  m_subscribedStates.insert(state);
}


void OwAdapter::unsubscribe (const State& state)
{
  debugMsg("OwAdapter:unsubscribe", " from state " << state.name());
  m_subscribedStates.erase(state);
}

extern "C" {
  void initow_adapter() {
    REGISTER_ADAPTER(OwAdapter, "ow_adapter");
  }
}
