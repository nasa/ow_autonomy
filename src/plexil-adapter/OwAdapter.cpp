// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter.

// OW
#include "OwAdapter.h"
#include "OwInterface.h"
#include "OwSimProxy.h"
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
  debugMsg("OwAdapter:receiveBool", " propagating " << state_name
           << " with value " << (val ? "true" : "false"));
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
  if (! isStateSubscribed (state)) {
    debugMsg("OwAdapter:propagateValueChange", " ignoring " << state);
    return;
  }

  debugMsg("OwAdapter:propagateValueChange", " sending " << state);
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

void OwAdapter::executeCommand(Command *cmd)
{

  // Sends a command (as invoked in a Plexil command node) to the system and
  // sends the status, and return value if applicable, back to the executive.

  string name = cmd->getName();
  debugMsg("OwAdapter:executeCommand", " for " << name);

  // Command arguments
  const vector<Value>& args = cmd->getArgValues();

  // Command return value
  Value retval = Unknown;

  // Utility commands
  if (name == "log_info") log_info (cmd->getArgValues());
  else if (name == "log_warning") log_warning (args);
  else if (name == "log_error") log_error (args);
  else if (name == "log_debug") log_debug (args);

  // "Demos"
  else if (name == "dig_circular_demo") OwInterface::instance()->digCircularDemo();
  else if (name == "guarded_move_demo") OwInterface::instance()->guardedMoveDemo();
  else if (name == "guarded_move_action_demo") { // proof of concept for now
    OwInterface::instance()->guardedMoveActionDemo();
  }
  else if (name == "publish_trajectory_demo") {
    OwInterface::instance()->publishTrajectoryDemo();
  }
  // Operations
  else if (name == "dig_linear") {
    double x, y, z, depth, length, width, pitch, yaw, dumpx, dumpy, dumpz;
    args[0].getValue(x);
    args[1].getValue(y);
    args[2].getValue(depth);
    args[3].getValue(length);
    args[4].getValue(width);
    args[5].getValue(pitch);
    args[6].getValue(yaw);
    args[7].getValue(dumpx);
    args[8].getValue(dumpy);
    args[9].getValue(dumpz);
    OwInterface::instance()->digLinear(x, y, depth, length, width,
                                       pitch, yaw, dumpx, dumpy, dumpz);
  }
  else if (name == "guarded_move") {
    double target_x, target_y, target_z, surf_norm_x, surf_norm_y, surf_norm_z;
    double offset_dist, overdrive_dist;
    bool delete_prev_traj, retract;
    args[0].getValue(target_x);
    args[1].getValue(target_y);
    args[2].getValue(surf_norm_x);
    args[3].getValue(surf_norm_y);
    args[4].getValue(surf_norm_z);
    args[5].getValue(offset_dist);
    args[6].getValue(overdrive_dist);
    args[7].getValue(delete_prev_traj);
    OwInterface::instance()->guardedMove (target_x, target_y,
                                          surf_norm_x, surf_norm_y, surf_norm_z,
                                          offset_dist, overdrive_dist,
                                          delete_prev_traj);
  }
  else if (name == "tilt_antenna") {
    double degrees;
    args[0].getValue (degrees);
    retval = OwInterface::instance()->tiltAntenna (degrees);
  }
  else if (name == "pan_antenna") {
    double degrees;
    args[0].getValue (degrees);
    retval = OwInterface::instance()->panAntenna (degrees);
  }
  else if (name == "take_picture") {
    OwInterface::instance()->takePicture();
  }
  else if (name == "stop_operation") {
    string name;
    args[0].getValue (name);
    OwInterface::instance()->stopOperation (name);
  }
  else if (name == "downlink_target") {
    OwInterface::instance()->downlinkTarget();
  }
  else if (name == "request_fwd_link") {
    OwInterface::instance()->requestFwdLink();
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
