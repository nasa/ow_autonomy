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

// C++
#include <utility>  // for std::pair
#include <list>
#include <map>
#include <iostream>
using std::string;
using std::vector;
using std::pair;
using std::make_pair;
using std::list;
using std::copy;
using std::cout;
using std::endl;


///////////////////////////// Conveniences //////////////////////////////////

// A prettier name for the "unknown" value.
//static Value const Unknown;
const Value Unknown;

// An empty argument vector.
static vector<Value> const EmptyArgs;


//////////////////////////// Command Handling //////////////////////////////

static int CommandId = 0;

static std::map<int, pair<Command*, AdapterExecInterface*>> CommandRegistry;

static void ack_command (Command* cmd,
                         PLEXIL::CommandHandleValue handle,
                         AdapterExecInterface* intf)
{
  intf->handleCommandAck(cmd, handle);
  intf->notifyOfExternalEvent();
}

static void ack_success (Command* cmd, AdapterExecInterface* intf)
{
  ack_command (cmd, COMMAND_SUCCESS, intf);
}

static void ack_failure (Command* cmd, AdapterExecInterface* intf)
{
  ack_command (cmd, COMMAND_FAILED, intf);
}


static void ack_sent (Command* cmd, AdapterExecInterface* intf)
{
  ack_command (cmd, COMMAND_SENT_TO_SYSTEM, intf);
}

static void command_status_callback (int id, bool success)
{
  pair<Command*, AdapterExecInterface*> cmd;

  try {
    cmd = CommandRegistry.at(id);
  }
  catch (const std::out_of_range& oor) {
    ROS_ERROR("command_status_callback: no command registered under id %d", id);
    return;
  }
  if (success) ack_success (cmd.first, cmd.second);
  else ack_failure (cmd.first, cmd.second);
}


static string log_string (const vector<Value>& args)
{
  std::ostringstream out;
  out << "PLEXIL: ";
  for (vector<Value>::const_iterator iter = args.begin();
       iter != args.end();
       iter++) out << *iter;
  return out.str();
}

static void log_info (Command* cmd, AdapterExecInterface* intf)
{
  ROS_INFO("%s", log_string(cmd->getArgValues()).c_str());
  ack_success (cmd, intf);
}

static void log_warning (Command* cmd, AdapterExecInterface* intf)
{
  ROS_WARN("%s", log_string(cmd->getArgValues()).c_str());
  ack_success (cmd, intf);
}

static void log_error (Command* cmd, AdapterExecInterface* intf)
{
  ROS_ERROR("%s", log_string(cmd->getArgValues()).c_str());
  ack_success (cmd, intf);
}

static void log_debug (Command* cmd, AdapterExecInterface* intf)
{
  ROS_DEBUG("%s", log_string(cmd->getArgValues()).c_str());
  ack_success (cmd, intf);
}

static void stow (Command* cmd, AdapterExecInterface* intf)
{
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->stow (CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void unstow (Command* cmd, AdapterExecInterface* intf)
{
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->unstow (CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void publish_trajectory (Command* cmd, AdapterExecInterface* intf)
{
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->publishTrajectory (CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void guarded_move (Command* cmd, AdapterExecInterface* intf)
{
  double x, y, z, dir_x, dir_y, dir_z, search_distance;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(z);
  args[3].getValue(dir_x);
  args[4].getValue(dir_y);
  args[5].getValue(dir_z);
  args[6].getValue(search_distance);
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->guardedMove (x, y, z, dir_x, dir_y, dir_z,
                                        search_distance, CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void guarded_move_action_demo (Command* cmd,
                                      AdapterExecInterface* intf)
{
  double x, y, z, dir_x, dir_y, dir_z, search_distance;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(z);
  args[3].getValue(dir_x);
  args[4].getValue(dir_y);
  args[5].getValue(dir_z);
  args[6].getValue(search_distance);
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->guardedMoveActionDemo (x, y, z, dir_x, dir_y, dir_z,
                                                  search_distance, CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void grind (Command* cmd, AdapterExecInterface* intf)
{
  double x, y, depth, length, ground_pos;
  bool parallel;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(depth);
  args[3].getValue(length);
  args[4].getValue(parallel);
  args[5].getValue(ground_pos);
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->grind(x, y, depth, length, parallel, ground_pos,
                                 CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void dig_circular (Command* cmd, AdapterExecInterface* intf)
{
  double x, y, depth, ground_position;
  bool parallel;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(depth);
  args[3].getValue(ground_position);
  args[4].getValue(parallel);
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->digCircular(x, y, depth, ground_position, parallel,
                                       CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void dig_linear (Command* cmd, AdapterExecInterface* intf)
{
  double x, y, depth, length, ground_position;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(depth);
  args[3].getValue(length);
  args[4].getValue(ground_position);
  OwInterface::instance()->digLinear(x, y, depth, length, ground_position,
                                     CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void deliver_sample (Command* cmd, AdapterExecInterface* intf)
{
  double x, y, z;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(z);
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->deliverSample (x, y, z, CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void tilt_antenna (Command* cmd, AdapterExecInterface* intf)
{
  double degrees;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (degrees);
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->tiltAntenna (degrees, CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void pan_antenna (Command* cmd, AdapterExecInterface* intf)
{
  double degrees;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (degrees);
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->panAntenna (degrees, CommandId);
  ack_sent (cmd, intf);
  CommandId++;
}

static void take_picture (Command* cmd, AdapterExecInterface* intf)
{
  CommandRegistry[CommandId] = make_pair (cmd, intf);
  OwInterface::instance()->takePicture();
  ack_sent (cmd, intf);
  CommandId++;
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
  g_configuration->registerCommandHandler("log_info", log_info);
  g_configuration->registerCommandHandler("log_warning", log_warning);
  g_configuration->registerCommandHandler("log_error", log_error);
  g_configuration->registerCommandHandler("log_debug", log_debug);
  g_configuration->registerCommandHandler("stow", stow);
  g_configuration->registerCommandHandler("unstow", unstow);
  g_configuration->registerCommandHandler("grind", grind);
  g_configuration->registerCommandHandler("guarded_move", guarded_move);
  g_configuration->registerCommandHandler("guarded_move_action_demo",
                                          guarded_move_action_demo);
  g_configuration->registerCommandHandler("dig_circular", dig_circular);
  g_configuration->registerCommandHandler("dig_linear", dig_linear);
  g_configuration->registerCommandHandler("deliver_sample", deliver_sample);
  g_configuration->registerCommandHandler("tilt_antenna", tilt_antenna);
  g_configuration->registerCommandHandler("pan_antenna", pan_antenna);
  g_configuration->registerCommandHandler("take_picture", take_picture);
  g_configuration->registerCommandHandler("publish_trajectory",
                                          publish_trajectory);
  TheAdapter = this;
  setSubscriber (receiveBool);
  setSubscriber (receiveString);
  setSubscriber (receiveDouble);
  setSubscriber (receiveBoolString);
  OwInterface::instance()->setCommandStatusCallback (command_status_callback);
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
  ROS_ERROR("Cannot abort command %s, not implemented, ignoring.",
            cmd->getName().c_str());
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
