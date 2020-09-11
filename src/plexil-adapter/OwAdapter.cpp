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
#include <list>
#include <map>
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


//////////////////////////// Command Handling //////////////////////////////

static int CommandId = 0;

static std::map<int, Command*> CommandRegistry;

static void command_sent (Command* cmd)
{
  g_execInterface->handleCommandAck(cmd, COMMAND_SENT_TO_SYSTEM);
  g_execInterface->notifyOfExternalEvent();
}

static void command_success (Command* cmd)
{
  g_execInterface->handleCommandAck(cmd, COMMAND_SUCCESS);
  g_execInterface->notifyOfExternalEvent();
}

static void ack_command (Command* cmd, PLEXIL::CommandHandleValue handle)
{
  g_execInterface->handleCommandAck(cmd, handle);
  g_execInterface->notifyOfExternalEvent();
}

static void command_status_callback (int id, bool success)
{
  Command* cmd;

  try {
    cmd = CommandRegistry.at(id);
  }
  catch (const std::out_of_range& oor) {
    ROS_ERROR("command_status_callback: no command registered under id %d", id);
    return;
  }
  // This ack is needed only when the command was registered.
  if (success) ack_command (cmd, COMMAND_SUCCESS);
  else ack_command (cmd, COMMAND_FAILED);
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

static void log_info (Command* cmd)
{
  ROS_INFO("%s", log_string(cmd->getArgValues()).c_str());
  ack_command (cmd, COMMAND_SUCCESS);
}

static void log_warning (Command* cmd)
{
  ROS_WARN("%s", log_string(cmd->getArgValues()).c_str());
  command_success (cmd);
}

static void log_error (Command* cmd)
{
  ROS_ERROR("%s", log_string(cmd->getArgValues()).c_str());
  command_success (cmd);
}

static void log_debug (Command* cmd)
{
  ROS_DEBUG("%s", log_string(cmd->getArgValues()).c_str());
  command_success (cmd);
}

static void unstow (Command* cmd)
{
  CommandRegistry[CommandId++] = cmd;
  OwInterface::instance()->unstow (CommandId);
  command_sent (cmd);
}

static void publish_trajectory (Command* cmd)
{
  CommandRegistry[CommandId] = cmd;
  OwInterface::instance()->publishTrajectory (CommandId);
  CommandId++;
  command_sent (cmd);
}

static void guarded_move (Command* cmd)
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
  CommandRegistry[CommandId] = cmd;
  OwInterface::instance()->guardedMove (x, y, z, dir_x, dir_y, dir_z,
                                        search_distance, CommandId);
  CommandId++;
  command_sent (cmd);
}

static void grind (Command* cmd)
{
  double x, y, depth, length, ground_pos;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(depth);
  args[3].getValue(length);
  args[4].getValue(ground_pos);
  CommandRegistry[CommandId] = cmd;
  OwInterface::instance()->grind(x, y, depth, length, ground_pos, CommandId);
  CommandId++;
}

static void dig_circular (Command* cmd)
{
  double x, y, depth, ground_position;
  bool radial;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(depth);
  args[3].getValue(ground_position);
  args[4].getValue(radial);
  CommandRegistry[CommandId] = cmd;
  OwInterface::instance()->digCircular(x, y, depth, ground_position, radial,
                                       CommandId);
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
  g_configuration->registerCommandHandler("unstow", unstow);
  g_configuration->registerCommandHandler("grind", grind);
  g_configuration->registerCommandHandler("guarded_move", guarded_move);
  g_configuration->registerCommandHandler("dig_circular", dig_circular);
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

  // "Demos" - temporary
  if (name == "guarded_move_demo") OwInterface::instance()->guardedMoveDemo();
  else if (name == "guarded_move_action_demo") { // proof of concept for now
    OwInterface::instance()->guardedMoveActionDemo();
  }
  // Operations
  else if (name == "dig_linear") {
    double x, y, depth, length, ground_position;
    args[0].getValue(x);
    args[1].getValue(y);
    args[2].getValue(depth);
    args[3].getValue(length);
    args[4].getValue(ground_position);
    OwInterface::instance()->digLinear(x, y, depth, length, ground_position);
  }
  else if (name == "deliver_sample") {
    double x, y, z;
    args[0].getValue(x);
    args[1].getValue(y);
    args[2].getValue(z);
    OwInterface::instance()->deliverSample (x, y, z);
  }
  else if (name == "stow") {
    OwInterface::instance()->stow();
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
