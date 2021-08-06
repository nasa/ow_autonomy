// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter as common to all testbeds.

// ow_plexil
#include "CommonAdapter.h"
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

// C++
#include <map>
#include <mutex>
using std::string;
using std::vector;
using std::unique_ptr;
using std::get;
using std::mutex;

// An empty argument vector.
//static vector<Value> const EmptyArgs;
const vector<Value> EmptyArgs;

// A pointer to the interface adapter, so that it can be accessed from static
// functions.  WARNING: this allows only one adapter instance to be usable at a
// time; multiple testbeds cannot operate concurrently.
static CommonAdapter* TheAdapter;

static mutex g_shared_mutex;

int CommandId = 0;

std::map<int, unique_ptr<CommandRecord>> CommandRegistry;

unique_ptr<CommandRecord>&
new_command_record(Command* cmd, AdapterExecInterface* intf)
{
  auto cr = std::make_tuple(cmd, intf, false);
  CommandRegistry[++CommandId] = std::make_unique<CommandRecord>(cr);
  return CommandRegistry[CommandId];
}

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

void send_ack_once(CommandRecord& cr, bool skip)
{
  std::lock_guard<mutex> g(g_shared_mutex);
  bool& sent_flag = get<CR_ACK_SENT>(cr);
  if (!sent_flag)
  {
    if (!skip) {
      ack_sent(get<CR_COMMAND>(cr), get<CR_ADAPTER>(cr));
    }
    sent_flag = true;
  }
}

void command_status_callback (int id, bool success)
{
  auto it = CommandRegistry.find(id);
  if (it == CommandRegistry.end())
  {
    ROS_ERROR_STREAM("command_status_callback: no command registered under id"
                     << id);
    return;
  }

  unique_ptr<CommandRecord>& cr = it->second;
  Command* cmd = get<CR_COMMAND>(*cr);
  AdapterExecInterface* intf = get<CR_ADAPTER>(*cr);
  send_ack_once(*cr, true);
  if (success) ack_success (cmd, intf);
  else ack_failure (cmd, intf);
}

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


void CommonAdapter::propagateValueChange (const State& state,
                                       const vector<Value>& vals) const
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

CommonAdapter::~CommonAdapter ()
{
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
  TheAdapter = this;
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
