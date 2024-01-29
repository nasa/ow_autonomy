// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ow_plexil
#include "adapter_support.h"

// ROS
#include <ros/ros.h>

// PLEXIL
#include <Debug.hh>

// C++
#include <map>
#include <mutex>

using namespace PLEXIL;
using std::string;
using std::vector;
using std::unique_ptr;
using std::get;
using std::mutex;

// An empty argument vector, for convenience.
const vector<Value> EmptyArgs;

PlexilAdapter* g_adapter;

static mutex g_shared_mutex;

int g_cmd_id = 0;

std::map<int, unique_ptr<CommandRecord>> CommandRegistry;

unique_ptr<CommandRecord>&
new_command_record(Command* cmd, AdapterExecInterface* intf)
{
  auto cr = std::make_tuple(cmd, intf, false);
  CommandRegistry[++g_cmd_id] = std::make_unique<CommandRecord>(cr);
  return CommandRegistry[g_cmd_id];
}

static void ack_command (Command* cmd,
                         CommandHandleValue handle,
                         AdapterExecInterface* intf)
{
  intf->handleCommandAck(cmd, handle);
  intf->notifyOfExternalEvent();
}

void acknowledge_command_success (Command* cmd, AdapterExecInterface* intf)
{
  ack_command (cmd, COMMAND_SUCCESS, intf);
}

static void acknowledge_command_failure (Command* cmd, AdapterExecInterface* intf)
{
  ack_command (cmd, COMMAND_FAILED, intf);
}

void acknowledge_command_denied (Command* cmd, AdapterExecInterface* intf)
{
  ack_command (cmd, COMMAND_DENIED, intf);
}

void acknowledge_command_sent(CommandRecord& cr, bool skip)
{
  // Sends acknowledgment that command issued by a Plexil plan was
  // sent to system, in a way that guarantees only one acknowledgment
  // (acks are not idempotent).  NOTE: this thread safety is not
  // provided in the other acknowledgment types above, and assumed not
  // needed in them.

  std::lock_guard<mutex> g(g_shared_mutex);
  bool& sent_flag = get<CR_ACK_SENT>(cr);
  if (!sent_flag)
  {
    if (!skip) {
      ack_command (get<CR_COMMAND>(cr),
		   COMMAND_SENT_TO_SYSTEM,
		   get<CR_ADAPTER>(cr));
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
  acknowledge_command_sent(*cr, true);
  if (success) acknowledge_command_success (cmd, intf);
  else acknowledge_command_failure (cmd, intf);
}

static State create_state (const string& state_name, const vector<Value>& value)
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

// For effeciency, the following two (overloaded) functions assume the
// g_adapter object has already been initialized.

static void propagate (const State& state, const Value& value)
{
  g_adapter->propagateValueChange (state, value);
}

static void propagate (const State& state, const vector<Value>& value)
{
  g_adapter->propagateValueChange (state, value);
}

template<class V>
void receive_value (const std::string& state_name, const V& val)
{
  propagate (create_state(state_name, EmptyArgs), val);
}

template<class V, class A>
void receive_value_from_arg (const std::string& state_name,
                             const V& val,
                             const A& arg)
{
  propagate (create_state(state_name, std::vector<Value>(1, arg)), val);
}

void receiveBool (const string& state_name, bool val)
{
  receive_value<bool> (state_name, val);
}

void receiveInt (const string& state_name, int val)
{
  receive_value<int> (state_name, val);
}

void receiveDouble (const string& state_name, double val)
{
  receive_value<double> (state_name, val);
}

void receiveString (const string& state_name, const string& val)
{
  receive_value<string> (state_name, val);
}

void receiveIntFromString (const string& state_name, int val, const string& arg)
{
  receive_value_from_arg<int, string> (state_name, val, arg);
}

void receiveBoolFromString (const string& state_name, bool val, const string& arg)
{
  receive_value_from_arg<bool, string> (state_name, val, arg);
}

void receiveDoubleFromInt (const string& state_name, double val, int arg)
{
  receive_value_from_arg<double, int> (state_name, val, arg);
}

void receiveDoubleVector (const string& state_name,
                          const vector<double>& vals)
{
  vector<Value> v;
  v.resize(vals.size());
  for(int i = 0; i < vals.size(); i++) {
    v[i] = Value(static_cast<Real>(vals[i]));
  }
  receive_value<vector<Value> > (state_name, v);
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

void log_info (Command* cmd, AdapterExecInterface* intf)
{
  ROS_INFO("%s", log_string(cmd->getArgValues()).c_str());
  acknowledge_command_success (cmd, intf);
}

void log_warning (Command* cmd, AdapterExecInterface* intf)
{
  ROS_WARN("%s", log_string(cmd->getArgValues()).c_str());
  acknowledge_command_success (cmd, intf);
}

void log_error (Command* cmd, AdapterExecInterface* intf)
{
  ROS_ERROR("%s", log_string(cmd->getArgValues()).c_str());
  acknowledge_command_success (cmd, intf);
}

void log_debug (Command* cmd, AdapterExecInterface* intf)
{
  ROS_DEBUG("%s", log_string(cmd->getArgValues()).c_str());
  acknowledge_command_success (cmd, intf);
}
