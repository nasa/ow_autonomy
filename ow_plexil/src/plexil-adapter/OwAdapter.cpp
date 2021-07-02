// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter.

// OW
#include "OwAdapter.h"
#include "OwInterface.h"
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


///////////////////////////// Conveniences //////////////////////////////////

// A prettier name for the "unknown" value.
//static Value const Unknown;
const Value Unknown;

// An empty argument vector.
static vector<Value> const EmptyArgs;


//////////////////////// PLEXIL Lookup Support //////////////////////////////

static void stubbed_lookup (const string& name, const string& value)
{
  // This warning is annoying.  Could parameterize it.
  //  ROS_WARN("PLEXIL Adapter: Stubbed lookup of %s returning %s",
  //           name.c_str(), value.c_str());
}


// NOTE: This macro, and the stub it implements, are temporary.
#define STATE_STUB(name,val)                    \
  if (state_name == #name) {                    \
    stubbed_lookup (#name, #val);               \
    value_out = val;                            \
  }

static bool lookup (const std::string& state_name,
                    const std::vector<PLEXIL::Value>& args,
                    PLEXIL::Value& value_out)
{
  bool retval = true;

  // Stubbed mission and system parameters.  Many of these will eventually be
  // obsolete.

  STATE_STUB(TrenchLength, 10)
  else STATE_STUB(TrenchGroundPosition, -0.155)
  else STATE_STUB(TrenchWidth, 10)
  else STATE_STUB(TrenchDepth, 2)
  else STATE_STUB(TrenchPitch, 0)
  else STATE_STUB(TrenchYaw, 0)
  else STATE_STUB(TrenchStartX, 5)
  else STATE_STUB(TrenchStartY, 10)
  else STATE_STUB(TrenchStartZ, 0)
  else STATE_STUB(TrenchDumpX, 0)
  else STATE_STUB(TrenchDumpY, 0)
  else STATE_STUB(TrenchDumpZ, 5)
  else STATE_STUB(TrenchIdentified, true)
  else STATE_STUB(TrenchTargetTimeout, 60)
  else STATE_STUB(ExcavationTimeout, 10)
  else STATE_STUB(ExcavationTimeout, 60)
  else STATE_STUB(SampleGood, true)
  else STATE_STUB(CollectAndTransferTimeout, 10)

  else if (state_name == "TiltDegrees") {
    value_out = OwInterface::instance()->getTilt();
  }
  else if (state_name == "PanDegrees") {
    value_out = OwInterface::instance()->getPanDegrees();
  }
  else if (state_name == "PanVelocity") {
    value_out = OwInterface::instance()->getPanVelocity();
  }
  else if (state_name == "TiltVelocity") {
    value_out = OwInterface::instance()->getTiltVelocity();
  }
  else if (state_name == "HardTorqueLimitReached") {
    string s;
    args[0].getValue(s);
    value_out = OwInterface::instance()->hardTorqueLimitReached(s);
  }
  else if (state_name == "SoftTorqueLimitReached") {
    string s;
    args[0].getValue(s);
    value_out = OwInterface::instance()->softTorqueLimitReached(s);
  }
  else if (state_name == "Running") {
    string operation;
    args[0].getValue(operation);
    value_out = OwInterface::instance()->running (operation);
  }
  else if (state_name == "StateOfCharge") {
    value_out = OwInterface::instance()->getStateOfCharge();
  }
  else if (state_name == "RemainingUsefulLife") {
    value_out = OwInterface::instance()->getRemainingUsefulLife();
  }
  else if (state_name == "BatteryTemperature") {
    value_out = OwInterface::instance()->getBatteryTemperature();
  }
  else if (state_name == "GroundFound") {
    value_out = OwInterface::instance()->groundFound();
  }
  else if (state_name == "GroundPosition") {
    value_out = OwInterface::instance()->groundPosition();
  }
  // Faults
  else if (state_name == "SystemFault") {
    value_out = OwInterface::instance()->systemFault();
  }
  else if (state_name == "AntennaFault") {
    value_out = OwInterface::instance()->antennaFault();
  }
  else if (state_name == "ArmFault") {
    value_out = OwInterface::instance()->armFault();
  }
  else if (state_name == "PowerFault") {
    value_out = OwInterface::instance()->powerFault();
  }
  else retval = false;

  return retval;
}

//////////////////////////// Command Handling //////////////////////////////

static int CommandId = 0;

std::mutex g_shared_mutex;
using CommandRecord = std::tuple<Command*,
                                 AdapterExecInterface*,
                                 bool>;

enum CommandRecordFields {CR_COMMAND, CR_ADAPTER, CR_ACK_SENT};

static std::map<int, std::unique_ptr<CommandRecord>> CommandRegistry;

std::unique_ptr<CommandRecord>& new_command_record(Command* cmd,
                                                   AdapterExecInterface* intf)
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

static void send_ack_once(CommandRecord& cr, bool skip=false)
{
  std::lock_guard<std::mutex> g(g_shared_mutex);
  bool& sent_flag = std::get<CR_ACK_SENT>(cr);
  if (!sent_flag)
  {
    if (!skip) {
      ack_sent(std::get<CR_COMMAND>(cr), std::get<CR_ADAPTER>(cr));
    }
    sent_flag = true;
  }
}

static void command_status_callback (int id, bool success)
{
  auto it = CommandRegistry.find(id);
  if (it == CommandRegistry.end())
  {
    ROS_ERROR_STREAM("command_status_callback: no command registered under id"
                     << id);
    return;
  }

  std::unique_ptr<CommandRecord>& cr = it->second;
  Command* cmd = std::get<CR_COMMAND>(*cr);
  AdapterExecInterface* intf = std::get<CR_ADAPTER>(*cr);
  send_ack_once(*cr, true);
  if (success) ack_success (cmd, intf);
  else ack_failure (cmd, intf);
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
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->stow (CommandId);
  send_ack_once(*cr);

}

static void unstow (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->unstow (CommandId);
  send_ack_once(*cr);
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
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->guardedMove (x, y, z, dir_x, dir_y, dir_z,
                                        search_distance, CommandId);
  send_ack_once(*cr);
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
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->grind(x, y, depth, length, parallel, ground_pos,
                                 CommandId);
  send_ack_once(*cr);
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
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->digCircular(x, y, depth, ground_position, parallel,
                                       CommandId);
  send_ack_once(*cr);
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
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->digLinear(x, y, depth, length, ground_position,
                                     CommandId);
  send_ack_once(*cr);
}

static void deliver (Command* cmd, AdapterExecInterface* intf)
{
  double x, y, z;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(x);
  args[1].getValue(y);
  args[2].getValue(z);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->deliver (x, y, z, CommandId);
  send_ack_once(*cr);
}

static void tilt_antenna (Command* cmd, AdapterExecInterface* intf)
{
  double degrees;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (degrees);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->tiltAntenna (degrees, CommandId);
  send_ack_once(*cr);
}

static void pan_antenna (Command* cmd, AdapterExecInterface* intf)
{
  double degrees;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (degrees);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->panAntenna (degrees, CommandId);
  send_ack_once(*cr);
}

static void take_picture (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->takePicture (CommandId);
  send_ack_once(*cr);
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


///////////////////////////// Member functions //////////////////////////////////


OwAdapter::OwAdapter(AdapterExecInterface& execInterface,
                     const pugi::xml_node& configXml) :
  InterfaceAdapter(execInterface, configXml)
{
  debugMsg("OwAdapter", " created.");
}

OwAdapter::~OwAdapter ()
{
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
  g_configuration->registerCommandHandler("dig_circular", dig_circular);
  g_configuration->registerCommandHandler("dig_linear", dig_linear);
  g_configuration->registerCommandHandler("deliver", deliver);
  g_configuration->registerCommandHandler("tilt_antenna", tilt_antenna);
  g_configuration->registerCommandHandler("pan_antenna", pan_antenna);
  g_configuration->registerCommandHandler("take_picture", take_picture);

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

  if (! lookup(state.name(), state.parameters(), retval)) {
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
