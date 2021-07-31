// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter for OWLAT simulator.

// OW
#include "OwlatAdapter.h"
#include "OWLATSimInterface.h"
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


//////////////////////////// Command Handling //////////////////////////////

static int CommandId = 0;

static std::mutex g_shared_mutex;
using CommandRecord = std::tuple<Command*,
                                 AdapterExecInterface*,
                                 bool>;

enum CommandRecordFields {CR_COMMAND, CR_ADAPTER, CR_ACK_SENT};

static std::map<int, std::unique_ptr<CommandRecord>> CommandRegistry;

static std::unique_ptr<CommandRecord>&
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

static void owlat_unstow (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OWLATSimInterface::instance()->owlatUnstow (CommandId);
  send_ack_once(*cr);
}

static void owlat_stow (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OWLATSimInterface::instance()->owlatStow (CommandId);
  send_ack_once(*cr);
}

OwlatAdapter::OwlatAdapter (AdapterExecInterface& execInterface,
                            const pugi::xml_node& configXml)
  : CommonAdapter (execInterface, configXml)
{
  debugMsg("OwlatAdapter", " created.");
}

OwlatAdapter::~OwlatAdapter ()
{
}

bool OwlatAdapter::initialize()
{
  CommonAdapter::initialize();
  g_configuration->registerCommandHandler("owlat_unstow", owlat_unstow);
  g_configuration->registerCommandHandler("owlat_stow", owlat_stow);
  OWLATSimInterface::instance()->
    setCommandStatusCallback (command_status_callback);
  debugMsg("OwlatAdapter", " initialized.");
  return true;
}

void OwlatAdapter::lookupNow (const State& state, StateCacheEntry& entry)
{
  debugMsg("OwlatAdapter:lookupNow", " called on " << state.name() << " with "
           << state.parameters().size() << " arguments");

  Value retval = Unknown;  // the value of the queried state

  // At the moment there are no lookups in OWLAT, so return Unknown.
  //  if (! lookup(state.name(), state.parameters(), retval)) {
  //    ROS_ERROR("PLEXIL Adapter: Invalid lookup name: %s", state.name().c_str());
  //  }

  entry.update(retval);
}


extern "C" {
  void initowlat_adapter() {
    REGISTER_ADAPTER(OwlatAdapter, "owlat_adapter");
  }
}
