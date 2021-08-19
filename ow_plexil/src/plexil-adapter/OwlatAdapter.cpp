// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter for OWLAT simulator.

// ow_plexil
#include "OwlatAdapter.h"
#include "OwlatInterface.h"
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

using std::string;

static void owlat_unstow (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatUnstow (CommandId);
  send_ack_once(*cr);
}

static void owlat_stow (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatStow (CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_move_cartesian (Command* cmd, AdapterExecInterface* intf)
{
  string frame;
  bool relative;
  vector<double> position_vector;
  vector<double> orientation_vector;
  //double* position;
  //double* orientation;
  //Value position = Value(positionv);
  //Value orientation = Value(orientationv);
 // vector<Real> position;
 // vector<Real> orientation;
  size_t length = 5;
  RealArray *position = &RealArray(length, false);
  RealArray *orientation = &RealArray(length, false);
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(position);
  args[3].getValuePointer(orientation);
  position->getContentsVector(position_vector);
  orientation->getContentsVector(orientation_vector);


  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveCartesian (frame, relative, position_vector, 
                                                     orientation_vector, CommandId);
  send_ack_once(*cr);
}

OwlatAdapter::OwlatAdapter (AdapterExecInterface& execInterface,
                            const pugi::xml_node& configXml)
  : CommonAdapter (execInterface, configXml)
{
  debugMsg("OwlatAdapter", " created.");
}

bool OwlatAdapter::initialize()
{
  CommonAdapter::initialize();
  g_configuration->registerCommandHandler("owlat_unstow", owlat_unstow);
  g_configuration->registerCommandHandler("owlat_stow", owlat_stow);
  g_configuration->registerCommandHandler("owlat_arm_move_cartesian", owlat_arm_move_cartesian);
  OwlatInterface::instance()->setCommandStatusCallback (command_status_callback);
  debugMsg("OwlatAdapter", " initialized.");
  return true;
}

void OwlatAdapter::lookupNow (const State& state, StateCacheEntry& entry)
{
  debugMsg("OwlatAdapter:lookupNow", " called on " << state.name() << " with "
           << state.parameters().size() << " arguments");

  Value retval = Unknown;  // the value of the queried state

  // At the moment there are no lookups defined for OWLAT.
  ROS_ERROR("PLEXIL Adapter: Invalid lookup name: %s", state.name().c_str());
  
  entry.update(retval);
}


extern "C" {
  void initowlat_adapter() {
    REGISTER_ADAPTER(OwlatAdapter, "owlat_adapter");
  }
}
