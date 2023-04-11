// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

// ow_plexil
#include "LanderAdapter.h"
#include "LanderInterface.h"
#include "adapter_support.h"
#include "joint_support.h"
#include "subscriber.h"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// C++
#include <map>
using std::string;
using std::unique_ptr;

// PLEXIL API
#include <AdapterConfiguration.hh>
#include <AdapterFactory.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
#include <Expression.hh>
#include <StateCacheEntry.hh>
using namespace PLEXIL;

static void arm_unstow (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  LanderAdapter::s_interface->armUnstow (CommandId);
  acknowledge_command_sent(*cr);
}

static void arm_stop (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  LanderAdapter::s_interface->armStop (CommandId);
  acknowledge_command_sent(*cr);
}

static void arm_stow (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  LanderAdapter::s_interface->armStow (CommandId);
  acknowledge_command_sent(*cr);
}

static void arm_move_cartesian (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  vector<double> const *position_vector = nullptr;
  vector<double> const *orientation_vector = nullptr;
  RealArray const *position = nullptr;
  RealArray const *orientation = nullptr;
  // Get arguments.
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(position);
  args[3].getValuePointer(orientation);
  // Change real array into a vector.
  position->getContentsVector(position_vector);
  orientation->getContentsVector(orientation_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  LanderAdapter::s_interface->armMoveCartesian (frame, relative,
                                                *position_vector,
                                                *orientation_vector,
                                                CommandId);
  acknowledge_command_sent(*cr);
}

static void arm_move_cartesian_guarded (Command* cmd,
                                        AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  vector<double> const *position_vector = nullptr;
  vector<double> const *orientation_vector = nullptr;
  double force_threshold, torque_threshold;
  RealArray const *position = nullptr;
  RealArray const *orientation = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(position);
  args[3].getValuePointer(orientation);
  args[4].getValue(force_threshold);
  args[5].getValue(torque_threshold);

    //change real array into a vector
  position->getContentsVector(position_vector);
  orientation->getContentsVector(orientation_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  LanderAdapter::s_interface->armMoveCartesianGuarded (frame, relative,
                                                       *position_vector,
                                                       *orientation_vector,
                                                       force_threshold,
                                                       torque_threshold,
                                                       CommandId);
  acknowledge_command_sent(*cr);
}

static void arm_move_joint (Command* cmd, AdapterExecInterface* intf)
{
  bool relative;
  int joint;
  double angle; //radians
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValue(joint);
  args[2].getValue(angle);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  LanderAdapter::s_interface->armMoveJoint (relative, joint, angle, CommandId);
  acknowledge_command_sent(*cr);
}

static void default_lookup_handler (const State& state, StateCacheEntry &entry)
{
  debugMsg("default_lookup_handler", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  debugMsg("Invalid State: ", state.name());
  entry.update(Unknown);
}

LanderInterface* LanderAdapter::s_interface = NULL;

LanderAdapter::LanderAdapter (AdapterExecInterface& execInterface,
                              const pugi::xml_node& configXml)
  : PlexilAdapter (execInterface, configXml)
{
  debugMsg("LanderAdapter", " created.");
}

bool LanderAdapter::initialize (LanderInterface* li)
{
  PlexilAdapter::initialize();
  s_interface = li;
  s_interface->setCommandStatusCallback (command_status_callback);

  // Commands

  g_configuration->registerCommandHandler("arm_move_cartesian",
                                          arm_move_cartesian);
  g_configuration->registerCommandHandler("arm_move_cartesian_q",
                                          arm_move_cartesian);
  g_configuration->registerCommandHandler("arm_move_cartesian_guarded",
                                          arm_move_cartesian_guarded);
  g_configuration->registerCommandHandler("arm_move_cartesian_guarded_q",
                                          arm_move_cartesian_guarded);
  g_configuration->registerCommandHandler("arm_move_joint", arm_move_joint);
  g_configuration->registerCommandHandler("arm_stop", arm_stop);
  g_configuration->registerCommandHandler("arm_stow", arm_stow);
  g_configuration->registerCommandHandler("arm_unstow", arm_unstow);

  // Telemetry
  g_configuration->setDefaultLookupHandler(default_lookup_handler);

  debugMsg("LanderAdapter", " initialized.");
  return true;
}

extern "C" {
  void initlander_adapter() {
    REGISTER_ADAPTER(LanderAdapter, "lander_adapter");
  }
}
