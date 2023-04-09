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

/*
static void armJointAngles (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointAngles ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmJointAngles());
}

static void armJointAccelerations (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointAccelerations ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmJointAccelerations());
}

static void armJointTorques (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointTorques ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmJointTorques());
}

static void armJointVelocities (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointVelocities ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmJointVelocities());
}

static void armFTTorque (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmFTTorque ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmFTTorque());
}

static void armFTForce (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmFTForce ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmFTForce());
}

static void armPose (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmPose ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmPose());
}

static void armTool (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmTool ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getArmTool());
}

static void pspStopReason (const State& state, StateCacheEntry &entry)
{
  debugMsg("getPSPStopReason ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getPSPStopReason());
}

static void panRadians (const State& state, StateCacheEntry &entry)
{
  debugMsg("PanRadians ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getPanRadians());
}

static void panDegrees (const State& state, StateCacheEntry &entry)
{
  debugMsg("PanDegrees ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getPanDegrees());
}

static void tiltRadians (const State& state, StateCacheEntry &entry)
{
  debugMsg("TiltRadians ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getTiltRadians());
}

static void tiltDegrees (const State& state, StateCacheEntry &entry)
{
  debugMsg("TiltDegrees ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(LanderAdapter::s_interface->getTiltDegrees());
}

static void joint_velocity (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_velocity ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(LanderAdapter::s_interface->
               getJointTelemetry(joint, TelemetryType::Velocity));
}

static void joint_position (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_position ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(LanderAdapter::s_interface->
               getJointTelemetry(joint, TelemetryType::Position));
}

static void joint_effort (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_effort ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(LanderAdapter::s_interface->
               getJointTelemetry(joint, TelemetryType::Effort));
}

static void joint_acceleration (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_effort ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(LanderAdapter::s_interface->
               getJointTelemetry(joint, TelemetryType::Acceleration));
}
*/

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
  /*
  g_configuration->registerLookupHandler("ArmJointAngles", armJointAngles);
  g_configuration->registerLookupHandler("ArmJointAccelerations",
                                         armJointAccelerations);
  g_configuration->registerLookupHandler("ArmJointTorques",
                                         armJointTorques);
  g_configuration->registerLookupHandler("ArmJointVelocities",
                                         armJointVelocities);
  g_configuration->registerLookupHandler("ArmFTTorque", armFTTorque);
  g_configuration->registerLookupHandler("ArmFTForce", armFTForce);
  g_configuration->registerLookupHandler("ArmPose", armPose);
  g_configuration->registerLookupHandler("ArmTool", armTool);
  g_configuration->registerLookupHandler("PSPStopReason", pspStopReason);
  g_configuration->registerLookupHandler("PanRadians", panRadians);
  g_configuration->registerLookupHandler("PanDegrees", panDegrees);
  g_configuration->registerLookupHandler("TiltRadians", tiltRadians);
  g_configuration->registerLookupHandler("TiltDegrees", tiltDegrees);
  g_configuration->registerLookupHandler("JointVelocity", joint_velocity);
  g_configuration->registerLookupHandler("JointPosition", joint_position);
  g_configuration->registerLookupHandler("JointEffort", joint_effort);
  g_configuration->registerLookupHandler("JointAcceleration", joint_acceleration);
  */
  g_configuration->setDefaultLookupHandler(default_lookup_handler);

  debugMsg("LanderAdapter", " initialized.");
  return true;
}

extern "C" {
  void initlander_adapter() {
    REGISTER_ADAPTER(LanderAdapter, "lander_adapter");
  }
}
