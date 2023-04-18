// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

// Implementation of PLEXIL interface adapter for OWLAT simulator.

// ow_plexil
#include "OwlatAdapter.h"
#include "OwlatInterface.h"
#include "adapter_support.h"
#include "joint_support.h"
#include "subscriber.h"
using namespace PLEXIL;

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// C++
#include <map>


// PLEXIL API
#include <AdapterConfiguration.hh>
#include <AdapterFactory.hh>
#include <AdapterExecInterface.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
#include <Expression.hh>
#include <StateCacheEntry.hh>

using std::string;

static void owlat_arm_move_joints (Command* cmd, AdapterExecInterface* intf)
{
  bool relative;
  vector<double> const *angles_vector = nullptr;
  RealArray const *angles = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(angles);
  //change real array into a vector
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveJoints (relative, *angles_vector,
                                                  CommandId);
  acknowledge_command_sent(*cr);
}

static void owlat_arm_move_joints_guarded (Command* cmd,
                                           AdapterExecInterface* intf)
{
  bool relative, retracting;
  double force_threshold, torque_threshold;
  vector<double> const *angles_vector = nullptr;
  RealArray const *angles = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(angles);
  args[2].getValue(retracting);
  args[3].getValue(force_threshold);
  args[4].getValue(torque_threshold);
  //change real array into a vector
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveJointsGuarded (relative,
                                                         *angles_vector,
                                                         retracting,
                                                         force_threshold,
                                                         torque_threshold,
                                                         CommandId);
  acknowledge_command_sent(*cr);
}

static void owlat_arm_place_tool (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative, retracting;
  double force_threshold, torque_threshold, distance, overdrive;
  vector<double> const *position_vector = nullptr;
  vector<double> const *normal_vector = nullptr;
  RealArray const *position = nullptr;
  RealArray const *normal = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(position);
  args[3].getValuePointer(normal);
  args[4].getValue(distance);
  args[5].getValue(overdrive);
  args[6].getValue(retracting);
  args[7].getValue(force_threshold);
  args[8].getValue(torque_threshold);
  //change real array into a vector
  position->getContentsVector(position_vector);
  normal->getContentsVector(normal_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmPlaceTool (frame, relative,
                                                 *position_vector,
                                                 *normal_vector,
                                                 distance, overdrive,
                                                 retracting,
                                                 force_threshold,
                                                 torque_threshold,
                                                 CommandId);
  acknowledge_command_sent(*cr);
}

static void arm_set_tool (Command* cmd, AdapterExecInterface* intf)
{
  int tool;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(tool);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->armSetTool (tool, CommandId);
  acknowledge_command_sent(*cr);
}

static void arm_tare_ft_sensor (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->armTareFTSensor (CommandId);
  acknowledge_command_sent(*cr);
}

static void task_psp (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  double max_depth, max_force;
  vector<double> const *point_vector = nullptr;
  vector<double> const *normal_vector = nullptr;
  RealArray const *point = nullptr;
  RealArray const *normal = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(point);
  args[3].getValuePointer(normal);
  args[4].getValue(max_depth);
  args[5].getValue(max_force);
  //change real array into a vector
  point->getContentsVector(point_vector);
  normal->getContentsVector(normal_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatTaskPSP (frame, relative,*point_vector,
                                            *normal_vector, max_depth, max_force,
                                            CommandId);
  acknowledge_command_sent(*cr);
}

static void owlat_task_scoop (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  vector<double> const *point_vector = nullptr;
  vector<double> const *normal_vector = nullptr;
  RealArray const *point = nullptr;
  RealArray const *normal = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(point);
  args[3].getValuePointer(normal);
  //change real array into a vector
  point->getContentsVector(point_vector);
  normal->getContentsVector(normal_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatTaskScoop (frame, relative,*point_vector,
                                              *normal_vector, CommandId);
  acknowledge_command_sent(*cr);
}

static void using_owlat (const State&, StateCacheEntry& entry)
{
  entry.update(true);
}

static void using_oceanwaters (const State&, StateCacheEntry& entry)
{
  entry.update(false);
}

static void armJointAngles (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointAngles ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmJointAngles());
}

static void armFTTorque (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmFTTorque ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmFTTorque());
}

static void armFTForce (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmFTForce ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmFTForce());
}

static void armTool (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmTool ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmTool());
}

static void pspStopReason (const State& state, StateCacheEntry &entry)
{
  debugMsg("getPSPStopReason ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getPSPStopReason());
}

static void joint_velocity (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_velocity ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(OwlatInterface::instance()->
               getJointTelemetry(joint, TelemetryType::Velocity));
}

static void joint_position (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_position ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(OwlatInterface::instance()->
               getJointTelemetry(joint, TelemetryType::Position));
}

static void joint_effort (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_effort ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(OwlatInterface::instance()->
               getJointTelemetry(joint, TelemetryType::Effort));
}

static void joint_acceleration (const State& state, StateCacheEntry &entry)
{
  const vector<PLEXIL::Value>& args = state.parameters();
  debugMsg("joint_effort ", "lookup called for " << state.name()
           << " with " << args.size() << " args");
  int joint;
  args[0].getValue(joint);
  entry.update(OwlatInterface::instance()->
               getJointTelemetry(joint, TelemetryType::Acceleration));
}

static void armJointAccelerations (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointAccelerations ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmJointAccelerations());
}

static void armJointTorques (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointTorques ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmJointTorques());
}

static void armJointVelocities (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmJointVelocities ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmJointVelocities());
}

static void armPose (const State& state, StateCacheEntry &entry)
{
  debugMsg("getArmPose ", "lookup called for " << state.name()
           << " with " << state.parameters().size() << " args");
  entry.update(OwlatInterface::instance()->getArmPose());
}


static void default_lookup_handler (const State& state, StateCacheEntry &entry)
{
  ROS_WARN ("Unsupported Plexil Lookup %s, called with %zu arguments",
            state.name().c_str(), state.parameters().size());
  entry.update(Unknown);
}

OwlatAdapter::OwlatAdapter (AdapterExecInterface& execInterface,
                            const pugi::xml_node& configXml)
  : LanderAdapter (execInterface, configXml)
{
  debugMsg("OwlatAdapter", " created.");
}

bool OwlatAdapter::initialize()
{
  LanderAdapter::initialize (OwlatInterface::instance());

  // Commands

  g_configuration->registerCommandHandler("arm_set_tool", arm_set_tool);
  g_configuration->registerCommandHandler("arm_tare_ft_sensor", arm_tare_ft_sensor);
  g_configuration->registerCommandHandler("owlat_arm_move_joints",
                                          owlat_arm_move_joints);
  g_configuration->registerCommandHandler("owlat_arm_move_joints_guarded",
                                          owlat_arm_move_joints_guarded);
  g_configuration->registerCommandHandler("owlat_arm_place_tool",
                                          owlat_arm_place_tool);
  g_configuration->registerCommandHandler("owlat_task_scoop", owlat_task_scoop);
  g_configuration->registerCommandHandler("task_psp", task_psp);

  // Telemetry
  g_configuration->registerLookupHandler("UsingOWLAT", using_owlat);
  g_configuration->registerLookupHandler("UsingOceanWATERS", using_oceanwaters);
  g_configuration->registerLookupHandler("ArmJointAngles", armJointAngles);
  g_configuration->registerLookupHandler("ArmFTTorque", armFTTorque);
  g_configuration->registerLookupHandler("ArmFTForce", armFTForce);
  g_configuration->registerLookupHandler("ArmTool", armTool);
  g_configuration->registerLookupHandler("PSPStopReason", pspStopReason);
  g_configuration->registerLookupHandler("JointVelocity", joint_velocity);
  g_configuration->registerLookupHandler("JointPosition", joint_position);
  g_configuration->registerLookupHandler("JointEffort", joint_effort);
  g_configuration->registerLookupHandler("JointAcceleration", joint_acceleration);
  g_configuration->setDefaultLookupHandler(default_lookup_handler);

  debugMsg("OwlatAdapter", " initialized.");
  return true;
}

extern "C" {
  void initowlat_adapter() {
    REGISTER_ADAPTER(OwlatAdapter, "owlat_adapter");
  }
}
