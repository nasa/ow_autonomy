// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

// Implementation of PLEXIL interface adapter for OWLAT simulator.

// ow_plexil
#include "OwlatAdapter.h"
#include "OwlatInterface.h"
#include "adapter_support.h"
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
#include <LookupReceiver.hh>

using std::string;

static void arm_move_joints (Command* cmd, AdapterExecInterface* intf)
{
  bool relative;
  vector<double> const *angles_vector = nullptr;
  RealArray const *angles = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(angles);
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->armMoveJoints (relative, *angles_vector, g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void arm_move_joints_guarded (Command* cmd, AdapterExecInterface* intf)
{
  bool relative;
  double force_threshold, torque_threshold;
  vector<double> const *angles_vector = nullptr;
  RealArray const *angles = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(angles);
  args[2].getValue(force_threshold);
  args[3].getValue(torque_threshold);
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->armMoveJointsGuarded (relative,
                                                    *angles_vector,
                                                    force_threshold,
                                                    torque_threshold,
                                                    g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void arm_set_tool (Command* cmd, AdapterExecInterface* intf)
{
  int tool;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(tool);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->armSetTool (tool, g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void arm_tare_ft_sensor (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->armTareFTSensor (g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void task_shear_bevameter (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  double max_torque, preload;
  vector<double> const *point_vector = nullptr;
  vector<double> const *normal_vector = nullptr;
  RealArray const *point = nullptr;
  RealArray const *normal = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(point);
  args[3].getValuePointer(normal);
  args[4].getValue(preload);
  args[5].getValue(max_torque);
  point->getContentsVector(point_vector);
  normal->getContentsVector(normal_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->taskShearBevameter (frame,
                                                  relative,
                                                  *point_vector,
                                                  *normal_vector,
                                                  preload,
                                                  max_torque,
                                                  g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void task_p (bool psp, Command* cmd, AdapterExecInterface* intf)
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
  point->getContentsVector(point_vector);
  normal->getContentsVector(normal_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  if (psp) {
    OwlatInterface::instance()->taskPSP (frame, relative,*point_vector,
                                         *normal_vector, max_depth, max_force,
                                         g_cmd_id);
  }
  else {
    OwlatInterface::instance()->taskPenetrometer (frame, relative,
                                                  *point_vector,
                                                  *normal_vector,
                                                  max_depth, max_force,
                                                  g_cmd_id);
  }
  acknowledge_command_sent(*cr);
}

static void task_psp (Command* cmd, AdapterExecInterface* intf)
{
  task_p (true, cmd, intf);
}

static void task_penetrometer (Command* cmd, AdapterExecInterface* intf)
{
  task_p (false, cmd, intf);
}

static void task_scoop (bool linear, Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  double depth, length_or_scoop_angle;
  vector<double> const *point_vector = nullptr;
  vector<double> const *normal_vector = nullptr;
  RealArray const *point = nullptr;
  RealArray const *normal = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(point);
  args[3].getValuePointer(normal);
  args[4].getValue(depth);
  args[5].getValue(length_or_scoop_angle);
  point->getContentsVector(point_vector);
  normal->getContentsVector(normal_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  if (linear) {
    OwlatInterface::instance()->taskScoopLinear (frame, relative,
                                                 *point_vector, *normal_vector,
                                                 depth, length_or_scoop_angle,
                                                 g_cmd_id);
  }
  else {
    OwlatInterface::instance()->taskScoopCircular (frame, relative,
                                                   *point_vector, *normal_vector,
                                                   depth, length_or_scoop_angle,
                                                   g_cmd_id);
  }
  acknowledge_command_sent(*cr);
}


static void task_scoop_linear (Command* cmd, AdapterExecInterface* intf)
{
  task_scoop (true, cmd, intf);
}

static void task_scoop_circular (Command* cmd, AdapterExecInterface* intf)
{
  task_scoop (false, cmd, intf);
}

static void using_owlat (const State&, LookupReceiver* entry)
{
  entry->update(true);
}

static void using_oceanwaters (const State&, LookupReceiver* entry)
{
  entry->update(false);
}

static void arm_tool (const State&, LookupReceiver* entry)
{
  entry->update(OwlatInterface::instance()->getArmTool());
}

static void default_lookup_handler (const State& state, LookupReceiver* entry)
{
  ROS_WARN ("Unsupported Plexil Lookup %s, called with %zu arguments",
            state.name().c_str(), state.parameters().size());
  entry->update(Unknown);
}

OwlatAdapter::OwlatAdapter (AdapterExecInterface& execInterface,
                            PLEXIL::AdapterConf* conf)
  : LanderAdapter (execInterface, conf)
{
  debugMsg("OwlatAdapter", " created.");
}

bool OwlatAdapter::initialize (AdapterConfiguration* config)
{
  LanderAdapter::s_interface = OwlatInterface::instance();

  // Commands
  config->registerCommandHandlerFunction("arm_set_tool", arm_set_tool);
  config->registerCommandHandlerFunction("arm_tare_ft_sensor",
                                         arm_tare_ft_sensor);
  config->registerCommandHandlerFunction("arm_move_joints", arm_move_joints);
  config->registerCommandHandlerFunction("arm_move_joints_guarded",
                                 arm_move_joints_guarded);
  config->registerCommandHandlerFunction("task_scoop_circular",
                                         task_scoop_circular);
  config->registerCommandHandlerFunction("task_scoop_linear", task_scoop_linear);
  config->registerCommandHandlerFunction("task_psp", task_psp);
  config->registerCommandHandlerFunction("task_penetrometer", task_penetrometer);
  config->registerCommandHandlerFunction("task_shear_bevameter",
                                         task_shear_bevameter);

  // Telemetry
  config->registerLookupHandlerFunction("UsingOWLAT", using_owlat);
  config->registerLookupHandlerFunction("UsingOceanWATERS", using_oceanwaters);
  config->registerLookupHandlerFunction("ArmTool", arm_tool);
  config->setDefaultLookupHandler(default_lookup_handler);

  debugMsg("OwlatAdapter", " initialized.");
  return LanderAdapter::initialize (config);
}

extern "C" {
  void initowlat_adapter() {
    REGISTER_ADAPTER(OwlatAdapter, "owlat_adapter");
  }
}
