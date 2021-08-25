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
  int frame;
  bool relative;
  vector<double> const *position_vector = NULL;
  vector<double> const *orientation_vector = NULL;
  RealArray const *position = NULL;
  RealArray const *orientation = NULL;
  // Get our Frame, relative, position and orientation
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(position);
  args[3].getValuePointer(orientation);
  //change real array into a vector
  position->getContentsVector(position_vector);
  orientation->getContentsVector(orientation_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveCartesian (frame, relative, *position_vector, 
                                                     *orientation_vector, CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_move_cartesian_guarded (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative, retracting;
  double force_threshold, torque_threshold;
  vector<double> const *position_vector = NULL;
  vector<double> const *orientation_vector = NULL;
  RealArray const *position = NULL;
  RealArray const *orientation = NULL;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[4].getValue(retracting);
  args[5].getValue(force_threshold);
  args[6].getValue(torque_threshold);
  args[2].getValuePointer(position);
  args[3].getValuePointer(orientation);
    //change real array into a vector
  position->getContentsVector(position_vector);
  orientation->getContentsVector(orientation_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveCartesianGuarded (frame, relative, 
                                      *position_vector, *orientation_vector, 
                                      retracting, force_threshold, torque_threshold,
                                      CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_move_joint (Command* cmd, AdapterExecInterface* intf)
{
  bool relative;
  int joint;
  double angle; //radians
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValue(joint);
  args[2].getValue(angle);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveJoint (relative, joint, angle, CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_move_joints (Command* cmd, AdapterExecInterface* intf)
{
  bool relative;
  vector<double> const *angles_vector = NULL;
  RealArray const *angles = NULL;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(joints);
  //change real array into a vector
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveJoints (relative, *angles_vector,CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_move_joints_guarded (Command* cmd, AdapterExecInterface* intf)
{
  bool relative, retracting;
  double force_threshold, torque_threshold;
  vector<double> const *angles_vector = NULL;
  RealArray const *angles = NULL;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(angles);
  args[2].getValue(retracting);
  args[3].getValue(force_threshold);
  args[4].getValue(torque_threshold);
  //change real array into a vector
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmMoveJointsGuarded (relative, *joints_vector,
                                    retracting, force_threshold, torque_threshold,
                                    CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_place_tool (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative, retracting;
  double force_threshold, torque_threshold, distance, overdrive;
  vector<double> const *position_vector = NULL;
  vector<double> const *normal_vector = NULL;
  RealArray const *position = NULL;
  RealArray const *normal = NULL;
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
                                      *position_vector, *normal_vector, 
                                      distance, overdrive, retracting,
                                      force_threshold, torque_threshold,
                                      CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_set_tool (Command* cmd, AdapterExecInterface* intf)
{
  int tool;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(tool);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmSetTool (tool, CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_stop (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmStop (CommandId);
  send_ack_once(*cr);
}

static void owlat_arm_tare_fs (Command* cmd, AdapterExecInterface* intf)
{
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatArmTareFS (CommandId);
  send_ack_once(*cr);
}

static void owlat_task_dropoff (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  vector<double> const *point_vector = NULL;
  RealArray const *point = NULL;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(point);
  //change real array into a vector
  point->getContentsVector(point_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatTaskDropoff (frame, relative, *point_vector, 
                                                CommandId);
  send_ack_once(*cr);
}

static void owlat_task_psp (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  double max_depth, max_force;
  vector<double> const *point_vector = NULL;
  vector<double> const *normal_vector = NULL;
  RealArray const *point = NULL;
  RealArray const *normal = NULL;
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
  send_ack_once(*cr);
}

static void owlat_task_scoop (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  vector<double> const *point_vector = NULL;
  vector<double> const *normal_vector = NULL;
  RealArray const *point = NULL;
  RealArray const *normal = NULL;
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
  send_ack_once(*cr);
}

static void owlat_task_shear_bevameter (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  double preload, max_torque;
  vector<double> const *point_vector = NULL;
  vector<double> const *normal_vector = NULL;
  RealArray const *point = NULL;
  RealArray const *normal = NULL;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValuePointer(point);
  args[3].getValuePointer(normal);
  args[4].getValue(preload);
  args[5].getValue(max_torque);
  //change real array into a vector
  point->getContentsVector(point_vector);
  normal->getContentsVector(normal_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwlatInterface::instance()->owlatTaskShearBevameter (frame, relative,
                                                *point_vector,*normal_vector, 
                                                preload, max_torque,
                                                CommandId);
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
  g_configuration->registerCommandHandler("owlat_arm_move_cartesian_guarded", owlat_arm_move_cartesian_guarded);
  g_configuration->registerCommandHandler("owlat_arm_move_joint", owlat_arm_move_joint);
  g_configuration->registerCommandHandler("owlat_arm_move_joints", owlat_arm_move_joints);
  g_configuration->registerCommandHandler("owlat_arm_move_joints_guarded", owlat_arm_move_joints_guarded);
  g_configuration->registerCommandHandler("owlat_arm_place_tool", owlat_arm_place_tool);
  g_configuration->registerCommandHandler("owlat_arm_set_tool", owlat_arm_set_tool);
  g_configuration->registerCommandHandler("owlat_arm_stop", owlat_arm_stop);
  g_configuration->registerCommandHandler("owlat_arm_tare_fs", owlat_arm_tare_fs);
  g_configuration->registerCommandHandler("owlat_task_dropoff", owlat_task_dropoff);
  g_configuration->registerCommandHandler("owlat_task_psp", owlat_task_psp);
  g_configuration->registerCommandHandler("owlat_task_scoop", owlat_task_scoop);
  g_configuration->registerCommandHandler("owlat_task_shear_bevameter", owlat_task_shear_bevameter);
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
