// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implementation of PLEXIL interface adapter for OceanWATERS.

// OW
#include "OwAdapter.h"
#include "OwInterface.h"
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
using namespace PLEXIL;

// C++
#include <string>
using std::string;
using std::vector;
using std::unique_ptr;

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

static bool lookup (const string& state_name,
                    const vector<PLEXIL::Value>& args,
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

  else if (state_name == "UsingOceanWATERS") {
    value_out = true;
  }
  else if (state_name == "UsingOWLAT") {
    value_out = false;
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
  else if (state_name == "AntennaPanFault") {
    value_out = OwInterface::instance()->antennaPanFault();
  }
  else if (state_name == "AntennaTiltFault") {
    value_out = OwInterface::instance()->antennaTiltFault();
  }
  else if (state_name == "ArmFault") {
    value_out = OwInterface::instance()->armFault();
  }
  else if (state_name == "PowerFault") {
    value_out = OwInterface::instance()->powerFault();
  }
  else if (state_name == "CameraFault") {
    value_out = OwInterface::instance()->cameraFault();
  }
  else if (state_name == "ArmEndEffectorForceTorque") {
    vector<double> ft = OwInterface::instance()->getEndEffectorFT();
    value_out = (Value) ft;
  }
  else if (state_name == "ActionGoalStatus") {
    string s;
    args[0].getValue(s);
    value_out = OwInterface::instance()->actionGoalStatus(s);
  }
  else if (state_name == "AnglesEquivalent") {
    double deg1, deg2, tolerance;
    args[0].getValue(deg1);
    args[1].getValue(deg2);
    args[2].getValue(tolerance);
    value_out = OwInterface::instance()->anglesEquivalent (deg1, deg2, tolerance);
  }
  else retval = false;

  return retval;
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
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->guardedMove (x, y, z, dir_x, dir_y, dir_z,
                                        search_distance, g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void arm_move_joints (Command *cmd, AdapterExecInterface* intf)
{
  bool relative;
  vector<double> const *angles_vector = nullptr;
  RealArray const *angles = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(angles);
  //changes real array into a vector
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->armMoveJoints (relative, *angles_vector,
                                          g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void arm_move_joints_guarded (Command *cmd, AdapterExecInterface* intf)
{
  bool relative;
  vector<double> const *angles_vector = nullptr;
  double force_threshold, torque_threshold;
  RealArray const *angles = nullptr;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(relative);
  args[1].getValuePointer(angles);
  args[2].getValue(force_threshold);
  args[3].getValue(torque_threshold);
  //changes real array into a vector
  angles->getContentsVector(angles_vector);
  std::unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->armMoveJointsGuarded (relative, *angles_vector,
                                                 force_threshold,
                                                 torque_threshold,
                                                 g_cmd_id);
  acknowledge_command_sent(*cr);
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
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->grind(x, y, depth, length, parallel, ground_pos,
                                 g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void scoop_circular (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  double x, y, z, depth;
  bool parallel, relative;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValue(x);
  args[3].getValue(y);
  args[4].getValue(z);
  args[5].getValue(depth);
  args[6].getValue(parallel);
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->scoopCircular(frame, relative, x, y, z, depth,
                                         parallel, g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void scoop_linear (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  bool relative;
  double x, y, z, depth, length;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue(frame);
  args[1].getValue(relative);
  args[2].getValue(x);
  args[3].getValue(y);
  args[4].getValue(z);
  args[5].getValue(depth);
  args[6].getValue(length);
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->scoopLinear(frame, relative, x, y, z, depth, length,
                                       g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void pan (Command* cmd, AdapterExecInterface* intf)
{
  double degrees;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (degrees);
  if (! LanderAdapter::checkAngle ("pan", degrees,
                                   LanderAdapter::PanMinDegrees,
                                   LanderAdapter::PanMaxDegrees,
                                   LanderAdapter::PanTiltInputTolerance)) {
    acknowledge_command_denied (cmd, intf);
  }
  else {
    unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
    OwInterface::instance()->pan (degrees, g_cmd_id);
    acknowledge_command_sent(*cr);
  }
}

static void tilt (Command* cmd, AdapterExecInterface* intf)
{
  double degrees;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (degrees);
  if (! LanderAdapter::checkAngle ("tilt", degrees, LanderAdapter::TiltMinDegrees,
                                   LanderAdapter::TiltMaxDegrees,
                                   LanderAdapter::PanTiltInputTolerance)) {
    acknowledge_command_denied (cmd, intf);
  }
  else {
    unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
    OwInterface::instance()->tilt (degrees, g_cmd_id);
    acknowledge_command_sent(*cr);
  }
}

static void pan_tilt_cartesian (Command* cmd, AdapterExecInterface* intf)
{
  int frame;
  double x, y, z;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (frame);
  args[1].getValue (x);
  args[2].getValue (y);
  args[3].getValue (z);
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->panTiltCartesian (frame, x, y, z, g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void camera_set_exposure (Command* cmd, AdapterExecInterface* intf)
{
  double exposure_secs;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (exposure_secs);
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->cameraSetExposure (exposure_secs, g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void dock_ingest_sample (Command* cmd, AdapterExecInterface* intf)
{
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  OwInterface::instance()->dockIngestSample (g_cmd_id);
  acknowledge_command_sent(*cr);
}

static void light_set_intensity (Command* cmd, AdapterExecInterface* intf)
{
  bool valid_args = true;
  string side;
  double intensity;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (side);
  args[1].getValue (intensity);
  if (side != "left" && side != "right") {
    ROS_ERROR ("light_set_intensity: side was %s, should be 'left' or 'right'",
               side.c_str());
    valid_args = false;
  }
  if (intensity < 0.0 || intensity > 1.0) {
    ROS_ERROR ("light_set_intensity: intensity was %f, "
               "should be in range [0.0 1.0]", intensity);
    valid_args = false;
  }
  if (valid_args) {
    unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
    OwInterface::instance()->lightSetIntensity (side, intensity, g_cmd_id);
    acknowledge_command_sent(*cr);
  }
  else {
    acknowledge_command_denied (cmd, intf);
  }
}

static void identify_sample_location (Command* cmd, AdapterExecInterface* intf)
{
  int num_pictures;
  std::string filter_type;
  const vector<Value>& args = cmd->getArgValues();
  args[0].getValue (num_pictures);
  args[1].getValue (filter_type);
  unique_ptr<CommandRecord>& cr = new_command_record(cmd, intf);
  vector<double> point_vector;
  // Get our sample point from identifySampleLocation
  point_vector = OwInterface::instance()->identifySampleLocation (num_pictures,
                                                                  filter_type,
                                                                  g_cmd_id);
  // Checks if we have a valid sized point
  if(point_vector.size() != 3){
    // Resizes the array and initializes values to -500 so we know to skip
    point_vector.resize(1,-500);
  }
  // Contstruct a Value type vector for plexil before returning the result
  Value value_point_vector = Value(point_vector);
  intf->handleCommandReturn(cmd, value_point_vector);
  acknowledge_command_sent(*cr);
}


// Telemetry

OwAdapter::OwAdapter(AdapterExecInterface& execInterface,
                     const pugi::xml_node& configXml)
  : LanderAdapter(execInterface, configXml)
{
  debugMsg("OwAdapter", " created.");
}

bool OwAdapter::initialize()
{
  LanderAdapter::initialize (OwInterface::instance());

  // Commands
  g_configuration->registerCommandHandler("grind", grind);
  g_configuration->registerCommandHandler("guarded_move", guarded_move);
  g_configuration->registerCommandHandler("arm_move_joints", arm_move_joints);
  g_configuration->registerCommandHandler("arm_move_joints_guarded",
                                          arm_move_joints_guarded);
  g_configuration->registerCommandHandler("dock_ingest_sample", dock_ingest_sample);
  g_configuration->registerCommandHandler("pan", pan);
  g_configuration->registerCommandHandler("tilt", tilt);
  g_configuration->registerCommandHandler("scoop_circular", scoop_circular);
  g_configuration->registerCommandHandler("scoop_linear", scoop_linear);
  g_configuration->registerCommandHandler("pan_tilt_cartesian",
                                          pan_tilt_cartesian);
  g_configuration->registerCommandHandler("identify_sample_location",
                                          identify_sample_location);
  g_configuration->registerCommandHandler("camera_set_exposure",
                                          camera_set_exposure);
  g_configuration->registerCommandHandler("light_set_intensity",
                                          light_set_intensity);
  debugMsg("OwAdapter", " initialized.");
  return true;
}

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

extern "C" {
  void initow_adapter() {
    REGISTER_ADAPTER(OwAdapter, "ow_adapter");
  }
}
