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
#include <AdapterFactory.hh>
#include <LookupReceiver.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
using namespace PLEXIL;

// C++
#include <string>
using std::string;
using std::vector;
using std::unique_ptr;

//////////////////////// PLEXIL Lookup Support //////////////////////////////


static void hard_torque_limit_reached (const State& s, LookupReceiver* r)
{
  string joint;
  s.parameters()[0].getValue(joint);
  r->update(OwInterface::instance()->hardTorqueLimitReached(joint));
}

static void soft_torque_limit_reached (const State& s, LookupReceiver* r)
{
  string joint;
  s.parameters()[0].getValue(joint);
  r->update(OwInterface::instance()->softTorqueLimitReached(joint));
}

static void running (const State& s, LookupReceiver* r)
{
  string action;
  s.parameters()[0].getValue(action);
  r->update(OwInterface::instance()->running(action));
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
                     PLEXIL::AdapterConf* conf)
  : LanderAdapter(execInterface, conf)
{
  debugMsg("OwAdapter", " created.");
}

bool OwAdapter::initialize (AdapterConfiguration* config)
{
  LanderAdapter::s_interface = OwInterface::instance();

  // Commands
  config->registerCommandHandlerFunction("grind", grind);
  config->registerCommandHandlerFunction("guarded_move", guarded_move);
  config->registerCommandHandlerFunction("arm_move_joints", arm_move_joints);
  config->registerCommandHandlerFunction("arm_move_joints_guarded",
                                 arm_move_joints_guarded);
  config->registerCommandHandlerFunction("dock_ingest_sample",
                                         dock_ingest_sample);
  config->registerCommandHandlerFunction("pan", pan);
  config->registerCommandHandlerFunction("tilt", tilt);
  config->registerCommandHandlerFunction("scoop_circular", scoop_circular);
  config->registerCommandHandlerFunction("scoop_linear", scoop_linear);
  config->registerCommandHandlerFunction("pan_tilt_cartesian",
                                         pan_tilt_cartesian);
  config->registerCommandHandlerFunction("identify_sample_location",
                                         identify_sample_location);
  config->registerCommandHandlerFunction("camera_set_exposure",
                                         camera_set_exposure);
  config->registerCommandHandlerFunction("light_set_intensity",
                                         light_set_intensity);

  // Lookups

  // Stubs specific to EuropaMission.plp
  config->registerLookupHandlerFunction("TrenchIdentified",
					lookupHandler<bool>(true));
  config->registerLookupHandlerFunction("ExcavationTimeout",
					lookupHandler<int>(10));
  config->registerLookupHandlerFunction("CollectAndTransferTimeout",
					lookupHandler<int>(10));

  // Plan interface specific to OceanWATERS
  config->registerLookupHandlerFunction("UsingOWLAT",
					lookupHandler<bool>(false));
  config->registerLookupHandlerFunction("UsingOceanWATERS",
					lookupHandler<bool>(true));
  config->registerLookupHandlerFunction("HardTorqueLimitReached",
					hard_torque_limit_reached);
  config->registerLookupHandlerFunction("SoftTorqueLimitReached",
					soft_torque_limit_reached);
  config->registerLookupHandlerFunction("GroundFound",
					lookupHandler<bool>
                                        (OwInterface::instance()->groundFound()));
  config->registerLookupHandlerFunction("GroundPosition",
					lookupHandler<double>
                                        (OwInterface::instance()->groundPosition()));
  config->registerLookupHandlerFunction("Running", running);

  // Faults specific to OceanWATERS
  config->registerLookupHandlerFunction("SystemFault",
					lookupHandler<bool>
                                        (OwInterface::instance()->systemFault()));
  config->registerLookupHandlerFunction("AntennaFault",
					lookupHandler<bool>
                                        (OwInterface::instance()->antennaFault()));
  config->registerLookupHandlerFunction("AntennaPanFault",
					lookupHandler<bool>
                                        (OwInterface::instance()->antennaPanFault()));
  config->registerLookupHandlerFunction("AntennaTiltFault",
					lookupHandler<bool>
                                        (OwInterface::instance()->antennaTiltFault()));
  config->registerLookupHandlerFunction("ArmFault",
					lookupHandler<bool>
                                        (OwInterface::instance()->armFault()));
  config->registerLookupHandlerFunction("PowerFault",
					lookupHandler<bool>
                                        (OwInterface::instance()->powerFault()));
  config->registerLookupHandlerFunction("CameraFault",
					lookupHandler<bool>
                                        (OwInterface::instance()->cameraFault()));

  debugMsg("OwAdapter", " initialized.");
  return LanderAdapter::initialize (config);
}

extern "C" {
  void initow_adapter() {
    REGISTER_ADAPTER(OwAdapter, "ow_adapter");
  }
}
