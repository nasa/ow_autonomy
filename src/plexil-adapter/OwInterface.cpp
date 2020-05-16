// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// ow_autonomy
#include "OwInterface.h"
#include "subscriber.h"
#include "joint_support.h"

// OW - other
#include <ow_lander/StartPlanning.h>
#include <ow_lander/MoveGuarded.h>
#include <ow_lander/DigTrench.h>
#include <ow_lander/PublishTrajectory.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>

// C++
#include <set>
#include <map>
#include <thread>
using std::set;
using std::map;
using std::thread;

// C
#include <cmath>  // for M_PI and abs

// Degree/Radian
const double D2R = M_PI / 180.0 ;
const double R2D = 180.0 / M_PI ;


//////////////////// Lander Operation Support ////////////////////////

// Lander operation names.
// In some cases, these must match those used in PLEXIL and/or ow_lander
const string Op_MoveGuarded       = "MoveGuarded";
const string Op_MoveGuardedAction = "MoveGuardedAction";
const string Op_DigTrench         = "DigTrench";
const string Op_ArmPlanning       = "StartPlanning";
const string Op_PublishTrajectory = "PublishTrajectory";
const string Op_PanAntenna        = "PanAntenna";
const string Op_TiltAntenna       = "TiltAntenna";

static map<string, bool> Running
{
  { Op_MoveGuarded, false },
  { Op_MoveGuardedAction, false },
  { Op_DigTrench, false },
  { Op_ArmPlanning, false },
  { Op_PublishTrajectory, false },
  { Op_PanAntenna, false },
  { Op_TiltAntenna, false }
};

static bool is_lander_operation (const string& name)
{
  return Running.find (name) != Running.end();
}

static bool mark_operation_running (const string& name)
{
  if (Running.at (name)) {
    ROS_WARN ("%s already running, ignoring duplicate request.", name.c_str());
    return false;
  }

  ROS_INFO ("Marking %s RUNNING", name.c_str());
  Running.at (name) = true;
  publish ("Finished", false, name);
  publish ("Running", true, name);
  return true;
}

static void mark_operation_finished (const string& name)
{
  if (! Running.at (name)) {
    ROS_WARN ("%s was not running. Should never happen.", name.c_str());
  }
  ROS_INFO ("Marking %s FINISHED", name.c_str());
  Running.at (name) = false;
  publish ("Running", false, name);
  publish ("Finished", true, name);
}


//////////////////// Fault Support ////////////////////////

// NOTE: the design goal is to map each lander operation to the set of faults
// that should be monitored while it is running.  This direct inspection of ROS
// parameters is just a simple first cut (stub really) for actual fault
// detection which would look at telemetry.

// NOTE: The following three maps *should* be thread-safe, because C++11 docs
// says they are.  By definition, they are only read.

const map<string, string> AntennaFaults
{
  // Param name -> human-readable
  { "/faults/ant_pan_encoder_failure", "Antenna Pan Encoder" },
  { "/faults/ant_tilt_encoder_failure", "Antenna Tilt Encoder" },
  { "/faults/ant_pan_torque_sensor_failure", "Antenna Pan Torque Sensor" },
  { "/faults/ant_tilt_torque_sensor_failure", "Antenna Tilt Torque Sensor" }
};

const map<string, string> ArmFaults
{
  // Param name -> human-readable
  { "/faults/shou_yaw_encoder_failure", "Shoulder Yaw Encoder" },
  { "/faults/shou_pitch_encoder_failure", "Shoulder Pitch Encoder" },
  { "/faults/shou_pitch_torque_sensor_failure", "Shoulder Pitch Torque Sensor" },
  { "/faults/prox_pitch_encoder_failure", "Proximal Pitch Encoder" },
  { "/faults/prox_pitch_torque_sensor_failure", "Proximal Pitch Torque Sensor" },
  { "/faults/dist_pitch_encoder_failure", "Distal Pitch Encoder" },
  { "/faults/dist_pitch_torque_sensor_failure", "Distal Pitch Torque Sensor" },
  { "/faults/hand_yaw_encoder_failure", "Hand Yaw Encoder" },
  { "/faults/hand_yaw_torque_sensor_failure", "Hand Yaw Torque Sensor" },
  { "/faults/scoop_yaw_encoder_failure", "Scoop Yaw Encoder" },
  { "/faults/scoop_yaw_torque_sensor_failure", "Scoop Yaw Torque Sensor" }
};

const map<string, map<string, string> > Faults
{
  // Map each lander operation to its relevant fault set.
  { Op_MoveGuarded, ArmFaults },
  { Op_MoveGuardedAction, ArmFaults },
  { Op_DigTrench, ArmFaults },
  { Op_ArmPlanning, ArmFaults },
  { Op_PublishTrajectory, ArmFaults },
  { Op_PanAntenna, AntennaFaults },
  { Op_TiltAntenna, AntennaFaults }
};

static bool faulty (const string& fault)
{
  bool val;
  ros::param::get (fault, val);
  return val;
}

static void monitor_for_faults (const string& opname)
{
  using namespace std::chrono_literals;
  while (Running.at (opname)) {
    ROS_DEBUG ("Monitoring for faults in %s", opname.c_str());
    for (auto fault : Faults.at (opname)) {
      if (faulty (fault.first)) {
        ROS_WARN("Fault in %s: %s failure.",
                 opname.c_str(), fault.second.c_str());
      }
    }
    std::this_thread::sleep_for (1s);
  }
}


/////////////////// ROS Service support //////////////////////

template<class Service>
static void service_call (ros::ServiceClient client, Service srv, string name)
{
  // NOTE: arguments are copies because this function is called in a thread that
  // outlives its caller.  Assumes that service is not already running; this is
  // checked upstream.

  thread fault_thread (monitor_for_faults, name);
  if (client.call (srv)) { // blocks
    ROS_INFO("%s returned: %d, %s", name.c_str(), srv.response.success,
             srv.response.message.c_str());  // make DEBUG later
  }
  else {
    ROS_ERROR("Failed to call service %s", name.c_str());
  }
  mark_operation_finished (name);
  fault_thread.join();
}

static bool check_service_client (ros::ServiceClient& client)
{
  if (! client.exists()) {
    ROS_ERROR("Service client does not exist!");
    return false;
  }

  if (! client.isValid()) {
    ROS_ERROR("Service client is invalid!");
    return false;
  }

  return true;
}


/////////////////////////// Joint/Torque Support ///////////////////////////////

static set<string> JointsAtHardTorqueLimit { };
static set<string> JointsAtSoftTorqueLimit { };

static map<string, Joint> JointMap {
  // ROS JointStates message name -> type
  { "j_shou_yaw", Joint::shoulder_yaw },
  { "j_shou_pitch", Joint::shoulder_pitch },
  { "j_prox_pitch", Joint::proximal_pitch },
  { "j_dist_pitch", Joint::distal_pitch },
  { "j_hand_yaw", Joint::hand_yaw },
  { "j_scoop_yaw", Joint::scoop_yaw },
  { "j_ant_pan", Joint::antenna_pan },
  { "j_ant_tilt", Joint::antenna_tilt }
};

static map<Joint, JointProperties> JointPropMap {
  // NOTE: Torque limits are made up, and there may be a better place for these
  // later.  Assuming that only magnitude matters.

  { Joint::shoulder_yaw,   { "j_shou_yaw", "ShoulderYaw", 60, 80 }},
  { Joint::shoulder_pitch, { "j_shou_pitch", "ShoulderPitch", 60, 80 }},
  { Joint::proximal_pitch, { "j_prox_pitch", "ProximalPitch", 60, 80 }},
  { Joint::distal_pitch,   { "j_dist_pitch", "DistalPitch", 60, 80 }},
  { Joint::hand_yaw,       { "j_hand_yaw", "HandYaw", 60, 80 }},
  { Joint::scoop_yaw,      { "j_scoop_yaw", "ScoopYaw", 60, 80 }},
  { Joint::antenna_pan,    { "j_ant_pan", "AntennaPan", 30, 30 }},
  { Joint::antenna_tilt,   { "j_ant_tilt", "AntennaTilt", 30, 30 }}
};

static map<Joint, JointTelemetry> JointTelemetryMap { };

static void handle_overtorque (Joint joint, double effort)
{
  // For now, torque is just effort (Newton-meter), and overtorque is specific
  // to the joint.

  string joint_name = JointPropMap[joint].plexilName;

  if (abs(effort) >= JointPropMap[joint].hardTorqueLimit) {
    JointsAtHardTorqueLimit.insert (joint_name);
  }
  else if (abs(effort) >= JointPropMap[joint].softTorqueLimit) {
    JointsAtSoftTorqueLimit.insert(joint_name);
  }
  else {
    JointsAtHardTorqueLimit.erase (joint_name);
    JointsAtSoftTorqueLimit.erase (joint_name);
  }
}

static void handle_joint_fault (Joint joint, int joint_index,
                                const sensor_msgs::JointState::ConstPtr& msg)
{
  // NOTE: For now, the only fault is overtorque.
  handle_overtorque (joint, msg->effort[joint_index]);
}

void OwInterface::jointStatesCallback
(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Publish all joint information for visibility to PLEXIL and handle any
  // joint-related faults.

  for (int i = 0; i < JointMap.size(); i++) {
    string ros_name = msg->name[i];
    if (JointMap.find (ros_name) != JointMap.end()) {
      Joint joint = JointMap[ros_name];
      JointTelemetryMap[joint] = JointTelemetry (msg->position[i],
                                                 msg->velocity[i],
                                                 msg->effort[i]);
      string plexil_name = JointPropMap[joint].plexilName;
      publish (plexil_name + "Velocity", msg->velocity[i]);
      publish (plexil_name + "Effort", msg->effort[i]);
      publish (plexil_name + "Position", msg->position[i]);
      handle_joint_fault (joint, i, msg);
    }
    else ROS_ERROR("jointStatesCallback: unsupported joint %s",
                   ros_name.c_str());
  }
}


////////////////////////////// Image Support ///////////////////////////////////

static double CurrentTilt         = 0.0;
static double CurrentPanDegrees   = 0.0;
static bool   ImageReceived       = false;

static void pan_callback
(const control_msgs::JointControllerState::ConstPtr& msg)
{
  CurrentPanDegrees = msg->set_point * R2D;
  publish ("PanDegrees", CurrentPanDegrees);
}

static void tilt_callback
(const control_msgs::JointControllerState::ConstPtr& msg)
{
  CurrentTilt = msg->set_point * R2D;
  publish ("TiltDegrees", CurrentTilt);
}

static void camera_callback (const sensor_msgs::Image::ConstPtr& msg)
{
  // Assuming that receipt of this message is success itself.
  ImageReceived = true;
  publish ("ImageReceived", ImageReceived);
}


//////////////////// MoveGuarded Action support ////////////////////////////////

// At present, this is a prototypical action using a dummy server in this
// directory, MoveGuardedServer.

static void move_guarded_done_cb
(const actionlib::SimpleClientGoalState& state,
 const ow_autonomy::MoveGuardedResultConstPtr& result)
{
  ROS_INFO ("Finished in state [%s]", state.toString().c_str());
  ROS_INFO ("Answer: %s", result->message.c_str());
}

static void move_guarded_active_cb ()
{
  ROS_INFO ("Goal just went active");
}

static void move_guarded_feedback_cb
(const ow_autonomy::MoveGuardedFeedbackConstPtr& feedback)
{
  ROS_INFO ("Feedback: (%f, %f, %f)",
            feedback->current_x, feedback->current_y, feedback->current_z);
}


/////////////////////////// OwInterface members ////////////////////////////////

OwInterface* OwInterface::m_instance = nullptr;

OwInterface* OwInterface::instance ()
{
  // Very simple singleton
  if (m_instance == nullptr) m_instance = new OwInterface();
  return m_instance;
}

OwInterface::OwInterface ()
  : m_genericNodeHandle (nullptr),
    m_antennaTiltPublisher (nullptr),
    m_antennaPanPublisher (nullptr),
    m_leftImageTriggerPublisher (nullptr),
    m_antennaTiltSubscriber (nullptr),
    m_antennaPanSubscriber (nullptr),
    m_jointStatesSubscriber (nullptr),
    m_cameraSubscriber (nullptr),
    m_moveGuardedClient ("MoveGuarded", true)
{
  ROS_INFO ("Waiting for action servers...");
  m_moveGuardedClient.waitForServer();
  ROS_INFO ("Action servers available.");
}

OwInterface::~OwInterface ()
{
  if (m_genericNodeHandle) delete m_genericNodeHandle;
  if (m_antennaTiltPublisher) delete m_antennaTiltPublisher;
  if (m_leftImageTriggerPublisher) delete m_leftImageTriggerPublisher;
  if (m_antennaTiltSubscriber) delete m_antennaTiltSubscriber;
  if (m_antennaPanSubscriber) delete m_antennaPanSubscriber;
  if (m_jointStatesSubscriber) delete m_jointStatesSubscriber;
  if (m_cameraSubscriber) delete m_cameraSubscriber;
  if (m_instance) delete m_instance;
}

void OwInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {
    m_genericNodeHandle = new ros::NodeHandle();

    // Initialize publishers.  Queue size is a guess at adequacy.  For now,
    // latching in lieu of waiting for publishers.

    const int qsize = 3;
    const bool latch = true;
    m_antennaTiltPublisher = new ros::Publisher
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_tilt_position_controller/command", qsize, latch));
    m_antennaPanPublisher = new ros::Publisher
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_pan_position_controller/command", qsize, latch));
    m_leftImageTriggerPublisher = new ros::Publisher
      (m_genericNodeHandle->advertise<std_msgs::Empty>
       ("/StereoCamera/left/image_trigger", qsize, latch));

    // Initialize subscribers

    m_antennaTiltSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/ant_tilt_position_controller/state", qsize, tilt_callback));
    m_antennaPanSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/ant_pan_position_controller/state", qsize, pan_callback));
    m_jointStatesSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/joint_states", qsize, OwInterface::jointStatesCallback));
    m_cameraSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/StereoCamera/left/image_raw", qsize, camera_callback));
  }
}

void OwInterface::startPlanningDemo()
{
  if (! mark_operation_running (Op_ArmPlanning)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    // NOTE: typo is deliberate
    nhandle.serviceClient<ow_lander::StartPlanning>("start_plannning_session");

  if (check_service_client (client)) {
    ow_lander::StartPlanning srv;
    srv.request.use_defaults = true;
    srv.request.trench_x = 0.0;
    srv.request.trench_y = 0.0;
    srv.request.trench_d = 0.0;
    srv.request.delete_prev_traj = false;
    thread service_thread (service_call<ow_lander::StartPlanning>,
                           client, srv, Op_ArmPlanning);
    service_thread.detach();
  }
}

void OwInterface::moveGuardedAction() // temporary, proof of concept
{
  if (! mark_operation_running (Op_MoveGuardedAction)) return;

  thread t (&OwInterface::moveGuardedActionThread, this);
  t.detach();
}

void OwInterface::moveGuardedActionThread() // temporary, proof of concept
{
  ow_autonomy::MoveGuardedGoal goal;
  goal.use_defaults = true;
  m_moveGuardedClient.sendGoal (goal,
                                move_guarded_done_cb,
                                move_guarded_active_cb,
                                move_guarded_feedback_cb);

  // Wait for the action to return
  bool finished_before_timeout =
    m_moveGuardedClient.waitForResult (ros::Duration (30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = m_moveGuardedClient.getState();
    ROS_INFO("MoveGuarded action finished: %s", state.toString().c_str());
    ow_autonomy::MoveGuardedResultConstPtr result =
      m_moveGuardedClient.getResult();
    ROS_INFO("Action result message: %s", result->message.c_str());
  }
  else {
    ROS_INFO("MoveGuarded action did not finish before the time out.");
  }

  mark_operation_finished (Op_MoveGuardedAction);
}

void OwInterface::moveGuardedDemo()
{
  moveGuarded();
}

void OwInterface::moveGuarded (double target_x, double target_y, double target_z,
                               double surf_norm_x,
                               double surf_norm_y,
                               double surf_norm_z,
                               double offset_dist, double overdrive_dist,
                               bool delete_prev_traj,
                               bool retract)
{
  if (! mark_operation_running (Op_MoveGuarded)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::MoveGuarded>("start_move_guarded");

  if (check_service_client (client)) {
    ow_lander::MoveGuarded srv;
    srv.request.use_defaults = false;
    srv.request.target_x = target_x;
    srv.request.target_y = target_y;
    srv.request.target_z = target_z;
    srv.request.surface_normal_x = surf_norm_x;
    srv.request.surface_normal_y = surf_norm_y;
    srv.request.surface_normal_z = surf_norm_z;
    srv.request.offset_distance = offset_dist;
    srv.request.overdrive_distance = overdrive_dist;
    srv.request.retract = retract;
    thread service_thread (service_call<ow_lander::MoveGuarded>,
                           client, srv, Op_MoveGuarded);
    service_thread.detach();
  }
}

void OwInterface::publishTrajectoryDemo()
{
  if (! mark_operation_running (Op_PublishTrajectory)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::PublishTrajectory>("publish_trajectory");

  if (check_service_client (client)) {
    ow_lander::PublishTrajectory srv;
    srv.request.use_latest = true;
    srv.request.trajectory_filename = "ow_lander_trajectory.txt";
    thread service_thread (service_call<ow_lander::PublishTrajectory>,
                           client, srv, Op_PublishTrajectory);
    service_thread.detach();
  }
}

static bool antenna_op (const string& name, double degrees, ros::Publisher* pub)
{
  if (! mark_operation_running (name)) {
    return false;
  }

  std_msgs::Float64 radians;
  radians.data = degrees * D2R;
  ROS_INFO ("Starting %s: %f degrees (%f radians)", name.c_str(),
            degrees, radians.data);
  thread t (monitor_for_faults, name);
  t.detach();
  pub->publish (radians);
  return true;
}

bool OwInterface::tiltAntenna (double degrees)
{
  return antenna_op (Op_TiltAntenna, degrees, m_antennaTiltPublisher);
}

bool OwInterface::panAntenna (double degrees)
{
  return antenna_op (Op_PanAntenna, degrees, m_antennaPanPublisher);
}

void OwInterface::takePicture ()
{
  std_msgs::Empty msg;
  ImageReceived = false;
  publish ("ImageReceived", ImageReceived);
  m_leftImageTriggerPublisher->publish (msg);
}

void OwInterface::digTrench (double x, double y, double z,
                             double depth, double length, double width,
                             double pitch, double yaw,
                             double dumpx, double dumpy, double dumpz)
{
  if (! mark_operation_running (Op_DigTrench)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::DigTrench>("start_dig_trench_session");

  if (check_service_client (client)) {
    ow_lander::DigTrench srv;
    srv.request.use_defaults = false;
    srv.request.trench_x = x;
    srv.request.trench_y = y;
    srv.request.trench_d = depth;
    srv.request.length = length;
    srv.request.delete_prev_traj = false;
    thread service_thread (service_call<ow_lander::DigTrench>,
                           client, srv, Op_DigTrench);
    service_thread.detach();
  }
}

double OwInterface::getTilt () const
{
  return CurrentTilt;
}

double OwInterface::getPanDegrees () const
{
  return CurrentPanDegrees;
}

double OwInterface::getPanVelocity () const
{
  return JointTelemetryMap[Joint::antenna_pan].velocity;
}

double OwInterface::getTiltVelocity () const
{
  return JointTelemetryMap[Joint::antenna_tilt].velocity;
}

bool OwInterface::imageReceived () const
{
  return ImageReceived;
}

bool OwInterface::operationRunning (const string& name) const
{
  // Note: check in caller guarantees 'at' to return a valid value.
  return Running.at (name);
}

bool OwInterface::operationFinished (const string& name) const
{
  return !operationRunning (name);
}

bool OwInterface::running (const string& name) const
{
  if (is_lander_operation (name)) return operationRunning (name);

  ROS_ERROR("OwInterface::running: unsupported operation: %s", name.c_str());
  return false;
}

bool OwInterface::finished (const string& name) const
{
  if (is_lander_operation (name)) return operationFinished (name);

  ROS_ERROR("OwInterface::finished: unsupported operation: %s", name.c_str());
  return false;
}

bool OwInterface::hardTorqueLimitReached (const std::string& joint_name) const
{
  return (JointsAtHardTorqueLimit.find (joint_name) !=
          JointsAtHardTorqueLimit.end());
}

bool OwInterface::softTorqueLimitReached (const std::string& joint_name) const
{
  return (JointsAtSoftTorqueLimit.find (joint_name) !=
          JointsAtSoftTorqueLimit.end());
}

void OwInterface::stopOperation (const string& name) const
{
  mark_operation_finished (name);
}
