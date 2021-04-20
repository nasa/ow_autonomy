// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ow_autonomy
#include "OwInterface.h"
#include "subscriber.h"
#include "joint_support.h"

// OW - other
#include <ow_lander/DigCircular.h>
#include <ow_lander/DigLinear.h>
#include <ow_lander/Grind.h>
#include <ow_lander/Stow.h>
#include <ow_lander/Unstow.h>
#include <ow_lander/DeliverSample.h>
#include <ow_lander/GuardedMove.h>
#include <ow_lander/GuardedMoveResult.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

// C++
#include <set>
#include <map>
#include <thread>
#include <functional>
using std::set;
using std::map;
using std::thread;
using std::ref;

// C
#include <cmath>  // for M_PI and fabs

//////////////////// Utilities ////////////////////////

// Degree/Radian
const double D2R = M_PI / 180.0 ;
const double R2D = 180.0 / M_PI ;

const double DegreeTolerance = 0.2;    // made up, degees
const double VelocityTolerance = 0.01; // made up, unitless

static bool within_tolerance (double val1, double val2, double tolerance)
{
  return fabs (val1 - val2) <= tolerance;
}


//////////////////// Lander Operation Support ////////////////////////

static void (* CommandStatusCallback) (int,bool);

const double PanTiltTimeout = 5.0; // seconds, made up

// Lander operation names.
// In some cases, these must match those used in PLEXIL and/or ow_lander
const string Op_GuardedMove       = "GuardedMove";
const string Op_GuardedMoveAction = "GuardedMoveAction";
const string Op_DigCircular       = "DigCircular";
const string Op_DigLinear         = "DigLinear";
const string Op_DeliverSample     = "DeliverSample";
const string Op_PanAntenna        = "PanAntenna";
const string Op_TiltAntenna       = "TiltAntenna";
const string Op_Grind             = "Grind";
const string Op_Stow              = "Stow";
const string Op_Unstow            = "Unstow";
const string Op_TakePicture       = "TakePicture";

// NOTE: The following map *should* be thread-safe, according to C++11 docs and
// in particular because map entries are never added or deleted, and the code
// insures that each entry can be read/written by only one thread.  (The map
// itself can be read/written by multiple threads concurrently).

// Unused operation ID that signifies idle lander operation.
#define IDLE_ID (-1)

static map<string, int> Running
{
  { Op_GuardedMove, IDLE_ID },
  { Op_GuardedMoveAction, IDLE_ID },
  { Op_DigCircular, IDLE_ID },
  { Op_DigLinear, IDLE_ID },
  { Op_DeliverSample, IDLE_ID },
  { Op_PanAntenna, IDLE_ID },
  { Op_TiltAntenna, IDLE_ID },
  { Op_Grind, IDLE_ID },
  { Op_Stow, IDLE_ID },
  { Op_Unstow, IDLE_ID },
  { Op_TakePicture, IDLE_ID }
};

static bool is_lander_operation (const string& name)
{
  return Running.find (name) != Running.end();
}

static bool mark_operation_running (const string& name, int id)
{
  if (Running.at (name) != IDLE_ID) {
    ROS_WARN ("%s already running, ignoring duplicate request.", name.c_str());
    return false;
  }
  Running.at (name) = id;
  publish ("Running", true, name);
  return true;
}

static void mark_operation_finished (const string& name, int id)
{
  if (! Running.at (name) == IDLE_ID) {
    ROS_WARN ("%s was not running. Should never happen.", name.c_str());
  }
  Running.at (name) = IDLE_ID;
  publish ("Running", false, name);
  publish ("Finished", true, name);
  if (id != IDLE_ID) CommandStatusCallback (id, true);
}


//////////////////// Fault Support ////////////////////////

static void monitor_for_faults (const string& opname)
{
  // This (threaded) function was formerly used for operation-specific fault
  // monitoring, using a mechanism that has been removed, which was direct
  // inspection of the fault injection ROS parameters.  TBD whether it will be
  // used again, but leaving it here for now.

  //  using namespace std::chrono_literals;
  //  while (Running.at (opname) != IDLE_ID) {
  //    std::this_thread::sleep_for (1s);
  //  }
}


/////////////////// ROS Service support //////////////////////

template<class Service>
static void call_ros_service (ros::ServiceClient client, Service srv,
                              string name, int id)
{
  // NOTE: arguments are copies because this function is called in a thread that
  // outlives its caller.  Assumes that service is not already running; this is
  // checked upstream.

  ROS_INFO("  Starting ROS service %s", name.c_str());
  thread fault_thread (monitor_for_faults, name);
  if (client.call (srv)) { // blocks
    ROS_INFO("  %s returned: %d, %s", name.c_str(), srv.response.success,
             srv.response.message.c_str());  // make DEBUG later
  }
  else {
    ROS_ERROR("Failed to call service %s", name.c_str());
  }
  mark_operation_finished (name, id);
  fault_thread.join();
}

static bool check_service_client (ros::ServiceClient& client,
                                  const string& name)
{
  if (! client.exists()) {
    ROS_ERROR("Service client for %s does not exist!", name.c_str());
    return false;
  }

  if (! client.isValid()) {
    ROS_ERROR("Service client for %s is invalid!", name.c_str());
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
  { "j_ant_tilt", Joint::antenna_tilt },
  { "j_grinder", Joint::grinder }
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
  { Joint::antenna_tilt,   { "j_ant_tilt", "AntennaTilt", 30, 30 }},
  { Joint::grinder,        { "j_grinder", "Grinder", 30, 30 }}
};

static map<Joint, JointTelemetry> JointTelemetryMap { };

static void handle_overtorque (Joint joint, double effort)
{
  // For now, torque is just effort (Newton-meter), and overtorque is specific
  // to the joint.

  string joint_name = JointPropMap[joint].plexilName;

  if (fabs(effort) >= JointPropMap[joint].hardTorqueLimit) {
    JointsAtHardTorqueLimit.insert (joint_name);
  }
  else if (fabs(effort) >= JointPropMap[joint].softTorqueLimit) {
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

void OwInterface::systemFaultMessageCallback
(const  ow_faults::SystemFaults::ConstPtr& msg)
{
  // Publish all joint information for visibility to PLEXIL and handle any
  // system-level fault messages.
  uint64_t msg_val = msg->value;

  for (auto const& entry : m_systemErrors){
    string key = entry.first;
    uint64_t value = entry.second.first;
    bool b = entry.second.second;

    if (checkFaultMessages("SYSTEM", msg_val, key, value, b)) {
      m_systemErrors[key].second = !m_systemErrors[key].second;
    }

  }
}

void OwInterface::armFaultCallback(const  ow_faults::ArmFaults::ConstPtr& msg)
{
  // Publish all ARM COMPONENT FAULT information for visibility to PLEXIL and handle any
  // system-level fault messages.
  uint32_t msg_val = msg->value;

  for (auto const& entry : m_armErrors){
    string key = entry.first;
    uint32_t value = entry.second.first;
    bool b = entry.second.second;

    if (checkFaultMessages("ARM", msg_val, key, value, b)) {
      m_armErrors[key].second = !m_armErrors[key].second;
    }
  }
}

void OwInterface::powerFaultCallback(const  ow_faults::PowerFaults::ConstPtr& msg)
{
  // Publish all POWER FAULT information for visibility to PLEXIL and handle any
  // system-level fault messages.
  uint32_t msg_val = msg->value;

  for (auto const& entry : m_powerErrors){
    string key = entry.first;
    uint32_t value = entry.second.first;
    bool b = entry.second.second;

    if (checkFaultMessages("POWER", msg_val, key, value, b)) {
      m_powerErrors[key].second = !m_powerErrors[key].second;
    }
  }
}

void OwInterface::antennaFaultCallback(const  ow_faults::PTFaults::ConstPtr& msg)
{
  // Publish all PANT TILT ANTENNA information for visibility to PLEXIL and handle any
  // system-level fault messages.
  uint32_t msg_val = msg->value;

  for (auto const& entry : m_panTiltErrors){
    string key = entry.first;
    uint32_t value = entry.second.first;
    bool b = entry.second.second;

    if (checkFaultMessages("ANTENNA", msg_val, key, value, b)) {
      m_panTiltErrors[key].second = !m_panTiltErrors[key].second;
    }
  }
}

template <typename T>
bool OwInterface::checkFaultMessages(string fault_component, T msg_val,
                                     string key, T value, bool b )
{
  if (!b && ((msg_val & value) == value)){
    ROS_ERROR("%s ERROR: %s", fault_component.c_str(),  key.c_str() );
    return true;
  }
  else if (b && ((msg_val & value) != value)){
    ROS_INFO("RESOLVED %s ERROR: %s", fault_component.c_str(), key.c_str() );
    return true;
  }
  return false;
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
      double position = msg->position[i];
      double velocity = msg->velocity[i];
      double effort = msg->effort[i];
      if (joint == Joint::antenna_pan) {
        managePanTilt (Op_PanAntenna, position, velocity, m_currentPan,
                       m_goalPan, m_panStart);
      }
      else if (joint == Joint::antenna_tilt) {
        managePanTilt (Op_TiltAntenna, position, velocity, m_currentTilt,
                       m_goalTilt, m_tiltStart);
      }
      JointTelemetryMap[joint] = JointTelemetry (position, velocity, effort);
      string plexil_name = JointPropMap[joint].plexilName;
      publish (plexil_name + "Position", position);
      publish (plexil_name + "Velocity", velocity);
      publish (plexil_name + "Effort", effort);
      handle_joint_fault (joint, i, msg);
    }
    else ROS_ERROR("jointStatesCallback: unsupported joint %s",
                   ros_name.c_str());
  }
}

void OwInterface::managePanTilt (const string& opname,
                                 double position, double velocity,
                                 double current, double goal,
                                 const ros::Time& start)
{
  // We are only concerned when there is a pan/tilt in progress.
  if (! operationRunning (opname)) return;

  int id = Running.at (opname);

  // Antenna states of interest,
  bool reached = within_tolerance (current, goal, DegreeTolerance);
  bool expired = ros::Time::now() > start + ros::Duration (PanTiltTimeout);

  if (reached || expired) {
    mark_operation_finished (opname, id);
    if (expired) ROS_ERROR("%s timed out", opname.c_str());
    if (! reached) {
      ROS_ERROR("%s failed. Ended at %f degrees, goal was %f.",
                opname.c_str(), current, goal);
    }
  }
}


///////////////////////// Antenna/Camera Support ///////////////////////////////

void OwInterface::panCallback
(const control_msgs::JointControllerState::ConstPtr& msg)
{
  m_currentPan = msg->set_point * R2D;
  publish ("PanDegrees", m_currentPan);
}

void OwInterface::tiltCallback
(const control_msgs::JointControllerState::ConstPtr& msg)
{
  m_currentTilt = msg->set_point * R2D;
  publish ("TiltDegrees", m_currentTilt);
}

void OwInterface::cameraCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  // NOTE: the received image is ignored for now.

  if (operationRunning (Op_TakePicture)) {
    mark_operation_finished (Op_TakePicture, Running.at (Op_TakePicture));
  }
}


///////////////////////// Power support /////////////////////////////////////

static double StateOfCharge       = NAN;
static double RemainingUsefulLife = NAN;
static double BatteryTemperature  = NAN;

static void soc_callback (const std_msgs::Float64::ConstPtr& msg)
{
  StateOfCharge = msg->data;
  publish ("StateOfCharge", StateOfCharge);
}

static void rul_callback (const std_msgs::Int16::ConstPtr& msg)
{
  // NOTE: This is not being called as of 4/12/21.  Jira OW-656 addresses.
  RemainingUsefulLife = msg->data;
  publish ("RemainingUsefulLife", RemainingUsefulLife);
}

static void temperature_callback (const std_msgs::Float64::ConstPtr& msg)
{
  BatteryTemperature = msg->data;
  publish ("BatteryTemperature", BatteryTemperature);
}


//////////////////// GuardedMove Service support ////////////////////////////////

// TODO: encapsulate GroundFound and GroundPosition within the GuardedMove
// operation: it is not meaningful otherwise, and can be possibly misused given
// the current plan interface.

static bool GroundFound = false;
static double GroundPosition = 0; // should not be queried unless GroundFound

bool OwInterface::groundFound () const
{
  return GroundFound;
}

double OwInterface::groundPosition () const
{
  return GroundPosition;
}

static void guarded_move_callback
(const ow_lander::GuardedMoveResult::ConstPtr& msg)
{
  if (msg->success) {
    GroundFound = true;
    string frame = msg->frame;
    string valid = "base_link";
    if (frame != valid) {  // the only supported value
      ROS_ERROR("GuardedMoveResult frame was not %s", valid.c_str());
      return;
    }
    GroundPosition = msg->position.z;
    publish ("GroundFound", true);
    publish ("GroundPosition", GroundPosition);
  }
  else {
    ROS_WARN("GuardedMove did not find ground!");
  }
}


//////////////////// GuardedMove Action support ////////////////////////////////

// At present, this is a prototypical ROS action using a dummy server in this
// directory, GuardedMoveServer.  It is NOT connected to the testbed.

static void guarded_move_done_cb
(const actionlib::SimpleClientGoalState& state,
 const ow_autonomy::GuardedMoveResultConstPtr& result)
{
  ROS_INFO ("GuardedMove done callback: finished in state [%s]",
            state.toString().c_str());
  ROS_INFO("GuardedMove done callback: result (%f, %f, %f)",
           result->final.x, result->final.y, result->final.z);
}

static void guarded_move_active_cb ()
{
  ROS_INFO ("GuardedMove active callback - goal active!");
}

static void guarded_move_feedback_cb
(const ow_autonomy::GuardedMoveFeedbackConstPtr& feedback)
{
  ROS_INFO ("GuardedMove feedback callback: (%f, %f, %f)",
            feedback->current.x, feedback->current.y, feedback->current.z);
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
    m_socSubscriber (nullptr),
    m_rulSubscriber (nullptr),
    m_batteryTempSubscriber (nullptr),
    m_guardedMoveSubscriber (nullptr),
    m_systemFaultMessagesSubscriber (nullptr),
    m_armFaultMessagesSubscriber (nullptr),
    m_powerFaultMessagesSubscriber (nullptr),
    m_ptFaultMessagesSubscriber (nullptr),

    m_currentPan (0), m_currentTilt (0),
    m_goalPan (0), m_goalTilt (0)
    // m_panStart, m_tiltStart left uninitialized
{
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
  if (m_socSubscriber) delete m_socSubscriber;
  if (m_rulSubscriber) delete m_rulSubscriber;
  if (m_batteryTempSubscriber) delete m_batteryTempSubscriber;
  if (m_guardedMoveSubscriber) delete m_guardedMoveSubscriber;
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
       subscribe("/ant_tilt_position_controller/state", qsize,
                 &OwInterface::tiltCallback, this));
    m_antennaPanSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/ant_pan_position_controller/state", qsize,
                 &OwInterface::panCallback, this));
    m_jointStatesSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/joint_states", qsize,
                 &OwInterface::jointStatesCallback, this));
    m_cameraSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/StereoCamera/left/image_raw", qsize,
                 &OwInterface::cameraCallback, this));
    m_socSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/power_system_node/state_of_charge", qsize, soc_callback));
    m_batteryTempSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/power_system_node/battery_temperature", qsize,
                 temperature_callback));
    m_rulSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/power_system_node/remaining_useful_life", qsize, rul_callback));
    m_guardedMoveSubscriber = new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/guarded_move_result", qsize, guarded_move_callback));
    // subscribers for fault messages
    m_systemFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/system_faults_status", qsize,
                &OwInterface::systemFaultMessageCallback, this)));
    m_armFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/arm_faults_status", qsize,
                &OwInterface::armFaultCallback, this)));
    m_powerFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/power_faults_status", qsize,
                &OwInterface::powerFaultCallback, this)));
    m_ptFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/pt_faults_status", qsize,
                &OwInterface::antennaFaultCallback, this)));

    ROS_INFO ("Waiting for action servers...");
    m_guardedMoveClient.reset(new GuardedMoveActionClient("GuardedMove", true));
    m_guardedMoveClient->waitForServer();
    ROS_INFO ("Action servers available.");
  }
}

void OwInterface::setCommandStatusCallback (void (*callback) (int, bool))
{
  CommandStatusCallback = callback;
}

void OwInterface::guardedMoveActionDemo (const geometry_msgs::Point& start,
                                         const geometry_msgs::Point& normal,
                                         double search_distance,
                                         int id)
{
  if (! mark_operation_running (Op_GuardedMoveAction, id)) return;

  thread action_thread (&OwInterface::guardedMoveActionDemo1, this, start,
                        normal, search_distance, id);
  action_thread.detach();
}

void OwInterface::guardedMoveActionDemo1 (const geometry_msgs::Point& start,
                                          const geometry_msgs::Point& normal,
                                          double search_distance,
                                          int id)
{
  ow_autonomy::GuardedMoveGoal goal;
  goal.start = start;
  goal.normal = normal;
  goal.search_distance = search_distance;

  thread fault_thread (monitor_for_faults, Op_GuardedMoveAction);
  m_guardedMoveClient->sendGoal (goal,
                                guarded_move_done_cb,
                                guarded_move_active_cb,
                                guarded_move_feedback_cb);

  // Wait for the action to return
  bool finished_before_timeout =
    m_guardedMoveClient->waitForResult (ros::Duration (30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = m_guardedMoveClient->getState();
    ROS_INFO("GuardedMove action finished: %s", state.toString().c_str());
    ow_autonomy::GuardedMoveResultConstPtr result =
      m_guardedMoveClient->getResult();
    ROS_INFO("GuardedMove action result: (%f, %f, %f)",
             result->final.x, result->final.y, result->final.z);
  }
  else {
    ROS_INFO("GuardedMove action did not finish before the time out.");
  }

  mark_operation_finished (Op_GuardedMoveAction, id);
  fault_thread.join();
}

void OwInterface::guardedMove (double x, double y, double z,
                               double direction_x,
                               double direction_y,
                               double direction_z,
                               double search_distance,
                               int id)
{
  if (! mark_operation_running (Op_GuardedMove, id)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::GuardedMove>("/arm/guarded_move");

  if (check_service_client (client, Op_GuardedMove)) {
    ow_lander::GuardedMove srv;
    srv.request.use_defaults = false;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.direction_x = direction_x;
    srv.request.direction_y = direction_y;
    srv.request.direction_z = direction_z;
    srv.request.search_distance = search_distance;
    GroundFound = false;
    thread service_thread (call_ros_service<ow_lander::GuardedMove>,
                           client, srv, Op_GuardedMove, id);
    service_thread.detach();
  }
}

static void antenna_op (const string& opname, double degrees,
                        ros::Publisher* pub, int id)
{
  if (! mark_operation_running (opname, id)) {
    return;
  }

  std_msgs::Float64 radians;
  radians.data = degrees * D2R;
  ROS_INFO ("  Starting %s: %f degrees (%f radians)", opname.c_str(),
            degrees, radians.data);
  thread fault_thread (monitor_for_faults, opname);
  fault_thread.detach();
  pub->publish (radians);
}

void OwInterface::tiltAntenna (double degrees, int id)
{
  m_goalTilt = degrees;
  m_tiltStart = ros::Time::now();
  antenna_op (Op_TiltAntenna, degrees, m_antennaTiltPublisher, id);
}

void OwInterface::panAntenna (double degrees, int id)
{
  m_goalPan = degrees;
  m_panStart = ros::Time::now();
  antenna_op (Op_PanAntenna, degrees, m_antennaPanPublisher, id);
}

void OwInterface::takePicture (int id)
{
  if (! mark_operation_running (Op_TakePicture, id)) return;
  std_msgs::Empty msg;
  ROS_INFO ("  Capturing stereo image using left image trigger.");
  thread fault_thread (monitor_for_faults, Op_TakePicture);
  fault_thread.detach();
  m_leftImageTriggerPublisher->publish (msg);
}

void OwInterface::digLinear (double x, double y,
                             double depth, double length, double ground_position,
                             int id)
{
  if (! mark_operation_running (Op_DigLinear, id)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::DigLinear>("/arm/dig_linear");

  if (check_service_client (client, Op_DigLinear)) {
    ow_lander::DigLinear srv;
    srv.request.use_defaults = false;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.depth = depth;
    srv.request.length = length;
    srv.request.ground_position = ground_position;
    thread service_thread (call_ros_service<ow_lander::DigLinear>,
                           client, srv, Op_DigLinear, id);
    service_thread.detach();
  }
}

void OwInterface::digCircular (double x, double y, double depth,
                               double ground_position, bool parallel, int id)
{
  if (! mark_operation_running (Op_DigCircular, id)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::DigCircular>("/arm/dig_circular");

  if (check_service_client (client, Op_DigCircular)) {
    ow_lander::DigCircular srv;
    srv.request.use_defaults = false;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.depth = depth;
    srv.request.ground_position = ground_position;
    srv.request.parallel = parallel;
    thread service_thread (call_ros_service<ow_lander::DigCircular>,
                           client, srv, Op_DigCircular, id);
    service_thread.detach();
  }
}

void OwInterface::grind (double x, double y, double depth, double length,
                         bool parallel, double ground_pos, int id)
{
  if (! mark_operation_running (Op_Grind, id)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::Grind>("/arm/grind");

  if (check_service_client (client, Op_Grind)) {
    ow_lander::Grind srv;
    srv.request.use_defaults = false;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.depth = depth;
    srv.request.length = length;
    srv.request.parallel = parallel;
    srv.request.ground_position = ground_pos;
    thread service_thread (call_ros_service<ow_lander::Grind>,
                           client, srv, Op_Grind, id);
    service_thread.detach();
  }
}

void OwInterface::stow (int id)
{
  if (! mark_operation_running (Op_Stow, id)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::Stow>("/arm/stow");

  if (check_service_client (client, Op_Stow)) {
    ow_lander::Stow srv;
    thread service_thread (call_ros_service<ow_lander::Stow>,
                           client, srv, Op_Stow, id);
    service_thread.detach();
  }
}

void OwInterface::unstow (int id)
{
  if (! mark_operation_running (Op_Unstow, id)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::Unstow>("/arm/unstow");


  if (check_service_client (client, Op_Unstow)) {
    ow_lander::Unstow srv;
    thread service_thread (call_ros_service<ow_lander::Unstow>,
                           client, srv, Op_Unstow, id);
    service_thread.detach();
  }
}

void OwInterface::deliverSample (double x, double y, double z, int id)
{
  if (! mark_operation_running (Op_DeliverSample, id)) return;

  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::DeliverSample>("/arm/deliver_sample");


  if (check_service_client (client, Op_DeliverSample)) {
    ow_lander::DeliverSample srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    thread service_thread (call_ros_service<ow_lander::DeliverSample>,
                           client, srv, Op_DeliverSample, id);
    service_thread.detach();
  }
}

double OwInterface::getTilt () const
{
  return m_currentTilt;
}

double OwInterface::getPanDegrees () const
{
  return m_currentPan;
}

double OwInterface::getPanVelocity () const
{
  return JointTelemetryMap[Joint::antenna_pan].velocity;
}

double OwInterface::getTiltVelocity () const
{
  return JointTelemetryMap[Joint::antenna_tilt].velocity;
}

double OwInterface::getStateOfCharge () const
{
  return StateOfCharge;
}

double OwInterface::getRemainingUsefulLife () const
{
  return RemainingUsefulLife;
}

double OwInterface::getBatteryTemperature () const
{
  return BatteryTemperature;
}

bool OwInterface::operationRunning (const string& name) const
{
  // Note: check in caller guarantees 'at' to return a valid value.
  return Running.at (name) != IDLE_ID;
}

bool OwInterface::running (const string& name) const
{
  if (is_lander_operation (name)) return operationRunning (name);

  ROS_ERROR("OwInterface::running: unsupported operation: %s", name.c_str());
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
