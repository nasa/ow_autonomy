// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ow_autonomy
#include "OwInterface.h"
#include "subscriber.h"
#include "joint_support.h"

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

// Lander operation names.  In general these match those used in PLEXIL and
// ow_lander.

const string Op_GuardedMove       = "GuardedMove";
const string Op_DigCircular       = "DigCircular";
const string Op_DigLinear         = "DigLinear";
const string Op_Deliver           = "Deliver";
const string Op_PanAntenna        = "PanAntenna";
const string Op_TiltAntenna       = "TiltAntenna";
const string Op_Grind             = "Grind";
const string Op_Stow              = "Stow";
const string Op_Unstow            = "Unstow";
const string Op_TakePicture       = "TakePicture";

enum LanderOps {
  GuardedMove,
  DigCircular,
  DigLinear,
  Deliver,
  Pan,
  Tilt,
  Grind,
  Stow,
  Unstow,
  TakePicture
};

static std::vector<string> LanderOpNames =
  { Op_GuardedMove, Op_DigCircular, Op_DigLinear, Op_Deliver,
    Op_PanAntenna, Op_TiltAntenna, Op_Grind, Op_Stow, Op_Unstow, Op_TakePicture
  };

// NOTE: The following map *should* be thread-safe, according to C++11 docs and
// in particular because map entries are never added or deleted, and the code
// insures that each entry can be read/written by only one thread.  (The map
// itself can be read/written by multiple threads concurrently).

// Unused operation ID that signifies idle lander operation.
#define IDLE_ID (-1)

static map<string, int> Running
{
  { Op_GuardedMove, IDLE_ID },
  { Op_DigCircular, IDLE_ID },
  { Op_DigLinear, IDLE_ID },
  { Op_Deliver, IDLE_ID },
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

template <typename T1, typename T2>
void OwInterface::faultCallback (T1 msg_val, T2& fmap,
                                 const string& component)
{
  for (auto const& entry : fmap) {
    string key = entry.first;
    T1 value = entry.second.first;
    bool fault_active = entry.second.second;
    bool faulty = (msg_val & value) == value;
    if (!fault_active && faulty) {
      ROS_WARN ("Fault in %s: %s", component.c_str(), key.c_str());
      fmap[key].second = true;
    }
    else if (fault_active && !faulty) {
      ROS_WARN ("Resolved fault in %s: %s", component.c_str(), key.c_str());
      fmap[key].second = false;
    }
  }
}

void OwInterface::systemFaultMessageCallback
(const  ow_faults::SystemFaults::ConstPtr& msg)
{
  faultCallback (msg->value, m_systemErrors, "SYSTEM");
}

void OwInterface::armFaultCallback(const ow_faults::ArmFaults::ConstPtr& msg)
{
  faultCallback (msg->value, m_armErrors, "ARM");
}

void OwInterface::powerFaultCallback (const ow_faults::PowerFaults::ConstPtr& msg)
{
  faultCallback (msg->value, m_powerErrors, "POWER");
}

void OwInterface::antennaFaultCallback(const ow_faults::PTFaults::ConstPtr& msg)
{
  faultCallback (msg->value, m_panTiltErrors, "ANTENNA");
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


//////////////////// GuardedMove Action support ////////////////////////////////

// TODO: encapsulate GroundFound and GroundPosition in the PLEXIL command.  They
// are not accurate outside the context of a single GuardedMove command.

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

template <typename T>
bool OwInterface::faultActive (const T& fmap) const
{
  for (auto const& entry : fmap) {
    if (entry.second.second) return true;
  }
  return false;
}

bool OwInterface::systemFault () const
{
  return faultActive (m_systemErrors);
}

bool OwInterface::antennaFault () const
{
  return faultActive (m_panTiltErrors);
}

bool OwInterface::armFault () const
{
  return faultActive (m_armErrors);
}

bool OwInterface::powerFault () const
{
  return faultActive (m_powerErrors);
}

template<int OpIndex, typename T>
static void guarded_move_done_cb
(const actionlib::SimpleClientGoalState& state,
 const T& result)
{
  ROS_INFO ("GuardedMove finished in state %s", state.toString().c_str());
  GroundFound = result->success;
  GroundPosition = result->final.z;
  publish ("GroundFound", GroundFound);
  publish ("GroundPosition", GroundPosition);
}

//////////////////// General Action support ///////////////////////////////

const auto ActionServerTimeout = 10.0;  // seconds

//////////////////// ROS Action callbacks - generic //////////////////////

template<typename T>
static void action_feedback_cb (const T& feedback)
{
}

template<int OpIndex>
static void active_cb ()
{
  ROS_INFO ("%s started...", LanderOpNames[OpIndex].c_str());
}

template<int OpIndex, typename T>
static void default_action_done_cb
(const actionlib::SimpleClientGoalState& state,
 const T& result_ignored)
{
  ROS_INFO ("%s finished in state %s", LanderOpNames[OpIndex].c_str(),
            state.toString().c_str());
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
    m_systemFaultMessagesSubscriber (nullptr),
    m_armFaultMessagesSubscriber (nullptr),
    m_powerFaultMessagesSubscriber (nullptr),
    m_ptFaultMessagesSubscriber (nullptr),

    m_currentPan (0), m_currentTilt (0),
    m_goalPan (0), m_goalTilt (0)
    // m_panStart, m_tiltStart are deliberately uninitialized
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
    // subscribers for fault messages
    m_systemFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/faults/system_faults_status", qsize,
                &OwInterface::systemFaultMessageCallback, this)));
    m_armFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/faults/arm_faults_status", qsize,
                &OwInterface::armFaultCallback, this)));
    m_powerFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/faults/power_faults_status", qsize,
                &OwInterface::powerFaultCallback, this)));
    m_ptFaultMessagesSubscriber.reset(new ros::Subscriber
      (m_genericNodeHandle ->
       subscribe("/faults/pt_faults_status", qsize,
                &OwInterface::antennaFaultCallback, this)));

    m_guardedMoveClient.reset(new GuardedMoveActionClient(Op_GuardedMove, true));
    m_unstowClient.reset(new UnstowActionClient(Op_Unstow, true));
    m_stowClient.reset(new StowActionClient(Op_Stow, true));
    m_grindClient.reset(new GrindActionClient(Op_Grind, true));
    m_digCircularClient.reset(new DigCircularActionClient(Op_DigCircular, true));
    m_digLinearClient.reset(new DigLinearActionClient(Op_DigLinear, true));
    m_deliverClient.reset(new DeliverActionClient(Op_Deliver, true));

    if (! m_unstowClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("Unstow action server did not connect!");
    }
    if (! m_stowClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("Stow action server did not connect!");
    }
    if (! m_digCircularClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("DigCircular action server did not connect!");
    }
    if (! m_digLinearClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("DigLinear action server did not connect!");
    }
    if (! m_deliverClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("Deliver action server did not connect!");
    }
    if (! m_guardedMoveClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("GuardedMove action server did not connect!");
    }
  }
}

void OwInterface::setCommandStatusCallback (void (*callback) (int, bool))
{
  CommandStatusCallback = callback;
}

static void antenna_op (const string& opname, double degrees,
                        ros::Publisher* pub, int id)
{
  if (! mark_operation_running (opname, id)) {
    return;
  }

  std_msgs::Float64 radians;
  radians.data = degrees * D2R;
  ROS_INFO ("Starting %s: %f degrees (%f radians)", opname.c_str(),
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
  ROS_INFO ("Capturing stereo image using left image trigger.");
  thread fault_thread (monitor_for_faults, Op_TakePicture);
  fault_thread.detach();
  m_leftImageTriggerPublisher->publish (msg);
}

void OwInterface::deliver (double x, double y, double z, int id)
{
  if (! mark_operation_running (Op_Deliver, id)) return;
  thread action_thread (&OwInterface::deliverAction, this, x, y, z, id);
  action_thread.detach();
}

template <int OpIndex, class ActionClient, class Goal,
          class ResultPtr, class FeedbackPtr>
void OwInterface::runAction (const string& opname,
                             std::unique_ptr<ActionClient>& ac,
                             const Goal& goal, int id,
                             t_action_done_cb<OpIndex, ResultPtr> done_cb)
{
  thread fault_thread (monitor_for_faults, opname);
  if (ac) {
    ROS_INFO ("Sending goal to action %s", opname.c_str());
    ac->sendGoal (goal,
                  done_cb,
                  active_cb<OpIndex>,
                  action_feedback_cb<FeedbackPtr>);
  }
  else {
    ROS_ERROR ("%s action client was null!", opname.c_str());
    return;
  }

  // Wait indefinitely for the action to complete.
  bool finished_before_timeout = ac->waitForResult (ros::Duration (0));
  mark_operation_finished (opname, id);
  fault_thread.join();
}

void OwInterface::deliverAction (double x, double y, double z, int id)
{
  ow_lander::DeliverGoal goal;
  goal.delivery.x = x;
  goal.delivery.y = y;
  goal.delivery.z = z;
  runAction<Deliver, actionlib::SimpleActionClient<ow_lander::DeliverAction>,
            ow_lander::DeliverGoal,
            ow_lander::DeliverResultConstPtr,
            ow_lander::DeliverFeedbackConstPtr>
    (Op_Deliver, m_deliverClient, goal, id);
}

void OwInterface::digLinear (double x, double y,
                             double depth, double length, double ground_pos,
                             int id)
{
  if (! mark_operation_running (Op_DigLinear, id)) return;
  thread action_thread (&OwInterface::digLinearAction, this, x, y, depth,
                        length, ground_pos, id);
  action_thread.detach();
}


void OwInterface::digLinearAction (double x, double y,
                                   double depth, double length,
                                   double ground_pos, int id)
{
  ow_lander::DigLinearGoal goal;
  goal.x_start = x;
  goal.y_start = y;
  goal.depth = depth;
  goal.length = length;
  goal.ground_position = ground_pos;

  runAction<DigLinear, actionlib::SimpleActionClient<ow_lander::DigLinearAction>,
            ow_lander::DigLinearGoal,
            ow_lander::DigLinearResultConstPtr,
            ow_lander::DigLinearFeedbackConstPtr>
    (Op_DigLinear, m_digLinearClient, goal, id);
}


void OwInterface::digCircular (double x, double y, double depth,
                               double ground_pos, bool parallel, int id)
{
  if (! mark_operation_running (Op_DigCircular, id)) return;
  thread action_thread (&OwInterface::digCircularAction, this, x, y, depth,
                        ground_pos, parallel, id);
  action_thread.detach();
}

void OwInterface::digCircularAction (double x, double y, double depth,
                                     double ground_pos, bool parallel, int id)
{
  ow_lander::DigCircularGoal goal;
  goal.x_start = x;
  goal.y_start = y;
  goal.depth = depth;
  goal.ground_position = ground_pos;
  goal.parallel = parallel;

  runAction<DigCircular,
            actionlib::SimpleActionClient<ow_lander::DigCircularAction>,
            ow_lander::DigCircularGoal,
            ow_lander::DigCircularResultConstPtr,
            ow_lander::DigCircularFeedbackConstPtr>
    (Op_DigCircular, m_digCircularClient, goal, id);
}


void OwInterface::unstow (int id)  // as action
{
  if (! mark_operation_running (Op_Unstow, id)) return;
  thread action_thread (&OwInterface::unstowAction, this, id);
  action_thread.detach();
}

void OwInterface::unstowAction (int id)
{
  ow_lander::UnstowGoal goal;
  goal.goal = 0;  // Arbitrary, meaningless value

  runAction<Unstow, actionlib::SimpleActionClient<ow_lander::UnstowAction>,
            ow_lander::UnstowGoal,
            ow_lander::UnstowResultConstPtr,
            ow_lander::UnstowFeedbackConstPtr>
    (Op_Unstow, m_unstowClient, goal, id);
}

void OwInterface::stow (int id)  // as action
{
  if (! mark_operation_running (Op_Stow, id)) return;
  thread action_thread (&OwInterface::stowAction, this, id);
  action_thread.detach();
}

void OwInterface::stowAction (int id)
{
  ow_lander::StowGoal goal;
  goal.goal = 0;  // Arbitrary, meaningless value

  runAction<Stow, actionlib::SimpleActionClient<ow_lander::StowAction>,
            ow_lander::StowGoal,
            ow_lander::StowResultConstPtr,
            ow_lander::StowFeedbackConstPtr>
    (Op_Stow, m_stowClient, goal, id);
}

void OwInterface::grind (double x, double y, double depth, double length,
                         bool parallel, double ground_pos, int id)
{
  if (! mark_operation_running (Op_Grind, id)) return;
  thread action_thread (&OwInterface::grindAction, this, x, y, depth, length,
                        parallel, ground_pos, id);
  action_thread.detach();
}

void OwInterface::grindAction (double x, double y, double depth, double length,
                               bool parallel, double ground_pos, int id)
{
  ow_lander::GrindGoal goal;
  goal.x_start = x;
  goal.y_start = y;
  goal.depth = depth;
  goal.length = length;
  goal.parallel = parallel;
  goal.ground_position = ground_pos;

  runAction<Grind, actionlib::SimpleActionClient<ow_lander::GrindAction>,
            ow_lander::GrindGoal,
            ow_lander::GrindResultConstPtr,
            ow_lander::GrindFeedbackConstPtr>
    (Op_Grind, m_grindClient, goal, id);
}

void OwInterface::guardedMove (double x, double y, double z,
                               double dir_x, double dir_y, double dir_z,
                               double search_dist, int id)
{
  if (! mark_operation_running (Op_GuardedMove, id)) return;
  thread action_thread (&OwInterface::guardedMoveAction, this, x, y, z,
                        dir_x, dir_y, dir_z, search_dist, id);
  action_thread.detach();
}

void OwInterface::guardedMoveAction (double x, double y, double z,
                                     double dir_x, double dir_y, double dir_z,
                                     double search_dist, int id)
{
  ow_lander::GuardedMoveGoal goal;
  goal.start.x = x;
  goal.start.y = y;
  goal.start.z = z;
  goal.normal.x = dir_x;
  goal.normal.y = dir_y;
  goal.normal.z = dir_z;
  goal.search_distance = search_dist;

  runAction<GuardedMove, actionlib::SimpleActionClient<ow_lander::GuardedMoveAction>,
            ow_lander::GuardedMoveGoal,
            ow_lander::GuardedMoveResultConstPtr,
            ow_lander::GuardedMoveFeedbackConstPtr>
    (Op_GuardedMove, m_guardedMoveClient, goal, id,
    guarded_move_done_cb<GuardedMove, ow_lander::GuardedMoveResultConstPtr>);
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
