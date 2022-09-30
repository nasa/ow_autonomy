// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ow_plexil
#include "OwInterface.h"
#include "subscriber.h"
#include "joint_support.h"

// OW - external
#include <ow_lander/Light.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

// C++
#include <set>
#include <map>
#include <thread>
#include <functional>
#include <algorithm> // for std::copy
#include <inttypes.h> // for int64 support
using std::set;
using std::map;
using std::vector;
using std::thread;
using std::ref;
using std::string;
using std::shared_ptr;
using std::make_unique;

// C
#include <cmath>  // for M_PI and fabs

using namespace ow_lander;

//////////////////// Utilities ////////////////////////

// Degree/Radian
const double D2R = M_PI / 180.0 ;
const double R2D = 180.0 / M_PI ;

const double DegreeTolerance = 0.4;    // made up, degees
const double VelocityTolerance = 0.01; // made up, unitless

static bool within_tolerance (double val1, double val2, double tolerance)
{
  return fabs (val1 - val2) <= tolerance;
}


/////////////////// ROS Service support //////////////////////

template<typename Service>
void OwInterface::callService (ros::ServiceClient client, Service srv,
                               string name, int id)
{
  // NOTE: arguments are copies because this function is called in a thread that
  // outlives its caller.  Assumes that service is not already running; this is
  // checked upstream.

  ROS_INFO("Starting ROS service %s", name.c_str());
  if (client.call (srv)) { // blocks
    ROS_INFO("%s returned: %d, %s", name.c_str(), srv.response.success,
             srv.response.message.c_str());  // make DEBUG later
  }
  else {
    ROS_ERROR("Failed to call service %s", name.c_str());
  }
  markOperationFinished (name, id);
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


//////////////////// Lander Operation Support ////////////////////////

const double PanTiltTimeout = 15.0; // seconds, made up
const double PointCloudTimeout = 50.0; // 5 second timeout assuming a rate of 10hz
const double SampleTimeout = 50.0; // 5 second timeout assuming a rate of 10hz

// Lander operation names.  In general these match those used in PLEXIL and
// ow_lander.

const string Op_GuardedMove            = "GuardedMove";
const string Op_ArmMoveJoint           = "ArmMoveJoint";
const string Op_ArmMoveJoints          = "ArmMoveJoints";
const string Op_DigCircular            = "DigCircular";
const string Op_DigLinear              = "DigLinear";
const string Op_Deliver                = "Deliver";
const string Op_Discard                = "Discard";
const string Op_PanAntenna             = "PanAntenna";
const string Op_TiltAntenna            = "TiltAntenna";
const string Op_Grind                  = "Grind";
const string Op_Stow                   = "Stow";
const string Op_Unstow                 = "Unstow";
const string Op_TakePicture            = "TakePicture";
const string Op_IdentifySampleLocation = "IdentifySampleLocation";
const string Op_SetLightIntensity      = "SetLightIntensity";


// 1. Indices into subsequent vector
//
enum LanderOps {
  GuardedMove,
  ArmMoveJoint,
  ArmMoveJoints,
  DigCircular,
  DigLinear,
  Deliver,
  Discard,
  Pan,
  Tilt,
  Grind,
  Stow,
  Unstow,
  TakePicture,
  IdentifySampleLocation,
  SetLightIntensity
};

static vector<string> LanderOpNames = {
  Op_GuardedMove, Op_ArmMoveJoint, Op_ArmMoveJoints, Op_DigCircular, Op_DigLinear,
  Op_Deliver, Op_Discard, Op_PanAntenna, Op_TiltAntenna, Op_Grind, Op_Stow,
  Op_Unstow, Op_TakePicture, Op_IdentifySampleLocation, Op_SetLightIntensity
};


///////////////////////// Action Goal Status Support /////////////////////////

// Repeating GoalStatus defined in actionlib_msgs/GoalStatus.msg
// with the addition of a NOGOAL status for when the action is not running.
static map<int, string> GoalStatus {
  // GoalID -> Goal Status

  { -1, "NOGOAL" },
  { 0, "PENDING" },
  { 1, "ACTIVE" },
  { 2, "PREEMPTED" },
  { 3, "SUCCEEDED" },
  { 4, "ABORTED" },
  { 5, "REJECTED" },
  { 6, "PREEMPTING" },
  { 7, "RECALLING" },
  { 8, "RECALLED" },
  { 9, "LOST" }
};

static map<string, int> ActionGoalStatusMap {
  // ROS action name -> Action goal status
  // Assigning -1 as the default state

  { "Stow", -1 },
  { "Unstow", -1 },
  { "Grind", -1 },
  { "GuardedMove", -1 },
  { "ArmMoveJoint", -1 },
  { "ArmMoveJoints", -1 },
  { "DigCircular", -1 },
  { "DigLinear", -1 },
  { "Deliver", -1 },
  { "Discard", -1 }
};

static void update_action_goal_state (string action, int state)
{
  if (ActionGoalStatusMap.find(action) != ActionGoalStatusMap.end()) {
    if (GoalStatus.find(state) != GoalStatus.end()) {
      
      // update ActionGoalStatusMap only if the state is different
      // from the current state
      if (ActionGoalStatusMap[action] != state) {
        ActionGoalStatusMap[action] = state;
      }
    }
    else {
      ROS_ERROR("Unknown goal status: %d", state);
    }
  }
  else {
    ROS_ERROR("Unknown action: %s", action.c_str());  
  }
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
void OwInterface::updateFaultStatus (T1 msg_val, T2& fmap,
                                     const string& component,
                                     const string& state_name)
{
  for (auto const& entry : fmap) {
    string key = entry.first;
    T1 value = entry.second.first;
    bool fault_active = entry.second.second;
    bool faulty = (msg_val & value) == value;
    if (!fault_active && faulty) {
      ROS_WARN ("Fault in %s: %s", component.c_str(), key.c_str());
      fmap[key].second = true;
      publish (state_name, true);
    }
    else if (fault_active && !faulty) {
      ROS_WARN ("Resolved fault in %s: %s", component.c_str(), key.c_str());
      fmap[key].second = false;
      publish (state_name, false);
    }
  }
}

///////////////////////// Subscriber Callbacks ///////////////////////////////

void OwInterface::systemFaultMessageCallback
(const  ow_faults_detection::SystemFaults::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_systemErrors, "SYSTEM", "SystemFault");
}

void OwInterface::armFaultCallback
(const ow_faults_detection::ArmFaults::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_armErrors, "ARM", "ArmFault");
}

void OwInterface::powerFaultCallback
(const ow_faults_detection::PowerFaults::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_powerErrors, "POWER", "PowerFault");
}

void OwInterface::antennaFaultCallback
(const ow_faults_detection::PTFaults::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_panTiltErrors, "ANTENNA", "AntennaFault");
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
        m_currentPan = position * R2D;
        managePanTilt (Op_PanAntenna, m_currentPan, m_goalPan, m_panStart);
        publish ("PanDegrees", m_currentPan);
     }
      else if (joint == Joint::antenna_tilt) {
        m_currentTilt = position * R2D;
        managePanTilt (Op_TiltAntenna, m_currentTilt, m_goalTilt, m_tiltStart);
        publish ("TiltDegrees", m_currentTilt);
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
                                 double current, double goal,
                                 const ros::Time& start)
{
  // We are only concerned when there is a pan/tilt in progress.
  if (! operationRunning (opname)) return;

  int id = m_runningOperations.at (opname);

  //if position is over 360 we want to bring it back within the
  //-360 to 360 range to check if goal position has been reached.
  if(fabs(current) > 360){
    if(current < 0){
      current = fmod(fabs(current),360.0)*-1;
    }
    else{
      current = fmod(fabs(current), 360.0);
    }
  }

  // We can't guarantee which way the antenna will move, so we have to check if
  // the current angle is equivalent.Ex:-180 degrees == 180 degrees.
  if(current > 0 && goal < 0){
    current = current - 360;
  }
  else if(current < 0 && goal > 0){
    current = current + 360;
  }

  // Antenna states of interest,
  bool reached = within_tolerance (current, goal, DegreeTolerance);
  bool expired = ros::Time::now() > start + ros::Duration (PanTiltTimeout);

  if (reached || expired) {
    markOperationFinished (opname, id);
    if (expired) ROS_ERROR("%s timed out", opname.c_str());
    if (! reached) {
      ROS_ERROR("%s failed. Ended at %f degrees, goal was %f.",
                opname.c_str(), current, goal);
    }
  }
}

///////////////////////// Antenna/Camera Support ///////////////////////////////
void OwInterface::cameraCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  // NOTE: the received image is ignored for now.
  m_pointCloudRecieved = false;
  if (operationRunning (Op_TakePicture)) {
    ros::Rate rate(10);
    int timeout = 0;
    // We wait for the pointcloud as well or the 5 sec timeout before marking as
    // finished.
    while(m_pointCloudRecieved == false && timeout < PointCloudTimeout){
        ros::spinOnce();
        rate.sleep();
        timeout+=1;
    }
    if(timeout == PointCloudTimeout){
      ROS_ERROR("Timeout Exceeded: Recieved an Image but no PointCloud2.");
    }
    markOperationFinished (Op_TakePicture,
                           m_runningOperations.at (Op_TakePicture));
  }
}

void OwInterface::pointCloudCallback
(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // NOTE: the received pointcloud is ignored for now.
  if (operationRunning (Op_TakePicture)) {
    //mark as recieved
    m_pointCloudRecieved = true;
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

//////////////////// Action Status support ////////////////////////////////
void OwInterface::actionGoalStatusCallback
(const actionlib_msgs::GoalStatusArray::ConstPtr& msg, const string action_name)
  // Update ActionGoalStatusMap of action action_name with the status from first  
  // goal in GoalStatusArray msg. This is based on the assumption that no action 
  // will have more than one goal in our system.
  
{
  if (msg->status_list.size() == 0) {
    int status = -1;
    update_action_goal_state (action_name, status);
  }
  else {
    int status = msg->status_list[0].status;
    update_action_goal_state (action_name, status);
  }
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

template<typename T>
static t_action_done_cb<T> guarded_move_done_cb (const string& opname)
{
  return [&] (const actionlib::SimpleClientGoalState& state,
              const T& result) {
    ROS_INFO ("%s finished in state %s", opname.c_str(),
              state.toString().c_str());
    GroundFound = result->success;
    GroundPosition = result->final.z;
    publish ("GroundFound", GroundFound);
    publish ("GroundPosition", GroundPosition);
  };
}

// Since template is outside the scope of the class we cannot use a member
// variable and instead have to use a static one like in the
// guarded_move_done_cb above.
static vector<double> SamplePoint;
static bool GotSampleLocation = false;
template<int OpIndex, typename T>
static void identify_sample_location_done_cb
(const actionlib::SimpleClientGoalState& state,
 const T& result)
{
  if(result->success == true){
    SamplePoint.push_back(result->sample_location.x);
    SamplePoint.push_back(result->sample_location.y);
    SamplePoint.push_back(result->sample_location.z);
    ROS_INFO ("Possible sample location identified at (%f, %f, %f)",
           SamplePoint[0], SamplePoint[1], SamplePoint[2]);
  }
  else{
    ROS_ERROR("Could not get sample point from IdentifySampleLocation");
  }
  GotSampleLocation = true;
}


/////////////////////////// OwInterface members ////////////////////////////////

OwInterface* OwInterface::instance ()
{
  // Very simple singleton
  static OwInterface instance;
  return &instance;
}

OwInterface::OwInterface ()
  : m_currentPan (0), m_currentTilt (0),
    m_goalPan (0), m_goalTilt (0), m_pointCloudRecieved(false)
    // m_panStart, m_tiltStart are deliberately uninitialized
{
}

void OwInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {

    for (const string& name : LanderOpNames) {
      registerLanderOperation (name);
    }

    m_genericNodeHandle = make_unique<ros::NodeHandle>();

    // Initialize publishers.  Queue size is a guess at adequacy.  For now,
    // latching in lieu of waiting for publishers.

    const int qsize = 3;
    const bool latch = true;
    m_antennaTiltPublisher = make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_tilt_position_controller/command", qsize, latch));
    m_antennaPanPublisher = make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_pan_position_controller/command", qsize, latch));
    m_leftImageTriggerPublisher = make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::Empty>
       ("/StereoCamera/left/image_trigger", qsize, latch));

    // Initialize subscribers
    m_jointStatesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/joint_states", qsize,
                 &OwInterface::jointStatesCallback, this));
    m_cameraSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/StereoCamera/left/image_raw", qsize,
                 &OwInterface::cameraCallback, this));
    m_pointCloudSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/StereoCamera/points2", qsize,
                 &OwInterface::pointCloudCallback, this));
    m_socSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/power_system_node/state_of_charge", qsize, soc_callback));
    m_batteryTempSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/power_system_node/battery_temperature", qsize,
                 temperature_callback));
    m_rulSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/power_system_node/remaining_useful_life", qsize,
                 rul_callback));
    // subscribers for fault messages
    m_systemFaultMessagesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/faults/system_faults_status", qsize,
                &OwInterface::systemFaultMessageCallback, this));
    m_armFaultMessagesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/faults/arm_faults_status", qsize,
                &OwInterface::armFaultCallback, this));
    m_powerFaultMessagesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/faults/power_faults_status", qsize,
                &OwInterface::powerFaultCallback, this));
    m_ptFaultMessagesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/faults/pt_faults_status", qsize,
                &OwInterface::antennaFaultCallback, this));
    // action servers and subscribers for action status
    m_guardedMoveClient =
      make_unique<GuardedMoveActionClient>(Op_GuardedMove, true);
    m_armMoveJointClient =
      make_unique<ArmMoveJointActionClient>(Op_ArmMoveJoint, true);
    m_armMoveJointsClient =
      make_unique<ArmMoveJointsActionClient>(Op_ArmMoveJoints, true);
    m_unstowClient =
      make_unique<UnstowActionClient>(Op_Unstow, true);
    m_stowClient =
      make_unique<StowActionClient>(Op_Stow, true);
    m_grindClient =
      make_unique<GrindActionClient>(Op_Grind, true);
    m_digCircularClient =
      make_unique<DigCircularActionClient>(Op_DigCircular, true);
    m_digLinearClient =
      make_unique<DigLinearActionClient>(Op_DigLinear, true);
    m_deliverClient =
      make_unique<DeliverActionClient>(Op_Deliver, true);
    m_discardClient =
      make_unique<DiscardActionClient>(Op_Discard, true);
    m_identifySampleLocationClient =
      make_unique<IdentifySampleLocationActionClient>
      (Op_IdentifySampleLocation, true);

    if (! m_unstowClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("Unstow action server did not connect!");
    } 
    else {
      m_unstowStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
              ("/Unstow/status", qsize,
              boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                          "Unstow")));
    }
    if (! m_stowClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("Stow action server did not connect!");
    }
    else {
      m_stowStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
        ("/Stow/status", qsize,
        boost::bind(&OwInterface::actionGoalStatusCallback, this, _1, "Stow")));
    }
    if (! m_armMoveJointClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS)))  {
      ROS_ERROR ("ArmMoveJoint action server did not connect!");
    }
    else {
      m_armMoveJointStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
            ("/ArmMoveJoint/status", qsize,
            boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                        "ArmMoveJoint")));
    }
    if (! m_armMoveJointsClient ->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("ArmMoveJoints action server did not connect!");
    }
    else {
      m_armMoveJointsStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
              ("/ArmMoveJoints/status", qsize,
              boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                          "ArmMoveJoints")));
    }
    if (! m_digCircularClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("DigCircular action server did not connect!");
    }
    else {
      m_digCircularStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
              ("/DigCircular/status", qsize,
              boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                          "DigCircular")));
    }
    if (! m_digLinearClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("DigLinear action server did not connect!");
    }
    else {
      m_digLinearStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
              ("/DigLinear/status", qsize,
              boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                          "DigLinear")));
    }
    if (! m_deliverClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("Deliver action server did not connect!");
    }
    else {
      m_deliverStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
              ("/Deliver/status", qsize,
              boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                          "Deliver")));
    }
    if (! m_discardClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("Discard action server did not connect!");
    }
    else {
      m_discardStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
              ("/Discard/status", qsize,
              boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                          "Discard")));
    }
    if (! m_guardedMoveClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("GuardedMove action server did not connect!");
    }
    else {
      m_guardedMoveStatusSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
              ("/GuardedMove/status", qsize,
              boost::bind(&OwInterface::actionGoalStatusCallback, this, _1,
                          "GuardedMove")));
    }
    if (! m_identifySampleLocationClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("IdentifySampleLocation action server did not connect!");
    }
  }
}

void OwInterface::antennaOp (const string& opname, double degrees,
                             std::unique_ptr<ros::Publisher>& pub, int id)
{
  if (! markOperationRunning (opname, id)) {
    return;
  }
  std_msgs::Float64 radians;
  radians.data = degrees * D2R;
  ROS_INFO ("Starting %s: %f degrees (%f radians)", opname.c_str(),
            degrees, radians.data);
  pub->publish (radians);
}

void OwInterface::tiltAntenna (double degrees, int id)
{
  m_goalTilt = degrees;
  m_tiltStart = ros::Time::now();
  antennaOp (Op_TiltAntenna, degrees, m_antennaTiltPublisher, id);
}

void OwInterface::panAntenna (double degrees, int id)
{
  m_goalPan = degrees;
  m_panStart = ros::Time::now();
  antennaOp (Op_PanAntenna, degrees, m_antennaPanPublisher, id);
}

void OwInterface::takePicture (int id)
{
  if (! markOperationRunning (Op_TakePicture, id)) return;
  std_msgs::Empty msg;
  ROS_INFO ("Capturing stereo image using left image trigger.");
  m_leftImageTriggerPublisher->publish (msg);
}

void OwInterface::deliver (int id)
{
  if (! markOperationRunning (Op_Deliver, id)) return;
  thread action_thread (&OwInterface::deliverAction, this, id);
  action_thread.detach();
}

void OwInterface::discard (double x, double y, double z, int id)
{
  if (! markOperationRunning (Op_Discard, id)) return;
  thread action_thread (&OwInterface::discardAction, this, x, y, z, id);
  action_thread.detach();
}


void OwInterface::deliverAction (int id)
{
  DeliverGoal goal;

  ROS_INFO ("Starting Deliver()");

  runAction<actionlib::SimpleActionClient<DeliverAction>,
            DeliverGoal,
            DeliverResultConstPtr,
            DeliverFeedbackConstPtr>
    (Op_Deliver, m_deliverClient, goal, id,
     default_action_active_cb (Op_Deliver),
     default_action_feedback_cb<DeliverFeedbackConstPtr> (Op_Deliver),
     default_action_done_cb<DeliverResultConstPtr> (Op_Deliver));
}

void OwInterface::discardAction (double x, double y, double z, int id)
{
  DiscardGoal goal;
  goal.discard.x = x;
  goal.discard.y = y;
  goal.discard.z = z;

  ROS_INFO ("Starting Discard(x=%.2f, y=%.2f, z=%.2f)", x, y, z);

  runAction<actionlib::SimpleActionClient<DiscardAction>,
            DiscardGoal,
            DiscardResultConstPtr,
            DiscardFeedbackConstPtr>
    (Op_Discard, m_discardClient, goal, id,
     default_action_active_cb (Op_Discard),
     default_action_feedback_cb<DiscardFeedbackConstPtr> (Op_Discard),
     default_action_done_cb<DiscardResultConstPtr> (Op_Discard));
}

void OwInterface::digLinear (double x, double y,
                             double depth, double length, double ground_pos,
                             int id)
{
  if (! markOperationRunning (Op_DigLinear, id)) return;
  thread action_thread (&OwInterface::digLinearAction, this, x, y, depth,
                        length, ground_pos, id);
  action_thread.detach();
}


void OwInterface::digLinearAction (double x, double y,
                                   double depth, double length,
                                   double ground_pos, int id)
{
  DigLinearGoal goal;
  goal.x_start = x;
  goal.y_start = y;
  goal.depth = depth;
  goal.length = length;
  goal.ground_position = ground_pos;

  ROS_INFO ("Starting DigLinear"
            "(x=%.2f, y=%.2f, depth=%.2f, length=%.2f, ground_pos=%.2f)",
            x, y, depth, length, ground_pos);

  runAction<actionlib::SimpleActionClient<DigLinearAction>,
            DigLinearGoal,
            DigLinearResultConstPtr,
            DigLinearFeedbackConstPtr>
    (Op_DigLinear, m_digLinearClient, goal, id,
     default_action_active_cb (Op_DigLinear),
     default_action_feedback_cb<DigLinearFeedbackConstPtr> (Op_DigLinear),
     default_action_done_cb<DigLinearResultConstPtr> (Op_DigLinear));
}


void OwInterface::digCircular (double x, double y, double depth,
                               double ground_pos, bool parallel, int id)
{
  if (! markOperationRunning (Op_DigCircular, id)) return;
  thread action_thread (&OwInterface::digCircularAction, this, x, y, depth,
                        ground_pos, parallel, id);
  action_thread.detach();
}

void OwInterface::digCircularAction (double x, double y, double depth,
                                     double ground_pos, bool parallel, int id)
{
  DigCircularGoal goal;
  goal.x_start = x;
  goal.y_start = y;
  goal.depth = depth;
  goal.ground_position = ground_pos;
  goal.parallel = parallel;

  ROS_INFO ("Starting DigCircular"
            "(x=%.2f, y=%.2f, depth=%.2f, parallel=%s, ground_pos=%.2f)",
            x, y, depth, (parallel ? "true" : "false"), ground_pos);

  runAction<actionlib::SimpleActionClient<DigCircularAction>,
            DigCircularGoal,
            DigCircularResultConstPtr,
            DigCircularFeedbackConstPtr>
    (Op_DigCircular, m_digCircularClient, goal, id,
     default_action_active_cb (Op_DigCircular),
     default_action_feedback_cb<DigCircularFeedbackConstPtr> (Op_DigCircular),
     default_action_done_cb<DigCircularResultConstPtr> (Op_DigCircular));
}


void OwInterface::unstow (int id)  // as action
{
  if (! markOperationRunning (Op_Unstow, id)) return;
  thread action_thread (&OwInterface::unstowAction, this, id);
  action_thread.detach();
}

void OwInterface::unstowAction (int id)
{
  UnstowGoal goal;
  goal.goal = 0;  // Arbitrary, meaningless value

  runAction<actionlib::SimpleActionClient<UnstowAction>,
            UnstowGoal,
            UnstowResultConstPtr,
            UnstowFeedbackConstPtr>
    (Op_Unstow, m_unstowClient, goal, id,
     default_action_active_cb (Op_Unstow),
     default_action_feedback_cb<UnstowFeedbackConstPtr> (Op_Unstow),
     default_action_done_cb<UnstowResultConstPtr> (Op_Unstow));
}

void OwInterface::stow (int id)  // as action
{
  if (! markOperationRunning (Op_Stow, id)) return;
  thread action_thread (&OwInterface::stowAction, this, id);
  action_thread.detach();
}

void OwInterface::stowAction (int id)
{
  StowGoal goal;
  goal.goal = 0;  // Arbitrary, meaningless value

  runAction<actionlib::SimpleActionClient<StowAction>,
            StowGoal,
            StowResultConstPtr,
            StowFeedbackConstPtr>
    (Op_Stow, m_stowClient, goal, id,
     default_action_active_cb (Op_Stow),
     default_action_feedback_cb<StowFeedbackConstPtr> (Op_Stow),
     default_action_done_cb<StowResultConstPtr> (Op_Stow));
}

void OwInterface::grind (double x, double y, double depth, double length,
                         bool parallel, double ground_pos, int id)
{
  if (! markOperationRunning (Op_Grind, id)) return;
  thread action_thread (&OwInterface::grindAction, this, x, y, depth, length,
                        parallel, ground_pos, id);
  action_thread.detach();
}

void OwInterface::grindAction (double x, double y, double depth, double length,
                               bool parallel, double ground_pos, int id)
{
  GrindGoal goal;
  goal.x_start = x;
  goal.y_start = y;
  goal.depth = depth;
  goal.length = length;
  goal.parallel = parallel;
  goal.ground_position = ground_pos;

  ROS_INFO ("Starting Grind"
            "(x=%.2f, y=%.2f, depth=%.2f, length=%.2f, "
            "parallel=%s, ground_pos=%.2f)",
            x, y, depth, length, (parallel ? "true" : "false"), ground_pos);

  runAction<actionlib::SimpleActionClient<GrindAction>,
            GrindGoal,
            GrindResultConstPtr,
            GrindFeedbackConstPtr>
    (Op_Grind, m_grindClient, goal, id,
     default_action_active_cb (Op_Grind),
     default_action_feedback_cb<GrindFeedbackConstPtr> (Op_Grind),
     default_action_done_cb<GrindResultConstPtr> (Op_Grind));
}

void OwInterface::guardedMove (double x, double y, double z,
                               double dir_x, double dir_y, double dir_z,
                               double search_dist, int id)
{
  if (! markOperationRunning (Op_GuardedMove, id)) return;
  thread action_thread (&OwInterface::guardedMoveAction, this, x, y, z,
                        dir_x, dir_y, dir_z, search_dist, id);
  action_thread.detach();
}

void OwInterface::guardedMoveAction (double x, double y, double z,
                                     double dir_x, double dir_y, double dir_z,
                                     double search_dist, int id)
{
  GuardedMoveGoal goal;
  goal.start.x = x;
  goal.start.y = y;
  goal.start.z = z;
  goal.normal.x = dir_x;
  goal.normal.y = dir_y;
  goal.normal.z = dir_z;
  goal.search_distance = search_dist;

  ROS_INFO ("Starting GuardedMove"
            "(x=%.2f, y=%.2f, z=%.2f, dir_x=%.2f, dir_y=%.2f,"
            "dir_z=%.2f, search_dist=%.2f)",
            x, y, z, dir_x, dir_y, dir_z, search_dist);

  runAction<actionlib::SimpleActionClient<GuardedMoveAction>,
            GuardedMoveGoal,
            GuardedMoveResultConstPtr,
            GuardedMoveFeedbackConstPtr>
    (Op_GuardedMove, m_guardedMoveClient, goal, id,
     default_action_active_cb (Op_GuardedMove),
     default_action_feedback_cb<GuardedMoveFeedbackConstPtr> (Op_GuardedMove),
     guarded_move_done_cb<GuardedMoveResultConstPtr> (Op_GuardedMove));
}

void OwInterface::armMoveJoint (bool relative,
                                int joint, double angle,
                                int id)
{
  if (! markOperationRunning (Op_ArmMoveJoint, id)) return;
  thread action_thread (&OwInterface::armMoveJointAction,
                        this, relative, joint, angle, id);
  action_thread.detach();
}

void OwInterface::armMoveJointAction (bool relative,
                                      int joint, double angle,
                                      int id)
{
  ArmMoveJointGoal goal;
  goal.relative = relative;
  // NOTE: goal.joint is of type int64_t.  Assignment safe, but type
  // should be either corrected all the way up the calling tree, or
  // the message type should be changed to int32, which is more than
  // sufficient and easiest to work with.
  goal.joint = joint;
  goal.angle = angle;

  ROS_INFO ("Starting ArmMoveJoint (relative=%d, joint=%" PRId64 ", angle=%f)",
            goal.relative, goal.joint, goal.angle);

  runAction<actionlib::SimpleActionClient<ArmMoveJointAction>,
            ArmMoveJointGoal,
            ArmMoveJointResultConstPtr,
            ArmMoveJointFeedbackConstPtr>
    (Op_ArmMoveJoint, m_armMoveJointClient, goal, id,
     default_action_active_cb (Op_ArmMoveJoint),
     default_action_feedback_cb<ArmMoveJointFeedbackConstPtr> (Op_ArmMoveJoint),
     default_action_done_cb<ArmMoveJointResultConstPtr> (Op_ArmMoveJoint));
}

void OwInterface::armMoveJoints (bool relative,
                                 const vector<double>& angles,
                                 int id)
{
  if (! markOperationRunning (Op_ArmMoveJoints, id)) return;
  thread action_thread (&OwInterface::armMoveJointsAction,
                        this, relative, angles, id);
  action_thread.detach();
}

void OwInterface::armMoveJointsAction (bool relative,
                                       const vector<double>& angles,
                                       int id)
{

  ArmMoveJointsGoal goal;
  goal.relative = relative;
  std::copy(angles.begin(), angles.end(), back_inserter(goal.angles));

  ROS_INFO ("Starting ArmMoveJoints"
            "(relative=%d, angles=[%f, %f, %f, %f, %f, %f])",
            goal.relative,
            goal.angles[0], goal.angles[1], goal.angles[2],
            goal.angles[3], goal.angles[4], goal.angles[5]);

  runAction<actionlib::SimpleActionClient<ArmMoveJointsAction>,
            ArmMoveJointsGoal,
            ArmMoveJointsResultConstPtr,
            ArmMoveJointsFeedbackConstPtr>
    (Op_ArmMoveJoints, m_armMoveJointsClient, goal, id,
     default_action_active_cb (Op_ArmMoveJoints),
     default_action_feedback_cb<ArmMoveJointsFeedbackConstPtr> (Op_ArmMoveJoints),
     default_action_done_cb<ArmMoveJointsResultConstPtr> (Op_ArmMoveJoints));
}

vector<double> OwInterface::identifySampleLocation (int num_images,
                                                    const string& filter_type,
                                                    int id)
{
  SamplePoint.clear();
  GotSampleLocation = false;
  if (! markOperationRunning (Op_IdentifySampleLocation, id)) return SamplePoint;
  thread action_thread (&OwInterface::identifySampleLocationAction,
                        this, num_images, filter_type, id);
  action_thread.detach();

  ros::Rate rate(10);
  int timeout = 0;
  // We wait for a result from the action server or the 5 sec timeout before
  // returning. I dont think a timeout is strictly necessary here but because
  // this is blocking I put it in for safety.
  while(GotSampleLocation == false && timeout < SampleTimeout){
      ros::spinOnce();
      rate.sleep();
      timeout+=1;
  }
  if(timeout == SampleTimeout){
    ROS_ERROR("Did not recieve a sample point from "
              "IdentifySampleLocation before timeout.");
  }
  return SamplePoint;
}

void OwInterface::identifySampleLocationAction (int num_images,
                                                const string& filter_type,
                                                int id)
{
  ow_plexil::IdentifyLocationGoal goal;
  goal.num_images = num_images;
  goal.filter_type = filter_type;

  runAction<actionlib::SimpleActionClient<ow_plexil::IdentifyLocationAction>,
            ow_plexil::IdentifyLocationGoal,
            ow_plexil::IdentifyLocationResultConstPtr,
            ow_plexil::IdentifyLocationFeedbackConstPtr>
    (Op_IdentifySampleLocation, m_identifySampleLocationClient, goal, id,
     default_action_active_cb (Op_IdentifySampleLocation),
     default_action_feedback_cb<ow_plexil::IdentifyLocationFeedbackConstPtr>
     (Op_IdentifySampleLocation),
     identify_sample_location_done_cb<IdentifySampleLocation,
                                      ow_plexil::IdentifyLocationResultConstPtr>);
}


void OwInterface::setLightIntensity (const string& side, double intensity, int id)
{
  if (! markOperationRunning (Op_SetLightIntensity, id)) return;

  ros::ServiceClient client =
    m_genericNodeHandle->serviceClient<ow_lander::Light>("/lander/light");

  if (check_service_client (client, Op_SetLightIntensity)) {
    ow_lander::Light srv;
    srv.request.name = side;
    srv.request.intensity = intensity;
    thread service_thread (&OwInterface::callService<ow_lander::Light>,
                           this, client, srv, Op_SetLightIntensity, id);
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

bool OwInterface::hardTorqueLimitReached (const string& joint_name) const
{
  return (JointsAtHardTorqueLimit.find (joint_name) !=
          JointsAtHardTorqueLimit.end());
}

bool OwInterface::softTorqueLimitReached (const string& joint_name) const
{
  return (JointsAtSoftTorqueLimit.find (joint_name) !=
          JointsAtSoftTorqueLimit.end());
}

int OwInterface::actionGoalStatus (const string& action_name) const
{
  return ActionGoalStatusMap[action_name];
}
