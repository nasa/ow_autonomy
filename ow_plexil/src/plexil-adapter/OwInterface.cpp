// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ow_plexil
#include "OwInterface.h"
#include "subscriber.h"

// OW - external
#include <ow_lander/Light.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

// C++
#include <set>
#include <vector>
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

using namespace ow_lander;
using namespace owl_msgs;

//////////////////// Utilities ////////////////////////

const double PanTiltToleranceDegrees = 2.865; // 0.05 radians, matching simulator
const double VelocityTolerance       = 0.01;  // made up, unitless


//////////////////// Lander Operation Support ////////////////////////

// Index into /joint_states message and JointTelemetries vector.
const size_t ArmJointStartIndex = 2;

const double PointCloudTimeout = 50.0; // 5 second timeout assuming a rate of 10hz
const double SampleTimeout = 50.0; // 5 second timeout assuming a rate of 10hz

// Lander operation names.  In general these match those used in PLEXIL and
// ow_lander.

const string Op_GuardedMove            = "GuardedMove";
const string Op_ArmMoveJoint           = "ArmMoveJoint";
const string Op_ArmMoveJoints          = "ArmMoveJoints";
const string Op_DigCircular            = "DigCircular";
const string Op_DigLinear              = "DigLinear";
const string Op_TaskDeliverSample      = "TaskDeliverSample";
const string Op_Discard                = "Discard";
const string Op_Grind                  = "Grind";
const string Op_ArmStow                = "ArmStow";
const string Op_ArmUnstow              = "ArmUnstow";
const string Op_CameraCapture          = "CameraCapture";
const string Op_CameraSetExposure      = "CameraSetExposure";
const string Op_PanTiltAntenna         = "AntennaPanTiltAction";
const string Op_IdentifySampleLocation = "IdentifySampleLocation";
const string Op_LightSetIntensity      = "LightSetIntensity";

static vector<string> LanderOpNames = {
  Op_GuardedMove,
  Op_ArmMoveJoint,
  Op_ArmMoveJoints,
  Op_DigCircular,
  Op_DigLinear,
  Op_TaskDeliverSample,
  Op_Discard,
  Op_PanTiltAntenna,
  Op_Grind,
  Op_ArmStow,
  Op_ArmUnstow,
  Op_CameraCapture,
  Op_CameraSetExposure,
  Op_IdentifySampleLocation,
  Op_LightSetIntensity
};


///////////////////////// Action Goal Status Support /////////////////////////

// Duplication of actionlib_msgs/GoalStatus.h with the addition of a
// NOGOAL status for when the action is not running.
//
enum ActionGoalStatus {
  NOGOAL = -1,
  PENDING = 0,
  ACTIVE = 1,
  PREEMPTED = 2,
  SUCCEEDED = 3,
  ABORTED = 4,
  REJECTED = 5,
  PREEMPTING = 6,
  RECALLING = 7,
  RECALLED = 8,
  LOST = 9
};

static map<string, int> ActionGoalStatusMap {
  // ROS action name -> Action goal status
  { Op_ArmStow, NOGOAL },
  { Op_ArmUnstow, NOGOAL },
  { Op_Grind, NOGOAL },
  { Op_GuardedMove, NOGOAL },
  { Op_ArmMoveJoint, NOGOAL },
  { Op_ArmMoveJoints, NOGOAL },
  { Op_DigCircular, NOGOAL },
  { Op_DigLinear, NOGOAL },
  { Op_TaskDeliverSample, NOGOAL },
  { Op_Discard, NOGOAL },
  { Op_CameraCapture, NOGOAL },
  { Op_CameraSetExposure, NOGOAL },
  { Op_PanTiltAntenna, NOGOAL },
  { Op_IdentifySampleLocation, NOGOAL },
  { Op_LightSetIntensity, NOGOAL }
};

static void update_action_goal_state (string action, int state)
{
  if (ActionGoalStatusMap.find(action) != ActionGoalStatusMap.end()) {
    ActionGoalStatusMap[action] = state;
  }
  else {
    ROS_ERROR("Unknown action: %s", action.c_str());
  }
}

/////////////////////////// Joint/Torque Support ///////////////////////////////

static set<string> JointsAtHardTorqueLimit { };
static set<string> JointsAtSoftTorqueLimit { };

static vector<JointProperties> JointProps {
  // NOTE: Torque limits are made up, as no reference specs are yet
  // known, and only magnitude is provided for now.

  { "AntennaPan", 30, 30 },
  { "AntennaTilt", 30, 30 },
  { "DistalPitch", 60, 80 },
  { "Grinder", 30, 30 },
  { "HandYaw", 60, 80 },
  { "ProximalPitch", 60, 80 },
  { "ScoopYaw", 60, 80 },
  { "ShoulderPitch", 60, 80 },
  { "ShoulderYaw", 60, 80 }
};

static vector<JointTelemetry> JointTelemetries (NumJoints, { 0, 0, 0, 0 });

static void handle_overtorque (int joint, double effort)
{
  // For now, torque is just effort (Newton-meter), and overtorque is specific
  // to the joint.

  string joint_name = JointProps[joint].plexilName;

  if (fabs(effort) >= JointProps[joint].hardTorqueLimit) {
    JointsAtHardTorqueLimit.insert (joint_name);
  }
  else if (fabs(effort) >= JointProps[joint].softTorqueLimit) {
    JointsAtSoftTorqueLimit.insert(joint_name);
  }
  else {
    JointsAtHardTorqueLimit.erase (joint_name);
    JointsAtSoftTorqueLimit.erase (joint_name);
  }
}

static void handle_joint_fault (int joint_index,
                                const sensor_msgs::JointState::ConstPtr& msg)
{
  // NOTE: For now, the only fault is overtorque.
  handle_overtorque (joint_index, msg->effort[joint_index]);
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
(const  owl_msgs::SystemFaultsStatus::ConstPtr& msg)
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

static double normalize_degrees (double angle)
{
  static double pi = R2D * M_PI;
  static double tau = pi * 2.0;
  double x = fmod(angle + pi, tau);
  if (x < 0) x += tau;
  return x - pi;
}

void OwInterface::armJointAccelerationsCb
(const owl_msgs::ArmJointAccelerations::ConstPtr& msg)
{
  // Joint acceleration is computed only for arm joints and not the
  // two antenna joints, so appropiate sanity check.
  size_t arm_joints = NumJoints - ArmJointStartIndex;
  size_t msg_size = msg->value.size();
  if (msg_size != arm_joints) {
    ROS_ERROR_ONCE ("OwInterface::armJointAccelerationsCb: "
                    "Number of actual joints, %zu, "
                    "doesn't match number of known arm joints, %zu. "
                    "This should never happen.",
                    msg_size, ArmJointStartIndex);
    return;
  }

  for (int i = 0; i < msg_size; i++) {
    double acceleration = msg->value[i];
    JointTelemetries[i + ArmJointStartIndex].acceleration = acceleration;
    publish ("JointAcceleration", acceleration, i + ArmJointStartIndex);
  }
}

void OwInterface::jointStatesCallback
(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Publish all joint information for visibility to PLEXIL and handle any
  // joint-related faults.

  size_t msg_size = msg->name.size();

  // Sanity check
  if (msg_size != NumJoints) {
    ROS_ERROR_ONCE ("OwInterface::jointStatesCallback: "
                    "Number of actual joints, %zu, "
                    "doesn't match number of known joints, %zu. "
                    "This should never happen.",
                    msg_size, NumJoints);
    return;
  }

  for (int i = 0; i < msg_size; i++) {
    double position = msg->position[i];
    double velocity = msg->velocity[i];
    double effort = msg->effort[i];
    if (i == ANTENNA_PAN) {
      m_currentPanRadians = position;
      publish ("PanRadians", m_currentPanRadians);
      publish ("PanDegrees", m_currentPanRadians * R2D);
    }
    else if (i == ANTENNA_TILT) {
      m_currentTiltRadians = position;
      publish ("TiltRadians", m_currentTiltRadians);
      publish ("TiltDegrees", m_currentTiltRadians * R2D);
    }
    JointTelemetries[i] = JointTelemetry {position, velocity, effort};
    publish ("JointPosition", position, i);
    publish ("JointVelocity", velocity, i);
    publish ("JointEffort", effort, i);
    handle_joint_fault (i, msg);
  }
}


///////////////////////// Antenna/Camera Support ///////////////////////////////

bool OwInterface::anglesEquivalent (double deg1, double deg2, double tolerance)
{
  return fabs(normalize_degrees(deg1 - deg2)) <= tolerance;
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

/// Queue size for subscribers is a guess at adequacy.
const int QSize = 3;

void OwInterface::actionGoalStatusCallback
(const actionlib_msgs::GoalStatusArray::ConstPtr& msg, const string action_name)

// Update ActionGoalStatusMap of action action_name with the status
// from first goal in GoalStatusArray msg. This is based on the
// assumption that no action will have more than one goal in our
// system.
{
  if (msg->status_list.size() == 0) {
    int status = NOGOAL;
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
template<typename T>
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
  : m_currentPanRadians (0),
    m_currentTiltRadians (0)
{ }

void OwInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {

    for (const string& name : LanderOpNames) {
      registerLanderOperation (name);
    }

    m_genericNodeHandle = make_unique<ros::NodeHandle>();

    // Initialize action clients

    m_guardedMoveClient =
      make_unique<GuardedMoveActionClient>(Op_GuardedMove, true);
    m_armMoveJointClient =
      make_unique<ArmMoveJointActionClient>(Op_ArmMoveJoint, true);
    m_armMoveJointsClient =
      make_unique<ArmMoveJointsActionClient>(Op_ArmMoveJoints, true);
    m_armUnstowClient =
      make_unique<ArmUnstowActionClient>(Op_ArmUnstow, true);
    m_armStowClient =
      make_unique<ArmStowActionClient>(Op_ArmStow, true);
    m_grindClient =
      make_unique<GrindActionClient>(Op_Grind, true);
    m_digCircularClient =
      make_unique<DigCircularActionClient>(Op_DigCircular, true);
    m_digLinearClient =
      make_unique<DigLinearActionClient>(Op_DigLinear, true);
    m_taskDeliverSampleClient =
      make_unique<TaskDeliverSampleActionClient>(Op_TaskDeliverSample, true);
    m_discardClient =
      make_unique<DiscardActionClient>(Op_Discard, true);
    m_cameraCaptureClient =
      make_unique<CameraCaptureActionClient>(Op_CameraCapture, true);
    m_cameraSetExposureClient =
      make_unique<CameraSetExposureActionClient>(Op_CameraSetExposure, true);
    m_lightSetIntensityClient =
      make_unique<LightSetIntensityActionClient>(Op_LightSetIntensity, true);
    m_identifySampleLocationClient =
      make_unique<IdentifySampleLocationActionClient>
      (Op_IdentifySampleLocation, true);
    m_panTiltClient = make_unique<PanTiltActionClient>(Op_PanTiltAntenna, true);

    // Initialize publishers.  For now, latching in lieu of waiting
    // for publishers.

    const bool latch = true;
    m_antennaTiltPublisher = make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_tilt_position_controller/command", QSize, latch));
    m_antennaPanPublisher = make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_pan_position_controller/command", QSize, latch));
    m_leftImageTriggerPublisher = make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::Empty>
       ("/StereoCamera/left/image_trigger", QSize, latch));

    // Initialize subscribers

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->subscribe("/joint_states", QSize,
                                        &OwInterface::jointStatesCallback,
                                        this)));
    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->subscribe("/arm_joint_accelerations", QSize,
                                        &OwInterface::armJointAccelerationsCb,
                                        this)));
    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/power_system_node/state_of_charge", QSize, soc_callback)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/power_system_node/battery_temperature", QSize,
                  temperature_callback)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/power_system_node/remaining_useful_life", QSize,
                  rul_callback)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/system_faults_status", QSize,
                  &OwInterface::systemFaultMessageCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/faults/arm_faults_status", QSize,
                  &OwInterface::armFaultCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/faults/power_faults_status", QSize,
                  &OwInterface::powerFaultCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/faults/pt_faults_status", QSize,
                  &OwInterface::antennaFaultCallback, this)));

    // Connect action clients to servers and add subscribers for
    // action status.

    if (! m_armUnstowClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("ArmUnstow action server did not connect!");
    }
    else addSubscriber ("/ArmUnstow/status", Op_ArmUnstow);

    if (! m_armStowClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("ArmStow action server did not connect!");
    }
    else addSubscriber ("/ArmStow/status", Op_ArmStow);

    if (! m_armMoveJointClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS)))  {
      ROS_ERROR ("ArmMoveJoint action server did not connect!");
    }
    else addSubscriber ("/ArmMoveJoint/status", Op_ArmMoveJoint);

    if (! m_armMoveJointsClient ->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("ArmMoveJoints action server did not connect!");
    }
    else addSubscriber ("/ArmMoveJoints/status", Op_ArmMoveJoints);

    if (! m_digCircularClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("DigCircular action server did not connect!");
    }
    else addSubscriber ("/DigCircular/status", Op_DigCircular);

    if (! m_digLinearClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("DigLinear action server did not connect!");
    }
    else addSubscriber ("/DigLinear/status", Op_DigLinear);

    if (! m_taskDeliverSampleClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("TaskDeliverSample action server did not connect!");
    }
    else addSubscriber ("/TaskDeliverSample/status", Op_TaskDeliverSample);

    if (! m_discardClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("Discard action server did not connect!");
    }
    else addSubscriber ("/Discard/status", Op_Discard);

    if (! m_cameraCaptureClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("CameraCapture action server did not connect!");
    }
    else addSubscriber ("/CameraCapture/status", Op_CameraCapture);

    if (! m_cameraSetExposureClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("CameraSetExposure action server did not connect!");
    }
    else addSubscriber ("/CameraSetExposure/status", Op_CameraSetExposure);

    if (! m_lightSetIntensityClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("LightSetIntensity action server did not connect!");
    }
    else addSubscriber ("/LightSetIntensity/status", Op_LightSetIntensity);

    if (! m_guardedMoveClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("GuardedMove action server did not connect!");
    }
    else addSubscriber ("/GuardedMove/status", Op_GuardedMove);

    if (! m_panTiltClient->
        waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("Antenna pan/tilt action server did not connect!");
    }
    if (! m_identifySampleLocationClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("IdentifySampleLocation action server did not connect!");
    }
  }
}


void OwInterface::addSubscriber (const string& topic, const string& operation)
{
  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
      (topic, QSize,
       boost::bind(&OwInterface::actionGoalStatusCallback,
                   this, _1, operation))));
}


void OwInterface::panTiltAntenna (double pan_degrees, double tilt_degrees, int id)
{
  if (! markOperationRunning (Op_PanTiltAntenna, id)) return;
  thread action_thread (&OwInterface::panTiltAntennaAction, this,
                        pan_degrees, tilt_degrees, id);
  action_thread.detach();
}

void OwInterface::panTiltAntennaAction (double pan_degrees, double tilt_degrees,
                                        int id)
{
  AntennaPanTiltGoal goal;
  goal.pan = pan_degrees * D2R;
  goal.tilt = tilt_degrees * D2R;
  std::stringstream args;
  args << goal.pan << ", " << goal.tilt;
  runAction<actionlib::SimpleActionClient<AntennaPanTiltAction>,
            AntennaPanTiltGoal,
            AntennaPanTiltResultConstPtr,
            AntennaPanTiltFeedbackConstPtr>
    (Op_PanTiltAntenna, m_panTiltClient, goal, id,
     default_action_active_cb (Op_PanTiltAntenna, args.str()),
     default_action_feedback_cb<AntennaPanTiltFeedbackConstPtr> (Op_PanTiltAntenna),
     default_action_done_cb<AntennaPanTiltResultConstPtr> (Op_PanTiltAntenna));
}

void OwInterface::cameraCapture (int id)
{
  if (! markOperationRunning (Op_CameraCapture, id)) return;
  thread action_thread (&OwInterface::cameraCaptureAction, this, id);
  action_thread.detach();
}


void OwInterface::cameraCaptureAction (int id)
{
  CameraCaptureGoal goal;

  ROS_INFO ("Starting CameraCapture()");

  runAction<actionlib::SimpleActionClient<CameraCaptureAction>,
            CameraCaptureGoal,
            CameraCaptureResultConstPtr,
            CameraCaptureFeedbackConstPtr>
    (Op_CameraCapture, m_cameraCaptureClient, goal, id,
     default_action_active_cb (Op_CameraCapture),
     default_action_feedback_cb<CameraCaptureFeedbackConstPtr> (Op_CameraCapture),
     default_action_done_cb<CameraCaptureResultConstPtr> (Op_CameraCapture));
}


void OwInterface::cameraSetExposure (double exposure_secs, int id)
{
  if (! markOperationRunning (Op_CameraSetExposure, id)) return;
  thread action_thread (&OwInterface::cameraSetExposureAction, this, exposure_secs, id);
  action_thread.detach();
}


void OwInterface::cameraSetExposureAction (double exposure_secs, int id)
{
  CameraSetExposureGoal goal;
  goal.exposure = exposure_secs;

  ROS_INFO ("Starting CameraSetExposure(exposure_secs=%.2f)", exposure_secs);

  runAction<actionlib::SimpleActionClient<CameraSetExposureAction>,
            CameraSetExposureGoal,
            CameraSetExposureResultConstPtr,
            CameraSetExposureFeedbackConstPtr>
    (Op_CameraSetExposure, m_cameraSetExposureClient, goal, id,
     default_action_active_cb (Op_CameraSetExposure),
     default_action_feedback_cb<CameraSetExposureFeedbackConstPtr> (Op_CameraSetExposure),
     default_action_done_cb<CameraSetExposureResultConstPtr> (Op_CameraSetExposure));
}


void OwInterface::taskDeliverSample (int id)
{
  if (! markOperationRunning (Op_TaskDeliverSample, id)) return;
  thread action_thread (&OwInterface::taskDeliverSampleAction, this, id);
  action_thread.detach();
}

void OwInterface::discard (double x, double y, double z, int id)
{
  if (! markOperationRunning (Op_Discard, id)) return;
  thread action_thread (&OwInterface::discardAction, this, x, y, z, id);
  action_thread.detach();
}


void OwInterface::taskDeliverSampleAction (int id)
{
  TaskDeliverSampleGoal goal;

  ROS_INFO ("Starting TaskDeliverSample()");

  runAction<actionlib::SimpleActionClient<TaskDeliverSampleAction>,
            TaskDeliverSampleGoal,
            TaskDeliverSampleResultConstPtr,
            TaskDeliverSampleFeedbackConstPtr>
    (Op_TaskDeliverSample, m_taskDeliverSampleClient, goal, id,
     default_action_active_cb (Op_TaskDeliverSample),
     default_action_feedback_cb<TaskDeliverSampleFeedbackConstPtr> (Op_TaskDeliverSample),
     default_action_done_cb<TaskDeliverSampleResultConstPtr> (Op_TaskDeliverSample));
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


void OwInterface::armUnstow (int id)  // as action
{
  if (! markOperationRunning (Op_ArmUnstow, id)) return;
  thread action_thread (&OwInterface::armUnstowAction, this, id);
  action_thread.detach();
}

void OwInterface::armUnstowAction (int id)
{
  ArmUnstowGoal goal; // empty/undefined

  runAction<actionlib::SimpleActionClient<ArmUnstowAction>,
            ArmUnstowGoal,
            ArmUnstowResultConstPtr,
            ArmUnstowFeedbackConstPtr>
    (Op_ArmUnstow, m_armUnstowClient, goal, id,
     default_action_active_cb (Op_ArmUnstow),
     default_action_feedback_cb<ArmUnstowFeedbackConstPtr> (Op_ArmUnstow),
     default_action_done_cb<ArmUnstowResultConstPtr> (Op_ArmUnstow));
}

void OwInterface::armStow (int id)  // as action
{
  if (! markOperationRunning (Op_ArmStow, id)) return;
  thread action_thread (&OwInterface::armStowAction, this, id);
  action_thread.detach();
}

void OwInterface::armStowAction (int id)
{
  ArmStowGoal goal; // empty/undefined

  runAction<actionlib::SimpleActionClient<ArmStowAction>,
            ArmStowGoal,
            ArmStowResultConstPtr,
            ArmStowFeedbackConstPtr>
    (Op_ArmStow, m_armStowClient, goal, id,
     default_action_active_cb (Op_ArmStow),
     default_action_feedback_cb<ArmStowFeedbackConstPtr> (Op_ArmStow),
     default_action_done_cb<ArmStowResultConstPtr> (Op_ArmStow));
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
     identify_sample_location_done_cb<ow_plexil::IdentifyLocationResultConstPtr>);
}


void OwInterface::lightSetIntensity (const string& side, double intensity,
                                     int id)
{
  if (! markOperationRunning (Op_LightSetIntensity, id)) return;
  thread action_thread (&OwInterface::lightSetIntensityAction, this,
                        side, intensity, id);
  action_thread.detach();
}


void OwInterface::lightSetIntensityAction (const string& side, double intensity,
                                           int id)
{
  LightSetIntensityGoal goal;
  goal.name = side;
  goal.intensity = intensity;

  ROS_INFO ("Starting LightSetIntensity(side=%s, intensity=%.2f)", side.c_str(),
            intensity);

  runAction<actionlib::SimpleActionClient<LightSetIntensityAction>,
            LightSetIntensityGoal,
            LightSetIntensityResultConstPtr,
            LightSetIntensityFeedbackConstPtr>
    (Op_LightSetIntensity, m_lightSetIntensityClient, goal, id,
     default_action_active_cb (Op_LightSetIntensity),
     default_action_feedback_cb<LightSetIntensityFeedbackConstPtr>
     (Op_LightSetIntensity),
     default_action_done_cb<LightSetIntensityResultConstPtr>
     (Op_LightSetIntensity));
}


double OwInterface::getTiltRadians () const
{
  return m_currentTiltRadians;
}

double OwInterface::getTiltDegrees () const
{
  return m_currentTiltRadians * R2D;
}

double OwInterface::getPanRadians () const
{
  return m_currentPanRadians;
}

double OwInterface::getPanDegrees () const
{
  return m_currentPanRadians * R2D;
}

double OwInterface::getPanVelocity () const
{
  return JointTelemetries[ANTENNA_PAN].velocity;
}

double OwInterface::getTiltVelocity () const
{
  return JointTelemetries[ANTENNA_TILT].velocity;
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

double OwInterface::jointTelemetry (int joint, TelemetryType type) const
{
  if (joint >= 0 && joint < NumJoints) {
    switch (type) {
      case TelemetryType::Position: return JointTelemetries[joint].position;
      case TelemetryType::Velocity: return JointTelemetries[joint].velocity;
      case TelemetryType::Effort: return JointTelemetries[joint].effort;
      case TelemetryType::Acceleration: {
        if (joint >= ArmJointStartIndex) return JointTelemetries[joint].acceleration;
        ROS_WARN ("jointTelemetry: acceleration not available for antenna joint.");
        break;
      }
      default:
        ROS_ERROR ("jointTelemetry: unsupported telemetry type.");
        break;
    }
  }
  else {
    ROS_ERROR ("jointTelemetry: invalid joint index %d", joint);
  }
  return 0;
}
