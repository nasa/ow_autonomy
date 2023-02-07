// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ow_plexil
#include "OwInterface.h"
#include "subscriber.h"

// OW - external
#include <owl_msgs/LightSetIntensity.h>
#include <owl_msgs/BatteryRemainingUsefulLife.h>
#include <owl_msgs/BatteryStateOfCharge.h>
#include <owl_msgs/BatteryTemperature.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++
#include <set>
#include <vector>
#include <map>
#include <thread>
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
// owl_msgs.

const string Op_ArmMoveJoint            = "ArmMoveJoint";
const string Op_ArmMoveJoints           = "ArmMoveJoints";
const string Op_ArmStop                 = "ArmStop";
const string Op_ArmStow                 = "ArmStow";
const string Op_ArmUnstow               = "ArmUnstow";
const string Op_ArmFindSurface          = "ArmFindSurface";
const string Op_ArmMoveCartesian        = "ArmMoveCartesian";
const string Op_ArmMoveCartesianGuarded = "ArmMoveCartesianGuarded";
const string Op_ArmMoveJointsGuarded    = "ArmMoveJointsGuarded";
const string Op_CameraCapture           = "CameraCapture";
const string Op_CameraSetExposure       = "CameraSetExposure";
const string Op_Ingest                  = "DockIngestSample";
const string Op_Grind                   = "TaskGrind";
const string Op_GuardedMove             = "GuardedMove";
const string Op_IdentifySampleLocation  = "IdentifySampleLocation";
const string Op_LightSetIntensity       = "LightSetIntensity";
const string Op_Pan                     = "PanAction";
const string Op_PanTilt                 = "PanTiltMoveJointsAction";
const string Op_PanTiltCartesian        = "PanTiltMoveCartesianAction";
const string Op_TaskDeliverSample       = "TaskDeliverSample";
const string Op_TaskDiscardSample       = "TaskDiscardSample";
const string Op_TaskScoopCircular       = "TaskScoopCircular";
const string Op_TaskScoopLinear         = "TaskScoopLinear";
const string Op_Tilt                    = "TiltAction";

static vector<string> LanderOpNames = {
  Op_GuardedMove,
  Op_ArmFindSurface,
  Op_ArmMoveCartesian,
  Op_ArmMoveCartesianGuarded,
  Op_ArmMoveJoint,
  Op_ArmMoveJoints,
  Op_ArmMoveJointsGuarded,
  Op_TaskDeliverSample,
  Op_Pan,
  Op_Tilt,
  Op_PanTilt,
  Op_PanTiltCartesian,
  Op_TaskScoopCircular,
  Op_TaskScoopLinear,
  Op_TaskDiscardSample,
  Op_Grind,
  Op_ArmStop,
  Op_ArmStow,
  Op_ArmUnstow,
  Op_CameraCapture,
  Op_CameraSetExposure,
  Op_Ingest,
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
  { Op_ArmFindSurface, NOGOAL },
  { Op_ArmMoveCartesian, NOGOAL },
  { Op_ArmMoveCartesianGuarded, NOGOAL },
  { Op_ArmStop, NOGOAL },
  { Op_ArmStow, NOGOAL },
  { Op_ArmUnstow, NOGOAL },
  { Op_Grind, NOGOAL },
  { Op_GuardedMove, NOGOAL },
  { Op_ArmMoveJoint, NOGOAL },
  { Op_ArmMoveJoints, NOGOAL },
  { Op_ArmMoveJointsGuarded, NOGOAL },
  { Op_TaskDeliverSample, NOGOAL },
  { Op_TaskScoopCircular, NOGOAL },
  { Op_TaskScoopLinear, NOGOAL },
  { Op_TaskDiscardSample, NOGOAL },
  { Op_CameraCapture, NOGOAL },
  { Op_CameraSetExposure, NOGOAL },
  { Op_Ingest, NOGOAL },
  { Op_Pan, NOGOAL },
  { Op_Tilt, NOGOAL },
  { Op_PanTilt, NOGOAL },
  { Op_PanTiltCartesian, NOGOAL },
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
(const owl_msgs::ArmFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_armErrors, "ARM", "ArmFault");
}

void OwInterface::powerFaultCallback
(const owl_msgs::PowerFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_powerErrors, "POWER", "PowerFault");
}

void OwInterface::antennaFaultCallback
(const owl_msgs::PanTiltFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_panTiltErrors, "ANTENNA", "AntennaFault");
}

void OwInterface::cameraFaultCallback
(const owl_msgs::CameraFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_cameraErrors, "CAMERA", "CameraFault");
}

void OwInterface::ftCallback
(const owl_msgs::ArmEndEffectorForceTorque::ConstPtr& msg)
{
  m_endEffectorFT[0] = msg->value.force.x;
  m_endEffectorFT[1] = msg->value.force.y;
  m_endEffectorFT[2] = msg->value.force.z;
  m_endEffectorFT[3] = msg->value.torque.x;
  m_endEffectorFT[4] = msg->value.torque.y;
  m_endEffectorFT[5] = msg->value.torque.z;
}

void OwInterface::armPoseCallback (const owl_msgs::ArmPose::ConstPtr& msg)
{
  m_armPose[0] = msg->value.position.x;
  m_armPose[1] = msg->value.position.y;
  m_armPose[2] = msg->value.position.z;
  m_armPose[3] = msg->value.orientation.x;
  m_armPose[4] = msg->value.orientation.y;
  m_armPose[5] = msg->value.orientation.z;
  m_armPose[6] = msg->value.orientation.w;
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

static double SOC = NAN;
static double RUL = NAN;
static double BatteryTemp = NAN;

static void soc_callback (const owl_msgs::BatteryStateOfCharge::ConstPtr& msg)
{
  SOC = msg->value;
  publish ("StateOfCharge", SOC);
}

static void rul_callback (const owl_msgs::BatteryRemainingUsefulLife::ConstPtr& msg)
{
  // NOTE: This is not being called as of 4/12/21.  Jira OW-656 addresses.
  RUL = msg->value;
  publish ("RemainingUsefulLife", RUL);
}

static void temperature_callback (const owl_msgs::BatteryTemperature::ConstPtr& msg)
{
  BatteryTemp = msg->value;
  publish ("BatteryTemperature", BatteryTemp);
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
  return antennaPanFault() || antennaTiltFault();
}

bool OwInterface::antennaPanFault () const
{
  return m_panTiltErrors.at(FaultPanJointLocked).second;
}

bool OwInterface::antennaTiltFault () const
{
  return m_panTiltErrors.at(FaultTiltJointLocked).second;
}

bool OwInterface::armFault () const
{
  return faultActive (m_armErrors);
}

bool OwInterface::powerFault () const
{
  return faultActive (m_powerErrors);
}

bool OwInterface::cameraFault () const
{
  return faultActive (m_cameraErrors);
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
{
  m_endEffectorFT.resize(6);
  m_endEffectorFT = {0,0,0,0,0,0};
  m_armPose.resize(7);
  m_armPose = {0,0,0,0,0,0,0};
}

void OwInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {

    for (const string& name : LanderOpNames) {
      registerLanderOperation (name);
    }

    m_genericNodeHandle = make_unique<ros::NodeHandle>();

    // Initialize action clients

    m_armFindSurfaceClient =
      make_unique<ArmFindSurfaceActionClient>(Op_ArmFindSurface, true);
    m_armMoveCartesianClient =
      make_unique<ArmMoveCartesianActionClient>(Op_ArmMoveCartesian, true);
    m_armMoveCartesianGuardedClient =
      make_unique<ArmMoveCartesianGuardedActionClient>(Op_ArmMoveCartesianGuarded,
                                                       true);
    m_guardedMoveClient =
      make_unique<GuardedMoveActionClient>(Op_GuardedMove, true);
    m_armMoveJointClient =
      make_unique<ArmMoveJointActionClient>(Op_ArmMoveJoint, true);
    m_armMoveJointsClient =
      make_unique<ArmMoveJointsActionClient>(Op_ArmMoveJoints, true);
    m_armMoveJointsGuardedClient =
      make_unique<ArmMoveJointsGuardedActionClient>(Op_ArmMoveJointsGuarded, true);
    m_armStopClient =
      make_unique<ArmStopActionClient>(Op_ArmStop, true);
    m_armUnstowClient =
      make_unique<ArmUnstowActionClient>(Op_ArmUnstow, true);
    m_armStowClient =
      make_unique<ArmStowActionClient>(Op_ArmStow, true);
    m_grindClient =
      make_unique<TaskGrindActionClient>(Op_Grind, true);
    m_taskDeliverSampleClient =
      make_unique<TaskDeliverSampleActionClient>(Op_TaskDeliverSample, true);
    m_scoopCircularClient =
      make_unique<TaskScoopCircularActionClient>(Op_TaskScoopCircular, true);
    m_scoopLinearClient =
      make_unique<TaskScoopLinearActionClient>(Op_TaskScoopLinear, true);
    m_discardClient =
      make_unique<TaskDiscardSampleActionClient>(Op_TaskDiscardSample, true);
    m_cameraCaptureClient =
      make_unique<CameraCaptureActionClient>(Op_CameraCapture, true);
    m_cameraSetExposureClient =
      make_unique<CameraSetExposureActionClient>(Op_CameraSetExposure, true);
    m_dockIngestSampleClient =
      make_unique<DockIngestSampleActionClient>(Op_Ingest, true);
    m_lightSetIntensityClient =
      make_unique<LightSetIntensityActionClient>(Op_LightSetIntensity, true);
    m_identifySampleLocationClient =
      make_unique<IdentifySampleLocationActionClient>
      (Op_IdentifySampleLocation, true);
    m_panClient = make_unique<PanActionClient>(Op_Pan, true);
    m_tiltClient = make_unique<TiltActionClient>(Op_Tilt, true);
    m_panTiltClient = make_unique<PanTiltMoveJointsActionClient>(Op_PanTilt, true);
    m_panTiltCartesianClient =
      make_unique<PanTiltMoveCartesianActionClient>(Op_PanTiltCartesian, true);

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
        subscribe("/battery_state_of_charge", QSize, soc_callback)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/battery_temperature", QSize,
                  temperature_callback)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/battery_remaining_useful_life", QSize,
                  rul_callback)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/system_faults_status", QSize,
                  &OwInterface::systemFaultMessageCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/arm_faults_status", QSize,
                  &OwInterface::armFaultCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/power_faults_status", QSize,
                  &OwInterface::powerFaultCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/pan_tilt_faults_status", QSize,
                  &OwInterface::antennaFaultCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/camera_faults_status", QSize,
                  &OwInterface::cameraFaultCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/arm_end_effector_force_torque", QSize,
                  &OwInterface::ftCallback, this)));

    connectActionServer (m_armStopClient, Op_ArmStop, "/ArmStop/status");
    connectActionServer (m_armUnstowClient, Op_ArmUnstow, "/ArmUnstow/status");
    connectActionServer (m_armStowClient, Op_ArmStow, "/ArmStow/status");
    connectActionServer (m_armMoveJointClient, Op_ArmMoveJoint,
                         "/ArmMoveJoint/status");
    connectActionServer (m_armMoveJointsClient, Op_ArmMoveJoints,
                         "/ArmMoveJoints/status");
    connectActionServer (m_armMoveJointsGuardedClient, Op_ArmMoveJointsGuarded,
                         "/ArmMoveJointsGuarded/status");
    connectActionServer (m_scoopCircularClient, Op_TaskScoopCircular,
                         "/TaskScoopCircular/status");
    connectActionServer (m_scoopLinearClient, Op_TaskScoopLinear,
                         "/TaskScoopLinear/status");
    connectActionServer (m_taskDeliverSampleClient, Op_TaskDeliverSample,
                         "/TaskDeliverSample/status");
    connectActionServer (m_discardClient, Op_TaskDiscardSample,
                         "/TaskDiscardSample/status");
    connectActionServer (m_cameraCaptureClient, Op_CameraCapture,
                         "/CameraCapture/status");
    connectActionServer (m_cameraSetExposureClient, Op_CameraSetExposure,
                         "/CameraSetExposure/status");
    connectActionServer (m_dockIngestSampleClient, Op_Ingest,
                         "/DockIngestSample/status");
    connectActionServer (m_lightSetIntensityClient, Op_LightSetIntensity,
                         "/LightSetIntensity/status");
    connectActionServer (m_guardedMoveClient, Op_GuardedMove,
                         "/GuardedMove/status");
    connectActionServer (m_armMoveCartesianClient, Op_ArmMoveCartesian,
                         "/ArmMoveCartesian/status");
    connectActionServer (m_armMoveCartesianGuardedClient, Op_ArmMoveCartesianGuarded,
                         "/ArmMoveCartesianGuarded/status");
    connectActionServer (m_armFindSurfaceClient, Op_ArmFindSurface,
                         "/ArmFindSurface/status");
    connectActionServer (m_panClient, Op_Pan);
    connectActionServer (m_tiltClient, Op_Tilt);
    connectActionServer (m_panTiltClient, Op_PanTilt);
    connectActionServer (m_panTiltCartesianClient, Op_PanTiltCartesian);
    connectActionServer (m_identifySampleLocationClient, Op_IdentifySampleLocation);
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

void OwInterface::pan (double degrees, int id)
{
  if (! markOperationRunning (Op_Pan, id)) return;
  thread action_thread (&OwInterface::panAction, this, degrees, id);
  action_thread.detach();
}

void OwInterface::panAction (double degrees, int id)
{
  PanGoal goal;
  goal.pan = degrees * D2R;
  std::stringstream args;
  args << goal.pan;
  runAction<actionlib::SimpleActionClient<PanAction>,
            PanGoal,
            PanResultConstPtr,
            PanFeedbackConstPtr>
    (Op_Pan, m_panClient, goal, id,
     default_action_active_cb (Op_Pan, args.str()),
     default_action_feedback_cb<PanFeedbackConstPtr> (Op_Pan),
     default_action_done_cb<PanResultConstPtr> (Op_Pan));
}

void OwInterface::tilt (double degrees, int id)
{
  if (! markOperationRunning (Op_Tilt, id)) return;
  thread action_thread (&OwInterface::tiltAction, this, degrees, id);
  action_thread.detach();
}

void OwInterface::tiltAction (double degrees, int id)
{
  TiltGoal goal;
  goal.tilt = degrees * D2R;
  std::stringstream args;
  args << goal.tilt;
  runAction<actionlib::SimpleActionClient<TiltAction>,
            TiltGoal,
            TiltResultConstPtr,
            TiltFeedbackConstPtr>
    (Op_Tilt, m_tiltClient, goal, id,
     default_action_active_cb (Op_Tilt, args.str()),
     default_action_feedback_cb<TiltFeedbackConstPtr> (Op_Tilt),
     default_action_done_cb<TiltResultConstPtr> (Op_Tilt));
}

void OwInterface::panTilt (double pan_degrees, double tilt_degrees, int id)
{
  if (! markOperationRunning (Op_PanTilt, id)) return;
  thread action_thread (&OwInterface::panTiltAction, this,
                        pan_degrees, tilt_degrees, id);
  action_thread.detach();
}

void OwInterface::panTiltAction (double pan_degrees, double tilt_degrees, int id)
{
  PanTiltMoveJointsGoal goal;
  goal.pan = pan_degrees * D2R;
  goal.tilt = tilt_degrees * D2R;
  std::stringstream args;
  args << goal.pan << ", " << goal.tilt;
  runAction<actionlib::SimpleActionClient<PanTiltMoveJointsAction>,
            PanTiltMoveJointsGoal,
            PanTiltMoveJointsResultConstPtr,
            PanTiltMoveJointsFeedbackConstPtr>
    (Op_PanTilt, m_panTiltClient, goal, id,
     default_action_active_cb (Op_PanTilt, args.str()),
     default_action_feedback_cb<PanTiltMoveJointsFeedbackConstPtr> (Op_PanTilt),
     default_action_done_cb<PanTiltMoveJointsResultConstPtr> (Op_PanTilt));
}

void OwInterface::panTiltCartesian (int frame, double x, double y, double z, int id)
{
  if (! markOperationRunning (Op_PanTiltCartesian, id)) return;
  thread action_thread (&OwInterface::panTiltCartesianAction,
                        this, frame, x, y, z, id);
  action_thread.detach();
}

void OwInterface::panTiltCartesianAction (int frame,
                                          double x, double y, double z,
                                          int id)
{
  geometry_msgs::Point p;
  p.x = x; p.y = y; p.z = z;

  PanTiltMoveCartesianGoal goal;
  goal.frame = frame;
  goal.point = p;
  std::stringstream args;
  args << goal.frame << ", " << goal.point;
  runAction<actionlib::SimpleActionClient<PanTiltMoveCartesianAction>,
            PanTiltMoveCartesianGoal,
            PanTiltMoveCartesianResultConstPtr,
            PanTiltMoveCartesianFeedbackConstPtr>
    (Op_PanTiltCartesian, m_panTiltCartesianClient, goal, id,
     default_action_active_cb (Op_PanTiltCartesian, args.str()),
     default_action_feedback_cb<PanTiltMoveCartesianFeedbackConstPtr>
     (Op_PanTiltCartesian),
     default_action_done_cb<PanTiltMoveCartesianResultConstPtr>
     (Op_PanTiltCartesian));
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

void OwInterface::dockIngestSample (int id)
{
  if (! markOperationRunning (Op_Ingest, id)) return;
  thread action_thread (&OwInterface::dockIngestSampleAction, this, id);
  action_thread.detach();
}

void OwInterface::dockIngestSampleAction (int id)
{
  DockIngestSampleGoal goal;

  ROS_INFO ("Starting DockIngestSample()");

  runAction<actionlib::SimpleActionClient<DockIngestSampleAction>,
            DockIngestSampleGoal,
            DockIngestSampleResultConstPtr,
            DockIngestSampleFeedbackConstPtr>
    (Op_Ingest, m_dockIngestSampleClient, goal, id,
     default_action_active_cb (Op_Ingest),
     default_action_feedback_cb<DockIngestSampleFeedbackConstPtr> (Op_Ingest),
     default_action_done_cb<DockIngestSampleResultConstPtr> (Op_Ingest));
}

void OwInterface::taskDeliverSample (int id)
{
  if (! markOperationRunning (Op_TaskDeliverSample, id)) return;
  thread action_thread (&OwInterface::taskDeliverSampleAction, this, id);
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

void OwInterface::discardSample (int frame, bool relative,
                                 double x, double y, double z,
                                 double height, int id)
{
  if (! markOperationRunning (Op_TaskDiscardSample, id)) return;
  thread action_thread (&OwInterface::discardSampleAction, this, frame, relative,
                        x, y, z, height, id);
  action_thread.detach();
}

void OwInterface::discardSampleAction (int frame, bool relative,
                                       double x, double y, double z,
                                       double height, int id)
{
  geometry_msgs::Point p;
  p.x = x; p.y = y; p.z = z;

  TaskDiscardSampleGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.point = p;
  goal.height = height;

  ROS_INFO ("Starting TaskDiscardSample(x=%.2f, y=%.2f, z=%.2f)", x, y, z);

  runAction<actionlib::SimpleActionClient<TaskDiscardSampleAction>,
            TaskDiscardSampleGoal,
            TaskDiscardSampleResultConstPtr,
            TaskDiscardSampleFeedbackConstPtr>
    (Op_TaskDiscardSample, m_discardClient, goal, id,
     default_action_active_cb (Op_TaskDiscardSample),
     default_action_feedback_cb<TaskDiscardSampleFeedbackConstPtr> (Op_TaskDiscardSample),
     default_action_done_cb<TaskDiscardSampleResultConstPtr> (Op_TaskDiscardSample));
}

void OwInterface::scoopLinear (int frame, bool relative,
                               double x, double y, double z,
                               double depth, double length, int id)
{
  if (! markOperationRunning (Op_TaskScoopLinear, id)) return;
  thread action_thread (&OwInterface::scoopLinearAction, this,
                        frame, relative, x, y, z, depth, length, id);
  action_thread.detach();
}


void OwInterface::scoopLinearAction (int frame, bool relative,
                                     double x, double y, double z,
                                     double depth, double length, int id)
{
  geometry_msgs::Point p;
  p.x = x; p.y = y; p.z = z;

  TaskScoopLinearGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.point = p;
  goal.depth = depth;
  goal.length = length;

  ROS_INFO ("Starting TaskScoopLinear(x=%.2f, y=%.2f, z=%.2f, depth=%.2f, length=%.2f)",
            x, y, z, depth, length);

  runAction<actionlib::SimpleActionClient<TaskScoopLinearAction>,
            TaskScoopLinearGoal,
            TaskScoopLinearResultConstPtr,
            TaskScoopLinearFeedbackConstPtr>
    (Op_TaskScoopLinear, m_scoopLinearClient, goal, id,
     default_action_active_cb (Op_TaskScoopLinear),
     default_action_feedback_cb<TaskScoopLinearFeedbackConstPtr> (Op_TaskScoopLinear),
     default_action_done_cb<TaskScoopLinearResultConstPtr> (Op_TaskScoopLinear));
}

void OwInterface::scoopCircular (int frame, bool relative,
                                 double x, double y, double z,
                                 double depth, bool parallel, int id)
{
  if (! markOperationRunning (Op_TaskScoopCircular, id)) return;
  thread action_thread (&OwInterface::scoopCircularAction, this,
                        frame, relative, x, y, z, depth, parallel, id);
  action_thread.detach();
}

void OwInterface::scoopCircularAction (int frame, bool relative,
                                       double x, double y, double z,
                                       double depth, bool parallel, int id)
{
  geometry_msgs::Point p;
  p.x = x; p.y = y; p.z = z;

  TaskScoopCircularGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.point = p;
  goal.depth = depth;
  goal.parallel = parallel;

  ROS_INFO ("Starting TaskScoopCircular(x=%.2f, y=%.2f, z=%.2f, depth=%.2f)",
            x, y, z, depth);

  runAction<actionlib::SimpleActionClient<TaskScoopCircularAction>,
            TaskScoopCircularGoal,
            TaskScoopCircularResultConstPtr,
            TaskScoopCircularFeedbackConstPtr>
    (Op_TaskScoopCircular, m_scoopCircularClient, goal, id,
     default_action_active_cb (Op_TaskScoopCircular),
     default_action_feedback_cb<TaskScoopCircularFeedbackConstPtr> (Op_TaskScoopCircular),
     default_action_done_cb<TaskScoopCircularResultConstPtr> (Op_TaskScoopCircular));
}


void OwInterface::armStop (int id)
{
  if (! markOperationRunning (Op_ArmStop, id)) return;
  thread action_thread (&OwInterface::armStop, this, id);
  action_thread.detach();
}

void OwInterface::armStopAction (int id)
{
  ArmStopGoal goal; // empty/undefined

  runAction<actionlib::SimpleActionClient<ArmStopAction>,
            ArmStopGoal,
            ArmStopResultConstPtr,
            ArmStopFeedbackConstPtr>
    (Op_ArmStop, m_armStopClient, goal, id,
     default_action_active_cb (Op_ArmStop),
     default_action_feedback_cb<ArmStopFeedbackConstPtr> (Op_ArmStop),
     default_action_done_cb<ArmStopResultConstPtr> (Op_ArmStop));
}

void OwInterface::armUnstow (int id)
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
  TaskGrindGoal goal;
  goal.x_start = x;
  goal.y_start = y;
  goal.depth = depth;
  goal.length = length;
  goal.parallel = parallel;
  goal.ground_position = ground_pos;

  ROS_INFO ("Starting TaskGrind"
            "(x=%.2f, y=%.2f, depth=%.2f, length=%.2f, "
            "parallel=%s, ground_pos=%.2f)",
            x, y, depth, length, (parallel ? "true" : "false"), ground_pos);

  runAction<actionlib::SimpleActionClient<TaskGrindAction>,
            TaskGrindGoal,
            TaskGrindResultConstPtr,
            TaskGrindFeedbackConstPtr>
    (Op_Grind, m_grindClient, goal, id,
     default_action_active_cb (Op_Grind),
     default_action_feedback_cb<TaskGrindFeedbackConstPtr> (Op_Grind),
     default_action_done_cb<TaskGrindResultConstPtr> (Op_Grind));
}

void OwInterface::armFindSurface (int frame, bool relative,
                                  double pos_x, double pos_y, double pos_z,
                                  double norm_x, double norm_y, double norm_z,
                                  double distance, double overdrive,
                                  double force_threshold, double torque_threshold,
                                  int id)
{
  if (! markOperationRunning (Op_ArmFindSurface, id)) return;

  geometry_msgs::Point pos;
  pos.x = pos_x;
  pos.y = pos_y;
  pos.z = pos_z;

  geometry_msgs::Vector3 normal;
  normal.x = norm_x;
  normal.y = norm_y;
  normal.z = norm_z;

  thread action_thread (&OwInterface::armFindSurfaceAction, this,
			frame, relative, pos, normal, distance, overdrive,
                        force_threshold, torque_threshold,
                        id);
  action_thread.detach();
}

void OwInterface::armFindSurfaceAction (int frame, bool relative,
                                        const geometry_msgs::Point& pos,
                                        const geometry_msgs::Vector3& normal,
                                        double distance, double overdrive,
                                        double force_threshold, double torque_threshold,
                                        int id)
{
  ArmFindSurfaceGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.position = pos;
  goal.normal = normal;
  goal.distance = distance;
  goal.overdrive = overdrive;
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;

  // Fill this out.
  ROS_INFO ("Starting ArmFindSurface (frame=%d, relative=%d)", frame, relative);

  runAction<actionlib::SimpleActionClient<ArmFindSurfaceAction>,
            ArmFindSurfaceGoal,
            ArmFindSurfaceResultConstPtr,
            ArmFindSurfaceFeedbackConstPtr>
	    (Op_ArmFindSurface, m_armFindSurfaceClient, goal, id,
	     default_action_active_cb (Op_ArmFindSurface),
	     default_action_feedback_cb<ArmFindSurfaceFeedbackConstPtr>
	     (Op_ArmFindSurface),
	     default_action_done_cb<ArmFindSurfaceResultConstPtr>
	     (Op_ArmFindSurface));
}

void OwInterface::armMoveCartesian (int frame, bool relative,
				    double x, double y, double z,
				    double orient_x, double orient_y,
				    double orient_z, int id)
{
  tf2::Quaternion q;
  q.setEuler (z,y,x);  // Yaw, pitch, roll, respectively.
  q.normalize();  // Recommended in ROS docs, not sure if needed here.

  geometry_msgs::Quaternion qm = tf2::toMsg(q);

  armMoveCartesianAux (frame, relative, x, y, z, qm, id);
}


void OwInterface::armMoveCartesian (int frame, bool relative,
				    double x, double y, double z,
				    double orient_x, double orient_y,
				    double orient_z, double orient_w, int id)
{
  geometry_msgs::Quaternion q;
  q.x = orient_x;
  q.y = orient_y;
  q.z = orient_z;
  q.w = orient_w;

  armMoveCartesianAux (frame, relative, x, y, z, q, id);
}

void OwInterface::armMoveCartesianGuarded (int frame, bool relative,
                                           double x, double y, double z,
                                           double orient_x,
                                           double orient_y,
                                           double orient_z,
                                           double force_threshold,
                                           double torque_threshold,
                                           int id)
{
  tf2::Quaternion q;
  q.setEuler (z,y,x);  // Yaw, pitch, roll, respectively.
  q.normalize();  // Recommended in ROS docs, not sure if needed here.

  geometry_msgs::Quaternion qm = tf2::toMsg(q);

  armMoveCartesianGuardedAux (frame, relative, x, y, z, qm,
                              force_threshold, torque_threshold, id);
}

void OwInterface::armMoveCartesianGuarded (int frame, bool relative,
                                           double x, double y, double z,
                                           double orient_x, double orient_y,
                                           double orient_z, double orient_w,
                                           double force_threshold,
                                           double torque_threshold,
                                           int id)
{
  geometry_msgs::Quaternion q;
  q.x = orient_x;
  q.y = orient_y;
  q.z = orient_z;
  q.w = orient_w;

  armMoveCartesianGuardedAux (frame, relative, x, y, z, q,
                              force_threshold, torque_threshold, id);
}

void OwInterface::armMoveCartesianGuardedAux (int frame, bool relative,
                                              double x, double y, double z,
                                              const geometry_msgs::Quaternion& q,
                                              double force_threshold,
                                              double torque_threshold,
                                              int id)
{
  if (! markOperationRunning (Op_ArmMoveCartesianGuarded, id)) return;

  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;

  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;

  thread action_thread (&OwInterface::armMoveCartesianGuardedAction, this,
			frame, relative, pose, force_threshold, torque_threshold,
                        id);
  action_thread.detach();
}


void OwInterface::armMoveCartesianAux (int frame, bool relative,
                                       double x, double y, double z,
                                       const geometry_msgs::Quaternion& q, int id)
{
  if (! markOperationRunning (Op_ArmMoveCartesian, id)) return;

  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;

  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;

  thread action_thread (&OwInterface::armMoveCartesianAction, this,
			frame, relative, pose, id);
  action_thread.detach();
}



void OwInterface::armMoveCartesianAction (int frame, bool relative,
                                          const geometry_msgs::Pose& pose, int id)
{
  ArmMoveCartesianGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.pose = pose;

  // Fill this out.
  ROS_INFO ("Starting ArmMoveCartesian (frame=%d, relative=%d)", frame, relative);

  runAction<actionlib::SimpleActionClient<ArmMoveCartesianAction>,
            ArmMoveCartesianGoal,
            ArmMoveCartesianResultConstPtr,
            ArmMoveCartesianFeedbackConstPtr>
	    (Op_ArmMoveCartesian, m_armMoveCartesianClient, goal, id,
	     default_action_active_cb (Op_ArmMoveCartesian),
	     default_action_feedback_cb<ArmMoveCartesianFeedbackConstPtr>
	     (Op_ArmMoveCartesian),
	     default_action_done_cb<ArmMoveCartesianResultConstPtr>
	     (Op_ArmMoveCartesian));
}

void OwInterface::armMoveCartesianGuardedAction (int frame, bool relative,
                                                 const geometry_msgs::Pose& pose,
                                                 double force_threshold,
                                                 double torque_threshold,
                                                 int id)
{
  ArmMoveCartesianGuardedGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.pose = pose;
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;

  // Fill this out.
  ROS_INFO ("Starting ArmMoveCartesianGuarded (frame=%d, relative=%d)",
            frame, relative);

  runAction<actionlib::SimpleActionClient<ArmMoveCartesianGuardedAction>,
            ArmMoveCartesianGuardedGoal,
            ArmMoveCartesianGuardedResultConstPtr,
            ArmMoveCartesianGuardedFeedbackConstPtr>
	    (Op_ArmMoveCartesianGuarded, m_armMoveCartesianGuardedClient, goal, id,
	     default_action_active_cb (Op_ArmMoveCartesianGuarded),
	     default_action_feedback_cb<ArmMoveCartesianGuardedFeedbackConstPtr>
	     (Op_ArmMoveCartesianGuarded),
	     default_action_done_cb<ArmMoveCartesianGuardedResultConstPtr>
	     (Op_ArmMoveCartesianGuarded));
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

  ROS_INFO ("Starting ArmMoveJoint (relative=%d, joint=%d, angle=%f)",
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

void OwInterface::armMoveJointsGuarded (bool relative,
                                        const vector<double>& angles,
                                        double force_threshold,
                                        double torque_threshold,
                                        int id)
{
  if (! markOperationRunning (Op_ArmMoveJointsGuarded, id)) return;
  thread action_thread (&OwInterface::armMoveJointsGuardedAction,
                        this, relative, angles,
                        force_threshold, torque_threshold, id);
  action_thread.detach();
}

void OwInterface::armMoveJointsGuardedAction (bool relative,
                                              const vector<double>& angles,
                                              double force_threshold,
                                              double torque_threshold,
                                              int id)
{
  ArmMoveJointsGuardedGoal goal;
  goal.relative = relative;
  std::copy(angles.begin(), angles.end(), back_inserter(goal.angles));
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;

  ROS_INFO ("Starting ArmMoveJointsGuarded"
            "(relative=%d, angles=[%f, %f, %f, %f, %f, %f], force_threshold=%f, "
            "torque_threshold=%f)",
            goal.relative,
            goal.angles[0], goal.angles[1], goal.angles[2],
            goal.angles[3], goal.angles[4], goal.angles[5],
            force_threshold, torque_threshold);

  runAction<actionlib::SimpleActionClient<ArmMoveJointsGuardedAction>,
            ArmMoveJointsGuardedGoal,
            ArmMoveJointsGuardedResultConstPtr,
            ArmMoveJointsGuardedFeedbackConstPtr>
    (Op_ArmMoveJointsGuarded, m_armMoveJointsGuardedClient, goal, id,
     default_action_active_cb (Op_ArmMoveJointsGuarded),
     default_action_feedback_cb<ArmMoveJointsGuardedFeedbackConstPtr>
     (Op_ArmMoveJointsGuarded),
     default_action_done_cb<ArmMoveJointsGuardedResultConstPtr>
     (Op_ArmMoveJointsGuarded));
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

double OwInterface::getBatteryStateOfCharge () const
{
  return SOC;
}

double OwInterface::getBatteryRemainingUsefulLife () const
{
  return RUL;
}

double OwInterface::getBatteryTemperature () const
{
  return BatteryTemp;
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

vector<double> OwInterface::getEndEffectorFT () const
{
  return m_endEffectorFT;
}

vector<double> OwInterface::getArmPose () const
{
  return m_armPose;
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
