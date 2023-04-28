// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ow_plexil
#include "OwInterface.h"
#include "subscriber.h"

// owl_msgs
#include <owl_msgs/BatteryRemainingUsefulLife.h>
#include <owl_msgs/BatteryStateOfCharge.h>
#include <owl_msgs/BatteryTemperature.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

// C++
#include <thread>
#include <vector>
#include <functional>
#include <set>
#include <algorithm> // for std::copy
#include <inttypes.h> // for int64 support

using namespace ow_lander;
using namespace owl_msgs;
using namespace PLEXIL;

using std::set;
using std::vector;
using std::thread;
using std::ref;
using std::string;
using std::shared_ptr;
using std::make_unique;


//////////////////// Lander Operation Support ////////////////////////

// Index into /joint_states message and JointTelemetries vector.
const size_t ArmJointStartIndex = 2;

const double PointCloudTimeout = 50.0; // 5 second timeout assuming a rate of 10hz
const double SampleTimeout = 50.0; // 5 second timeout assuming a rate of 10hz

// Lander operation names.  In general these match those used in PLEXIL and
// owl_msgs.

const string Name_ArmFindSurface         = "ArmFindSurface";
const string Name_ArmMoveJoints          = "ArmMoveJoints";
const string Name_ArmMoveJointsGuarded   = "ArmMoveJointsGuarded";
const string Name_CameraSetExposure      = "CameraSetExposure";
const string Name_Ingest                 = "DockIngestSample";
const string Name_Grind                  = "TaskGrind";
const string Name_GuardedMove            = "GuardedMove";
const string Name_IdentifySampleLocation = "IdentifySampleLocation";
const string Name_LightSetIntensity      = "LightSetIntensity";
const string Name_Pan                    = "Pan";
const string Name_PanTiltCartesian       = "PanTiltMoveCartesian";
const string Name_TaskScoopCircular      = "TaskScoopCircular";
const string Name_TaskScoopLinear        = "TaskScoopLinear";
const string Name_Tilt                   = "Tilt";
const string Name_TaskDiscardSample      = "TaskDiscardSample";

static vector<string> LanderOpNames = {
  Name_ArmFindSurface,
  Name_GuardedMove,
  Name_ArmMoveJoints,
  Name_ArmMoveJointsGuarded,
  Name_Pan,
  Name_Tilt,
  Name_PanTiltCartesian,
  Name_TaskScoopCircular,
  Name_TaskScoopLinear,
  Name_Grind,
  Name_CameraSetExposure,
  Name_Ingest,
  Name_IdentifySampleLocation,
  Name_TaskDiscardSample,
  Name_LightSetIntensity
};


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
    LanderInterface::initialize();

    for (const string& name : LanderOpNames) {
      registerLanderOperation (name);
    }

    // Initialize action clients

    m_armFindSurfaceClient =
      make_unique<ArmFindSurfaceActionClient>(Name_ArmFindSurface, true);
    m_guardedMoveClient =
      make_unique<GuardedMoveActionClient>(Name_GuardedMove, true);
    m_armMoveJointsClient =
      make_unique<ArmMoveJointsActionClient>(Name_ArmMoveJoints, true);
    m_armMoveJointsGuardedClient =
      make_unique<ArmMoveJointsGuardedActionClient>(Name_ArmMoveJointsGuarded,
                                                    true);
    m_grindClient = make_unique<TaskGrindActionClient>(Name_Grind, true);
    m_scoopCircularClient =
      make_unique<TaskScoopCircularActionClient>(Name_TaskScoopCircular, true);
    m_scoopLinearClient =
      make_unique<TaskScoopLinearActionClient>(Name_TaskScoopLinear, true);
    m_cameraSetExposureClient =
      make_unique<CameraSetExposureActionClient>(Name_CameraSetExposure, true);
    m_dockIngestSampleClient =
      make_unique<DockIngestSampleActionClient>(Name_Ingest, true);
    m_lightSetIntensityClient =
      make_unique<LightSetIntensityActionClient>(Name_LightSetIntensity, true);
    m_identifySampleLocationClient =
      make_unique<IdentifySampleLocationActionClient>
      (Name_IdentifySampleLocation, true);
    m_panClient = make_unique<PanActionClient>(Name_Pan, true);
    m_tiltClient = make_unique<TiltActionClient>(Name_Tilt, true);
    m_panTiltCartesianClient =
      make_unique<PanTiltMoveCartesianActionClient>(Name_PanTiltCartesian, true);
    m_taskDiscardSampleClient =
      make_unique<TaskDiscardSampleActionClient>(Name_TaskDiscardSample, true);

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

    connectActionServer (m_armFindSurfaceClient, Name_ArmFindSurface,
                         "/ArmFindSurface/status");
    connectActionServer (m_armMoveJointsClient, Name_ArmMoveJoints,
                         "/ArmMoveJoints/status");
    connectActionServer (m_armMoveJointsGuardedClient, Name_ArmMoveJointsGuarded,
                         "/ArmMoveJointsGuarded/status");
    connectActionServer (m_scoopCircularClient, Name_TaskScoopCircular,
                         "/TaskScoopCircular/status");
    connectActionServer (m_scoopLinearClient, Name_TaskScoopLinear,
                         "/TaskScoopLinear/status");
    connectActionServer (m_cameraSetExposureClient, Name_CameraSetExposure,
                         "/CameraSetExposure/status");
    connectActionServer (m_dockIngestSampleClient, Name_Ingest,
                         "/DockIngestSample/status");
    connectActionServer (m_lightSetIntensityClient, Name_LightSetIntensity,
                         "/LightSetIntensity/status");
    connectActionServer (m_guardedMoveClient, Name_GuardedMove,
                         "/GuardedMove/status");
    connectActionServer (m_panClient, Name_Pan);
    connectActionServer (m_tiltClient, Name_Tilt);
    connectActionServer (m_panTiltCartesianClient, Name_PanTiltCartesian);
    connectActionServer (m_identifySampleLocationClient, Name_IdentifySampleLocation);
    connectActionServer (m_taskDiscardSampleClient, Name_TaskDiscardSample,
                         "/TaskDiscardSample/status");

  }
}

void OwInterface::pan (double degrees, int id)
{
  if (! markOperationRunning (Name_Pan, id)) return;
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
    (Name_Pan, m_panClient, goal, id,
     default_action_active_cb (Name_Pan, args.str()),
     default_action_feedback_cb<PanFeedbackConstPtr> (Name_Pan),
     default_action_done_cb<PanResultConstPtr> (Name_Pan));
}

void OwInterface::tilt (double degrees, int id)
{
  if (! markOperationRunning (Name_Tilt, id)) return;
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
    (Name_Tilt, m_tiltClient, goal, id,
     default_action_active_cb (Name_Tilt, args.str()),
     default_action_feedback_cb<TiltFeedbackConstPtr> (Name_Tilt),
     default_action_done_cb<TiltResultConstPtr> (Name_Tilt));
}

void OwInterface::panTiltCartesian (int frame, double x, double y, double z, int id)
{
  if (! markOperationRunning (Name_PanTiltCartesian, id)) return;
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
    (Name_PanTiltCartesian, m_panTiltCartesianClient, goal, id,
     default_action_active_cb (Name_PanTiltCartesian, args.str()),
     default_action_feedback_cb<PanTiltMoveCartesianFeedbackConstPtr>
     (Name_PanTiltCartesian),
     default_action_done_cb<PanTiltMoveCartesianResultConstPtr>
     (Name_PanTiltCartesian));
}

void OwInterface::cameraSetExposure (double exposure_secs, int id)
{
  if (! markOperationRunning (Name_CameraSetExposure, id)) return;
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
    (Name_CameraSetExposure, m_cameraSetExposureClient, goal, id,
     default_action_active_cb (Name_CameraSetExposure),
     default_action_feedback_cb<CameraSetExposureFeedbackConstPtr> (Name_CameraSetExposure),
     default_action_done_cb<CameraSetExposureResultConstPtr> (Name_CameraSetExposure));
}

void OwInterface::dockIngestSample (int id)
{
  if (! markOperationRunning (Name_Ingest, id)) return;
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
    (Name_Ingest, m_dockIngestSampleClient, goal, id,
     default_action_active_cb (Name_Ingest),
     default_action_feedback_cb<DockIngestSampleFeedbackConstPtr> (Name_Ingest),
     default_action_done_cb<DockIngestSampleResultConstPtr> (Name_Ingest));
}

void OwInterface::scoopLinear (int frame, bool relative,
                               double x, double y, double z,
                               double depth, double length, int id)
{
  if (! markOperationRunning (Name_TaskScoopLinear, id)) return;
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
    (Name_TaskScoopLinear, m_scoopLinearClient, goal, id,
     default_action_active_cb (Name_TaskScoopLinear),
     default_action_feedback_cb<TaskScoopLinearFeedbackConstPtr> (Name_TaskScoopLinear),
     default_action_done_cb<TaskScoopLinearResultConstPtr> (Name_TaskScoopLinear));
}

void OwInterface::scoopCircular (int frame, bool relative,
                                 double x, double y, double z,
                                 double depth, bool parallel, int id)
{
  if (! markOperationRunning (Name_TaskScoopCircular, id)) return;
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
    (Name_TaskScoopCircular, m_scoopCircularClient, goal, id,
     default_action_active_cb (Name_TaskScoopCircular),
     default_action_feedback_cb<TaskScoopCircularFeedbackConstPtr> (Name_TaskScoopCircular),
     default_action_done_cb<TaskScoopCircularResultConstPtr> (Name_TaskScoopCircular));
}


void OwInterface::grind (double x, double y, double depth, double length,
                         bool parallel, double ground_pos, int id)
{
  if (! markOperationRunning (Name_Grind, id)) return;
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
    (Name_Grind, m_grindClient, goal, id,
     default_action_active_cb (Name_Grind),
     default_action_feedback_cb<TaskGrindFeedbackConstPtr> (Name_Grind),
     default_action_done_cb<TaskGrindResultConstPtr> (Name_Grind));
}

void OwInterface::armFindSurface (int frame,
                                  bool relative,
                                  const std::vector<double>& position,
                                  const std::vector<double>& normal,
                                  double distance,
                                  double overdrive,
                                  double force_threshold,
                                  double torque_threshold,
                                  int id)
{
  if (! markOperationRunning (Name_ArmFindSurface, id)) return;

  geometry_msgs::Point pos;
  pos.x = position[0];
  pos.y = position[1];
  pos.z = position[2];

  geometry_msgs::Vector3 norm;
  norm.x = normal[0];
  norm.y = normal[1];
  norm.z = normal[2];

  thread action_thread (&OwInterface::armFindSurfaceAction, this,
			frame, relative, pos, norm, distance, overdrive,
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
	    (Name_ArmFindSurface, m_armFindSurfaceClient, goal, id,
	     default_action_active_cb (Name_ArmFindSurface),
	     default_action_feedback_cb<ArmFindSurfaceFeedbackConstPtr>
	     (Name_ArmFindSurface),
	     default_action_done_cb<ArmFindSurfaceResultConstPtr>
	     (Name_ArmFindSurface));
}

void OwInterface::guardedMove (double x, double y, double z,
                               double dir_x, double dir_y, double dir_z,
                               double search_dist, int id)
{
  if (! markOperationRunning (Name_GuardedMove, id)) return;
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
    (Name_GuardedMove, m_guardedMoveClient, goal, id,
     default_action_active_cb (Name_GuardedMove),
     default_action_feedback_cb<GuardedMoveFeedbackConstPtr> (Name_GuardedMove),
     guarded_move_done_cb<GuardedMoveResultConstPtr> (Name_GuardedMove));
}

void OwInterface::armMoveJoints (bool relative,
                                 const vector<double>& angles,
                                 int id)
{
  if (! markOperationRunning (Name_ArmMoveJoints, id)) return;
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
    (Name_ArmMoveJoints, m_armMoveJointsClient, goal, id,
     default_action_active_cb (Name_ArmMoveJoints),
     default_action_feedback_cb<ArmMoveJointsFeedbackConstPtr> (Name_ArmMoveJoints),
     default_action_done_cb<ArmMoveJointsResultConstPtr> (Name_ArmMoveJoints));
}

void OwInterface::armMoveJointsGuarded (bool relative,
                                        const vector<double>& angles,
                                        double force_threshold,
                                        double torque_threshold,
                                        int id)
{
  if (! markOperationRunning (Name_ArmMoveJointsGuarded, id)) return;
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
    (Name_ArmMoveJointsGuarded, m_armMoveJointsGuardedClient, goal, id,
     default_action_active_cb (Name_ArmMoveJointsGuarded),
     default_action_feedback_cb<ArmMoveJointsGuardedFeedbackConstPtr>
     (Name_ArmMoveJointsGuarded),
     default_action_done_cb<ArmMoveJointsGuardedResultConstPtr>
     (Name_ArmMoveJointsGuarded));
}


vector<double> OwInterface::identifySampleLocation (int num_images,
                                                    const string& filter_type,
                                                    int id)
{
  SamplePoint.clear();
  GotSampleLocation = false;
  if (! markOperationRunning (Name_IdentifySampleLocation, id)) return SamplePoint;
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
    (Name_IdentifySampleLocation, m_identifySampleLocationClient, goal, id,
     default_action_active_cb (Name_IdentifySampleLocation),
     default_action_feedback_cb<ow_plexil::IdentifyLocationFeedbackConstPtr>
     (Name_IdentifySampleLocation),
     identify_sample_location_done_cb<ow_plexil::IdentifyLocationResultConstPtr>);
}


void OwInterface::lightSetIntensity (const string& side, double intensity,
                                     int id)
{
  if (! markOperationRunning (Name_LightSetIntensity, id)) return;
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
    (Name_LightSetIntensity, m_lightSetIntensityClient, goal, id,
     default_action_active_cb (Name_LightSetIntensity),
     default_action_feedback_cb<LightSetIntensityFeedbackConstPtr>
     (Name_LightSetIntensity),
     default_action_done_cb<LightSetIntensityResultConstPtr>
     (Name_LightSetIntensity));
}

void OwInterface::taskDiscardSample (int frame, bool relative,
                                     const std::vector<double>& point,
                                     double height, int id)
{
  if (! markOperationRunning (Name_TaskDiscardSample, id)) return;
  thread action_thread (&OwInterface::taskDiscardSampleAction, this,
                        frame, relative, point, height, id);
  action_thread.detach();
}

void OwInterface::taskDiscardSampleAction (int frame, bool relative,
                                           const std::vector<double>& point,
                                           double height, int id)
{
  geometry_msgs::Point p;
  p.x = point[0]; p.y = point[1]; p.z = point[2];

  TaskDiscardSampleGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.point = p;
  goal.height = height;

  ROS_INFO ("Starting TaskDiscardSample(x=%.2f, y=%.2f, z=%.2f)", p.x, p.y, p.z);

  runAction<actionlib::SimpleActionClient<TaskDiscardSampleAction>,
            TaskDiscardSampleGoal,
            TaskDiscardSampleResultConstPtr,
            TaskDiscardSampleFeedbackConstPtr>
    (Name_TaskDiscardSample, m_taskDiscardSampleClient, goal, id,
     default_action_active_cb (Name_TaskDiscardSample),
     default_action_feedback_cb<TaskDiscardSampleFeedbackConstPtr> (Name_TaskDiscardSample),
     default_action_done_cb<TaskDiscardSampleResultConstPtr> (Name_TaskDiscardSample));
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
