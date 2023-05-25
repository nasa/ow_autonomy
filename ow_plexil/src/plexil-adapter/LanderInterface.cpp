// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

// ow_plexil
#include "LanderInterface.h"
#include "subscriber.h"

// PLEXIL
#include <ArrayImpl.hh>

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++
#include <thread>
#include <vector>
#include <functional>

using namespace owl_msgs;
using namespace PLEXIL;

using std::hash;
using std::copy;
using std::string;
using std::thread;
using std::string;
using std::vector;
using std::make_unique;

const string Name_ArmMoveCartesian        = "ArmMoveCartesian";
const string Name_ArmMoveCartesianGuarded = "ArmMoveCartesianGuarded";
const string Name_ArmMoveJoint            = "ArmMoveJoint";
const string Name_ArmStop                 = "ArmStop";
const string Name_ArmStow                 = "ArmStow";
const string Name_ArmUnstow               = "ArmUnstow";
const string Name_CameraCapture           = "CameraCapture";
const string Name_PanTiltMoveJoints       = "PanTiltMoveJoints";
const string Name_TaskDeliverSample       = "TaskDeliverSample";

static vector<string> LanderOpNames = {
  Name_ArmMoveCartesian,
  Name_ArmMoveCartesianGuarded,
  Name_ArmMoveJoint,
  Name_ArmStop,
  Name_ArmStow,
  Name_ArmUnstow,
  Name_CameraCapture,
  Name_PanTiltMoveJoints,
  Name_TaskDeliverSample
};

void LanderInterface::initialize()
{
  PlexilInterface::initialize();

  m_current_pan_radians = NAN;
  m_current_tilt_radians = NAN;
  m_battery_soc = NAN;
  m_battery_rul = NAN;
  m_battery_temp = NAN;

  m_arm_joint_accelerations.resize(NumArmJoints);
  m_arm_joint_positions.resize(NumArmJoints);
  m_arm_joint_velocities.resize(NumArmJoints);
  m_arm_joint_torques.resize(NumArmJoints);
  m_arm_pose.resize(7);
  m_arm_pose = {0,0,0,0,0,0,0};

  for (auto name : LanderOpNames) {
    registerLanderOperation (name);
  }

  // Initialize action clients

  m_armMoveCartesianClient =
    make_unique<ArmMoveCartesianActionClient>(Name_ArmMoveCartesian, true);
  m_armMoveCartesianGuardedClient =
    make_unique<ArmMoveCartesianGuardedActionClient>
    (Name_ArmMoveCartesianGuarded, true);
  m_armMoveJointClient =
    make_unique<ArmMoveJointActionClient>(Name_ArmMoveJoint, true);
  m_armStopClient =
    make_unique<ArmStopActionClient>(Name_ArmStop, true);
  m_armStowClient =
    make_unique<ArmStowActionClient>(Name_ArmStow, true);
  m_armUnstowClient =
    make_unique<ArmUnstowActionClient>(Name_ArmUnstow, true);
  m_cameraCaptureClient =
    make_unique<CameraCaptureActionClient>(Name_CameraCapture, true);
  m_panTiltMoveJointsClient =
    make_unique<PanTiltMoveJointsActionClient>(Name_PanTiltMoveJoints, true);
  m_taskDeliverSampleClient =
    make_unique<TaskDeliverSampleActionClient>(Name_TaskDeliverSample, true);

  // Connect to action servers

  connectActionServer (m_armMoveCartesianClient, Name_ArmMoveCartesian,
                       "/ArmMoveCartesian/status");
  connectActionServer (m_armMoveCartesianGuardedClient, Name_ArmMoveCartesianGuarded,
                       "/ArmMoveCartesianGuarded/status");
  connectActionServer (m_armMoveJointClient, Name_ArmMoveJoint,
                       "/ArmMoveJoint/status");
  connectActionServer (m_armStopClient, Name_ArmStop, "/ArmStop/status");
  connectActionServer (m_armStowClient, Name_ArmStow, "/ArmStow/status");
  connectActionServer (m_armUnstowClient, Name_ArmUnstow, "/ArmUnstow/status");
  connectActionServer (m_taskDeliverSampleClient, Name_TaskDeliverSample,
                       "/TaskDeliverSample/status");
  connectActionServer (m_panTiltMoveJointsClient, Name_PanTiltMoveJoints);
  connectActionServer (m_cameraCaptureClient, Name_CameraCapture,
                       "/CameraCapture/status");

  // Initialize subscribers

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/arm_faults_status", QueueSize,
                &LanderInterface::armFaultCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/power_faults_status", QueueSize,
                &LanderInterface::powerFaultCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/pan_tilt_faults_status", QueueSize,
                &LanderInterface::antennaFaultCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/camera_faults_status", QueueSize,
                &LanderInterface::cameraFaultCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/arm_joint_accelerations", QueueSize,
                &LanderInterface::armJointAccelCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/arm_joint_positions", QueueSize,
                &LanderInterface::armJointPositionCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/arm_joint_torques", QueueSize,
                &LanderInterface::armJointTorqueCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/arm_joint_velocities", QueueSize,
                &LanderInterface::armJointVelocityCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/arm_pose", QueueSize,
                &LanderInterface::armPoseCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/pan_tilt_position", QueueSize,
                &LanderInterface::panTiltCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/battery_state_of_charge", QueueSize,
                &LanderInterface::batteryChargeCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/battery_temperature", QueueSize,
                &LanderInterface::batteryTempCb, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/battery_remaining_useful_life", QueueSize,
                &LanderInterface::batteryLifeCb, this)));
}

///////////////////////// Subscriber Callbacks ///////////////////////////////

void LanderInterface::batteryChargeCb
(const owl_msgs::BatteryStateOfCharge::ConstPtr& msg)
{
  m_battery_soc = msg->value;
  publish ("BatteryStateOfCharge", m_battery_soc);
}

void LanderInterface::batteryLifeCb
(const owl_msgs::BatteryRemainingUsefulLife::ConstPtr& msg)
{
  // NOTE: This is not being called as of 4/12/21.  Jira OW-656 addresses.
  m_battery_rul = msg->value;
  publish ("BatteryRemainingUsefulLife", m_battery_rul);
}

void LanderInterface::batteryTempCb
(const owl_msgs::BatteryTemperature::ConstPtr& msg)
{
  m_battery_temp = msg->value;
  publish ("BatteryTemperature", m_battery_temp);
}

void LanderInterface::armJointAccelCb
(const owl_msgs::ArmJointAccelerations::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_joint_accelerations.begin());
}

void LanderInterface::armJointPositionCb
(const owl_msgs::ArmJointPositions::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_joint_positions.begin());
}

void LanderInterface::armJointVelocityCb
(const owl_msgs::ArmJointVelocities::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_joint_velocities.begin());
}

void LanderInterface::armJointTorqueCb
(const owl_msgs::ArmJointTorques::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_joint_torques.begin());
}

void LanderInterface::panTiltCb (const owl_msgs::PanTiltPosition::ConstPtr& msg)
{
  m_current_pan_radians = msg->value[0];
  m_current_tilt_radians = msg->value[1];

  // Update PLEXIL lookups
  publish ("PanRadians", m_current_pan_radians);
  publish ("PanDegrees", m_current_pan_radians * R2D);
  publish ("TiltRadians", m_current_tilt_radians);
  publish ("TiltDegrees", m_current_tilt_radians * R2D);
}

void LanderInterface::armFaultCb (const owl_msgs::ArmFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_armErrors, "ARM", "ArmFault");
}

void LanderInterface::powerFaultCb
(const owl_msgs::PowerFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_powerErrors, "POWER", "PowerFault");
}

void LanderInterface::antennaFaultCb
(const owl_msgs::PanTiltFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_panTiltErrors, "ANTENNA", "AntennaFault");
}

void LanderInterface::cameraFaultCb
(const owl_msgs::CameraFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_cameraErrors, "CAMERA", "CameraFault");
}

void LanderInterface::armPoseCb (const owl_msgs::ArmPose::ConstPtr& msg)
{
  m_arm_pose[0] = msg->value.position.x;
  m_arm_pose[1] = msg->value.position.y;
  m_arm_pose[2] = msg->value.position.z;
  m_arm_pose[3] = msg->value.orientation.x;
  m_arm_pose[4] = msg->value.orientation.y;
  m_arm_pose[5] = msg->value.orientation.z;
  m_arm_pose[6] = msg->value.orientation.w;
}


////////////////////// Fault support ///////////////////////////////////////

bool LanderInterface::antennaFault () const
{
  return antennaPanFault() || antennaTiltFault();
}

bool LanderInterface::antennaPanFault () const
{
  return m_panTiltErrors.at(FaultPanJointLocked).second;
}

bool LanderInterface::antennaTiltFault () const
{
  return m_panTiltErrors.at(FaultTiltJointLocked).second;
}

bool LanderInterface::armFault () const
{
  return faultActive (m_armErrors);
}

bool LanderInterface::powerFault () const
{
  return faultActive (m_powerErrors);
}

bool LanderInterface::cameraFault () const
{
  return faultActive (m_cameraErrors);
}


/////////////////////////////// Unified Lander Interface ////////////////////////

void LanderInterface::taskDeliverSample (int id)
{
  if (! markOperationRunning (Name_TaskDeliverSample, id)) return;
  thread action_thread (&LanderInterface::runNullaryAction<
                        TaskDeliverSampleActionClient,
                        TaskDeliverSampleGoal,
                        TaskDeliverSampleResultConstPtr,
                        TaskDeliverSampleFeedbackConstPtr>,
                        this, id, Name_TaskDeliverSample,
                        std::ref(m_taskDeliverSampleClient));
  action_thread.detach();
}


void LanderInterface::armMoveCartesian (int frame,
                                       bool relative,
                                       const vector<double>& position,
                                       const vector<double>& orientation,
                                       int id)
{
  if (! markOperationRunning (Name_ArmMoveCartesian, id)) return;

  geometry_msgs::Quaternion qm;

  // Deal with type of orientation
  if (orientation.size() == 3) { // assume Euler angle
    tf2::Quaternion q;
    // Yaw, pitch, roll, respectively.
    q.setEuler (orientation[2], orientation[1], orientation[0]);
    q.normalize();  // Recommended in ROS docs, not sure if needed here.
    qm = tf2::toMsg(q);
  }
  else { // assume quaternion orientation
    qm.x = orientation[0];
    qm.y = orientation[1];
    qm.z = orientation[2];
    qm.w = orientation[3];
  }

  geometry_msgs::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation = qm;
  thread action_thread (&LanderInterface::armMoveCartesianAction, this,
                        frame, relative, pose, id);
  action_thread.detach();
}

void LanderInterface::armMoveCartesianAction (int frame,
                                             bool relative,
                                             const geometry_msgs::Pose& pose,
                                             int id)
{
  ArmMoveCartesianGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.pose = pose;
  string opname = Name_ArmMoveCartesian;

  runAction<actionlib::SimpleActionClient<ArmMoveCartesianAction>,
            ArmMoveCartesianGoal,
            ArmMoveCartesianResultConstPtr,
            ArmMoveCartesianFeedbackConstPtr>
    (opname, m_armMoveCartesianClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveCartesianFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveCartesianResultConstPtr> (opname));
}

void LanderInterface::armMoveCartesianGuarded (int frame, bool relative,
                                              const vector<double>& position,
                                              const vector<double>& orientation,
                                              double force_threshold,
                                              double torque_threshold,int id)
{
  if (! markOperationRunning (Name_ArmMoveCartesianGuarded, id)) return;

  geometry_msgs::Quaternion qm;

  // Deal with type of orientation
  if (orientation.size() == 3) { // assume Euler angle
    tf2::Quaternion q;
    // Yaw, pitch, roll, respectively.
    q.setEuler (orientation[2], orientation[1], orientation[0]);
    q.normalize();  // Recommended in ROS docs, not sure if needed here.
    qm = tf2::toMsg(q);
  }
  else { // assume quaternion orientation
    qm.x = orientation[0];
    qm.y = orientation[1];
    qm.z = orientation[2];
    qm.w = orientation[3];
  }

  geometry_msgs::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation = qm;
  thread action_thread (&LanderInterface::armMoveCartesianGuardedAction,
                        this, frame, relative, pose,
                        force_threshold, torque_threshold, id);
  action_thread.detach();
}

void LanderInterface::armMoveCartesianGuardedAction (int frame, bool relative,
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
  string opname = Name_ArmMoveCartesianGuarded;

  runAction<actionlib::SimpleActionClient<ArmMoveCartesianGuardedAction>,
            ArmMoveCartesianGuardedGoal,
            ArmMoveCartesianGuardedResultConstPtr,
            ArmMoveCartesianGuardedFeedbackConstPtr>
    (opname, m_armMoveCartesianGuardedClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveCartesianGuardedFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveCartesianGuardedResultConstPtr> (opname));
}

void LanderInterface::armMoveJoint (bool relative,
                                   int joint, double angle,
                                   int id)
{
  if (! markOperationRunning (Name_ArmMoveJoint, id)) return;
  thread action_thread (&LanderInterface::armMoveJointAction,
                        this, relative, joint, angle, id);
  action_thread.detach();
}

void LanderInterface::armMoveJointAction (bool relative,
                                         int joint, double angle,
                                         int id)
{
  ArmMoveJointGoal goal;
  goal.relative = relative;
  goal.joint = joint;
  goal.angle = angle;
  string opname = Name_ArmMoveJoint;  // shorter version

  ROS_INFO ("Starting ArmMoveJoint (relative=%d, joint=%d, angle=%f)",
            goal.relative, goal.joint, goal.angle);

  runAction<actionlib::SimpleActionClient<ArmMoveJointAction>,
            ArmMoveJointGoal,
            ArmMoveJointResultConstPtr,
            ArmMoveJointFeedbackConstPtr>
    (opname, m_armMoveJointClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveJointFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveJointResultConstPtr> (opname));
}

void LanderInterface::armStop (int id)
{
  if (! markOperationRunning (Name_ArmStop, id)) return;
  thread action_thread (&LanderInterface::runNullaryAction<
                        ArmStopActionClient,
                        ArmStopGoal,
                        ArmStopResultConstPtr,
                        ArmStopFeedbackConstPtr>,
                        this, id, Name_ArmStop, std::ref(m_armStopClient));
  action_thread.detach();
}

void LanderInterface::armStow (int id)
{
  if (! markOperationRunning (Name_ArmStow, id)) return;
  //  thread action_thread (&LanderInterface::armStowAction, this, id);
  thread action_thread (&LanderInterface::runNullaryAction<
                        ArmStowActionClient,
                        ArmStowGoal,
                        ArmStowResultConstPtr,
                        ArmStowFeedbackConstPtr>,
                        this, id, Name_ArmStow, std::ref(m_armStowClient));
  action_thread.detach();
}

void LanderInterface::armUnstow (int id)
{
  if (! markOperationRunning (Name_ArmUnstow, id)) return;
  thread action_thread (&LanderInterface::runNullaryAction<
                        ArmUnstowActionClient,
                        ArmUnstowGoal,
                        ArmUnstowResultConstPtr,
                        ArmUnstowFeedbackConstPtr>,
                        this, id, Name_ArmUnstow, std::ref(m_armUnstowClient));
  action_thread.detach();
}

void LanderInterface::panTiltMoveJoints (double pan_degrees,
                                         double tilt_degrees,
                                         int id)
{
  if (! markOperationRunning (Name_PanTiltMoveJoints, id)) return;
  thread action_thread (&LanderInterface::panTiltMoveJointsAction, this,
                        pan_degrees, tilt_degrees, id);
  action_thread.detach();
}

void LanderInterface::panTiltMoveJointsAction (double pan_degrees,
                                               double tilt_degrees,
                                               int id)
{
  PanTiltMoveJointsGoal goal;
  goal.pan = pan_degrees * D2R;
  goal.tilt = tilt_degrees * D2R;
  std::stringstream args;
  args << goal.pan << ", " << goal.tilt;
  string opname = Name_PanTiltMoveJoints;
  runAction<actionlib::SimpleActionClient<PanTiltMoveJointsAction>,
            PanTiltMoveJointsGoal,
            PanTiltMoveJointsResultConstPtr,
            PanTiltMoveJointsFeedbackConstPtr>
    (opname, m_panTiltMoveJointsClient, goal, id,
     default_action_active_cb (opname, args.str()),
     default_action_feedback_cb<PanTiltMoveJointsFeedbackConstPtr> (opname),
     default_action_done_cb<PanTiltMoveJointsResultConstPtr> (opname));
}

void LanderInterface::cameraCapture (int id)
{
  if (! markOperationRunning (Name_CameraCapture, id)) return;
  thread action_thread (&LanderInterface::runNullaryAction<
                        CameraCaptureActionClient,
                        CameraCaptureGoal,
                        CameraCaptureResultConstPtr,
                        CameraCaptureFeedbackConstPtr>,
                        this, id, Name_CameraCapture,
                        std::ref(m_cameraCaptureClient));
  action_thread.detach();
}

double LanderInterface::getArmJointAcceleration (int index) const
{
  return (m_arm_joint_accelerations[index]);
}

double LanderInterface::getArmJointVelocity (int index) const
{
  return (m_arm_joint_velocities[index]);
}

double LanderInterface::getArmJointTorque (int index) const
{
  return (m_arm_joint_torques[index]);
}

double LanderInterface::getArmJointPosition (int index) const
{
  return (m_arm_joint_positions[index]);
}
vector<double> LanderInterface::getArmPose () const
{
  return m_arm_pose;
}

double LanderInterface::getTiltRadians () const
{
  return m_current_tilt_radians;
}

double LanderInterface::getPanRadians () const
{
  return m_current_pan_radians;
}

double LanderInterface::getBatterySOC () const
{
  return m_battery_soc;
}

double LanderInterface::getBatteryRUL () const
{
  return m_battery_rul;
}

double LanderInterface::getBatteryTemperature () const
{
  return m_battery_temp;
}
