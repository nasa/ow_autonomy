// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// ow_autonomy
#include "OwInterface.h"
#include "subscriber.h"

// OW - other
#include <ow_lander/lander_joints.h>
#include <ow_lander/StartPlanning.h>
#include <ow_lander/MoveGuarded.h>
#include <ow_lander/PublishTrajectory.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>

// C++
#include <set>
using std::set;

// C
#include <cmath>  // for M_PI and abs

// Degree/Radian
const double D2R = M_PI / 180.0 ;
const double R2D = 180.0 / M_PI ;


//// Lander state cache, simple start for now.  We may want to refactor, move
//// everything into structures, and make part of object.

double CurrentTilt         = 0.0;
double CurrentPanDegrees   = 0.0;
bool   ImageReceived       = false;

static set<string> JointsAtHardTorqueLimit { };
static set<string> JointsAtSoftTorqueLimit { };

// Torque limits: made up for now, and there may be a better place to code or
// extract these.  Assuming that only magnitude matters.

const double TorqueSoftLimits[] = {
  30,   // j_ant_pan
  30,   // j_ant_tilt
  60, // j_dist_pitch
  60, // j_hand_yaw
  60, // j_prox_pitch
  60, // j_scoop_yaw
  60, // j_shou_pitch
  60  // j_shou_yaw
};

const double TorqueHardLimits[] = {
  30,  // j_ant_pan
  30,  // j_ant_tilt
  80, // j_dist_pitch
  80, // j_hand_yaw
  80, // j_prox_pitch
  80, // j_scoop_yaw
  80, // j_shou_pitch
  80  // j_shou_yaw
};

OwInterface* OwInterface::m_instance = nullptr;

static JointMap init_joint_map ()
{
  JointMap m;
  for (int i = 0; i < ow_lander::NUM_JOINTS; i++) m[i] = JointInfo (0,0,0);
  return m;
}

JointMap OwInterface::m_jointMap = init_joint_map();

OwInterface* OwInterface::instance ()
{
  // Very simple singleton
  if (m_instance == nullptr) m_instance = new OwInterface();
  return m_instance;
}

// Subscription callbacks

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

static double torque_soft_limit (int joint_index)
{
  return TorqueSoftLimits[joint_index];
}

static double torque_hard_limit (int joint_index)
{
  return TorqueHardLimits[joint_index];
}

static void handle_overtorque (int joint_index, double effort)
{
  // For now, torque is just effort (Newton-meter), and overtorque is specific
  // to the joint.

  if (abs(effort) >= torque_hard_limit (joint_index)) {
    JointsAtHardTorqueLimit.insert(ow_lander::joint_display_names[joint_index]);
  }
  else if (abs(effort) >= torque_soft_limit (joint_index)) {
    JointsAtSoftTorqueLimit.insert(ow_lander::joint_display_names[joint_index]);
  }
  else {
    JointsAtHardTorqueLimit.erase (ow_lander::joint_display_names[joint_index]);
    JointsAtSoftTorqueLimit.erase (ow_lander::joint_display_names[joint_index]);
  }
}

static void handle_joint_fault (int joint_index,
                                const sensor_msgs::JointState::ConstPtr& msg)
{
  // NOTE: For now, the only fault is overtorque.
  handle_overtorque (joint_index, msg->effort[joint_index]);
}

void OwInterface::jointStatesCallback
(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Publish all joint information for visibility to PLEXIL and handle any
  // joint-related faults.

  for (int i = 0; i < ow_lander::NUM_JOINTS; ++i) { // NOTE: archaic enum iteration
    m_jointMap[i] = JointInfo (msg->position[i],
                               msg->velocity[i],
                               msg->effort[i]);
    publish (ow_lander::joint_display_names[i] + " velocity", msg->velocity[i]);
    publish (ow_lander::joint_display_names[i] + " effort", msg->effort[i]);
    publish (ow_lander::joint_display_names[i] + " position", msg->position[i]);
    handle_joint_fault (i, msg);
  }
}

static void camera_callback (const sensor_msgs::Image::ConstPtr& msg)
{
  // Assuming that receipt of this message is success itself.
  ImageReceived = true;
  publish ("ImageReceived", ImageReceived);
}

OwInterface::OwInterface ()
  : m_genericNodeHandle (nullptr),
    m_antennaTiltPublisher (nullptr),
    m_antennaPanPublisher (nullptr),
    m_leftImageTriggerPublisher (nullptr),
    m_antennaTiltSubscriber (nullptr),
    m_antennaPanSubscriber (nullptr),
    m_jointStatesSubscriber (nullptr),
    m_cameraSubscriber (nullptr)
{ }

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

    // Holding off on this for now, as latching seems to do the trick.
    //    if (subscribersConfirmed()) initialized = true;
    //    else ROS_ERROR("Could not initialize OwInterface: subscribers down.");
  }
}

void OwInterface::startPlanningDemo()
{
  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    // NOTE: typo is deliberate
    nhandle.serviceClient<ow_lander::StartPlanning>("start_plannning_session");

  if (! client.exists()) {
    ROS_ERROR("Service client does not exist!");
  }
  else if (! client.isValid()) {
    ROS_ERROR("Service client is invalid!");
  }
  else {
    ow_lander::StartPlanning srv;
    srv.request.use_defaults = true;
    srv.request.trench_x = 0.0;
    srv.request.trench_y = 0.0;
    srv.request.trench_d = 0.0;
    srv.request.delete_prev_traj = false;

    if (client.call(srv)) {
      ROS_INFO("StartPlanning returned: %d, %s",
               srv.response.success,
               srv.response.message.c_str());
    }
    else {
      ROS_ERROR("Failed to call service StartPlanning");
    }
  }
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
  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::MoveGuarded>("start_move_guarded");

  if (! client.exists()) {
    ROS_ERROR("Service client does not exist!");
  }
  else if (! client.isValid()) {
    ROS_ERROR("Service client is invalid!");
  }
  else {
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

    if (client.call(srv)) {
      ROS_INFO("MoveGuarded returned: %d, %s",
               srv.response.success,
               srv.response.message.c_str());
    }
    else {
      ROS_ERROR("Failed to call service MoveGuarded");
    }
  }
}

void OwInterface::publishTrajectoryDemo()
{
  ros::NodeHandle nhandle ("planning");

  ros::ServiceClient client =
    nhandle.serviceClient<ow_lander::PublishTrajectory>("publish_trajectory");

  if (! client.exists()) {
    ROS_ERROR("Service client does not exist!");
  }
  else if (! client.isValid()) {
    ROS_ERROR("Service client is invalid!");
  }
  else {
    ow_lander::PublishTrajectory srv;
    srv.request.use_latest = true;
    srv.request.trajectory_filename = "ow_lander_trajectory.txt";
    if (client.call(srv)) {
      ROS_INFO("PublishTrajectory returned: %d, %s",
               srv.response.success,
               srv.response.message.c_str());
    }
    else {
      ROS_ERROR("Failed to call service PublishTrajectory");
    }
  }
}

// NOT USED - will remove if latching keeps working
bool OwInterface::subscribersConfirmed () const
{
  ros::Publisher* pubs [] = { m_antennaTiltPublisher,
                              m_antennaPanPublisher,
                              m_leftImageTriggerPublisher };

  for (auto p : pubs) if (! checkSubscribers (p)) return false;
  return true;
}


// NOT USED - will remove if latching keeps working
bool OwInterface::checkSubscribers (const ros::Publisher* pub) const
{
  int timeout = 5;  // seconds, arbitrary
  for (int secs = 0; secs < timeout; secs++) {
    if (pub->getNumSubscribers() > 0) return true;
    else sleep(1);
  }
  ROS_ERROR("No subscribers for topic %s", pub->getTopic().c_str());
  return false;
}


void OwInterface::tiltAntenna (double arg)
{
  std_msgs::Float64 msg;
  msg.data = arg * D2R;
  ROS_INFO("Tilting to %f degrees (%f radians)", arg, msg.data);
  m_antennaTiltPublisher->publish (msg);
}

void OwInterface::panAntenna (double arg)
{
  std_msgs::Float64 msg;
  msg.data = arg * D2R;
  ROS_INFO("Panning to %f degrees (%f radians)", arg, msg.data);
  m_antennaPanPublisher->publish (msg);
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
  ROS_WARN("digTrench is unimplemented!");
}


// State

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
  return m_jointMap[ow_lander::J_ANT_PAN].velocity;
}

double OwInterface::getTiltVelocity () const
{
  return m_jointMap[ow_lander::J_ANT_TILT].velocity;
}

bool OwInterface::imageReceived () const
{
  return ImageReceived;
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
