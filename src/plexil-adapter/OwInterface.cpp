// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// ow_autonomy
#include "OwInterface.h"
#include "subscriber.h"

// OW - other
#include <ow_lander/StartPlanning.h>
#include <ow_lander/MoveGuarded.h>
#include <ow_lander/PublishTrajectory.h>

// ROS
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>

// C
#include <math.h>  // for M_PI

// Degree/Radian
const double D2R = M_PI / 180.0 ;
const double R2D = 180.0 / M_PI ;

// Lander state cache, simple start for now -- may need to refactor later.
// Individual variables for now -- may want to employ a container if this gets
// big.

double CurrentTilt         = 0.0;
double CurrentPanDegrees   = 0.0;
bool   ImageReceived       = false;

OwInterface* OwInterface::m_instance = nullptr;

//JointMap OwInterface::m_jointMap { {j_ant_pan, JointInfo(0,0,0)} };

static JointMap init_joint_map ()
{
  JointMap m;
  for (int i = j_ant_pan; i <= j_shou_yaw; i++) m[i] = JointInfo (0,0,0);
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

void OwInterface::jointStatesCallback
(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (int i = j_ant_pan; i <= j_shou_yaw; ++i) {
    m_jointMap[i] = JointInfo (msg->position[i],
                               msg->velocity[i],
                               msg->effort[i]);
    publish (JointNames[i] + "Velocity", msg->velocity[i]);
    publish (JointNames[i] + "Effort", msg->effort[i]);
    // Don't need to publish position as yet.
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
    srv.request.use_defaults = true;
    srv.request.target_x = 0.0;
    srv.request.target_y = 0.0;
    srv.request.target_z = 0.0;
    srv.request.surface_normal_x = 0.0;
    srv.request.surface_normal_y = 0.0;
    srv.request.surface_normal_z = 0.0;
    srv.request.offset_distance = 0.0;
    srv.request.overdrive_distance = 0.0;
    srv.request.retract = false;

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
  return m_jointMap[j_ant_pan].velocity;
}

double OwInterface::getTiltVelocity () const
{
  return m_jointMap[j_ant_tilt].velocity;
}

bool OwInterface::imageReceived () const
{
  return ImageReceived;
}
