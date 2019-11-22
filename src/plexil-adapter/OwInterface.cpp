// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

// OW
#include "OwInterface.h"
#include <ow_lander/StartPlanning.h>
#include <ow_lander/MoveGuarded.h>
#include <ow_lander/PublishTrajectory.h>

// ROS
#include <std_msgs/Float64.h>

OwInterface* OwInterface::instance ()
{
  // Very simple singleton
  static OwInterface* inst = new OwInterface();  // TODO: eliminate memory leak
  return inst;
}

OwInterface::OwInterface ()
  : m_genericNodeHandle (NULL),
    m_antennaTiltPublisher (NULL)
{ }

OwInterface::~OwInterface ()
{
  if (m_genericNodeHandle) delete m_genericNodeHandle;
  if (m_antennaTiltPublisher) delete m_antennaTiltPublisher;
}

void OwInterface::initialize()
{
  // Hack?  Does this function need to be idempotent?
  static bool initialized = false;

  if (not initialized) {
    m_genericNodeHandle = new ros::NodeHandle();
    m_antennaTiltPublisher = new ros::Publisher
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_tilt_position_controller/command", 1));
    m_antennaPanPublisher = new ros::Publisher
      (m_genericNodeHandle->advertise<std_msgs::Float64>
       ("/ant_pan_position_controller/command", 1));
    initialized = true;
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

void OwInterface::tiltAntenna (double arg)
{
  std_msgs::Float64 msg;
  msg.data = arg;
  m_antennaTiltPublisher->publish (msg);
}

void OwInterface::panAntenna (double arg)
{
  std_msgs::Float64 msg;
  msg.data = arg;
  m_antennaPanPublisher->publish (msg);
}
