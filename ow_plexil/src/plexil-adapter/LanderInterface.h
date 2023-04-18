// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef LanderInterface_H
#define LanderInterface_H

// Interface shared by OceanWATERS and JPL's OWLAT landers.

// ow_plexil
#include "PlexilInterface.h"
#include "joint_support.h"

// PLEXIL
#include <Value.hh>

// owl_msgs - actions
#include <owl_msgs/ArmMoveCartesianAction.h>
#include <owl_msgs/ArmMoveCartesianGuardedAction.h>
#include <owl_msgs/ArmMoveJointAction.h>
#include <owl_msgs/ArmStopAction.h>
#include <owl_msgs/ArmStowAction.h>
#include <owl_msgs/ArmUnstowAction.h>

// owl_msgs - telemetry
#include <owl_msgs/PanTiltPosition.h>

// ROS
#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>

// C++
#include <string>
#include <memory>

// Action client short forms

using ArmMoveCartesianActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveCartesianAction>;
using ArmMoveCartesianGuardedActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveCartesianGuardedAction>;
using ArmMoveJointActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointAction>;
using ArmStopActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmStopAction>;
using ArmStowActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmStowAction>;
using ArmUnstowActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmUnstowAction>;


class LanderInterface : public PlexilInterface
{
 public:
  LanderInterface() = default;
  virtual ~LanderInterface() = default;
  LanderInterface (const LanderInterface&) = delete;
  LanderInterface& operator= (const LanderInterface&) = delete;

  void initialize();

  // Lander interface
  virtual void armFindSurface (int frame,
                               bool relative,
                               const std::vector<double>& pos,
                               const std::vector<double>& normal,
                               double distance,
                               double overdrive,
                               double force_threshold,
                               double torque_threshold,
                               int id) = 0;

  void armMoveCartesian (int frame, bool relative,
                         const std::vector<double>& position,
                         const std::vector<double>& orientation,
                         int id);
  void armMoveCartesianGuarded (int frame, bool relative,
                                const std::vector<double>& position,
                                const std::vector<double>& orientation,
                                double force_threshold,
                                double torque_threshold,
                                int id);
  void armMoveJoint (bool relative, int joint, double angle, int id);
  void armStop (int id);
  void armStow (int id);
  void armUnstow (int id);

 private:
  // Actions
  void armMoveCartesianAction (int frame,
                               bool relative,
                               const geometry_msgs::Pose& pose,
                               int id);
  void armMoveCartesianGuardedAction (int frame,
                                      bool relative,
                                      const geometry_msgs::Pose&,
                                      double force_threshold,
                                      double torque_threshold,
                                      int id);
  void armMoveJointAction (bool relative, int joint, double angle, int id);

  // Action Clients

  std::unique_ptr<ArmMoveCartesianActionClient> m_armMoveCartesianClient;
  std::unique_ptr<ArmMoveCartesianGuardedActionClient> m_armMoveCartesianGuardedClient;
  std::unique_ptr<ArmMoveJointActionClient> m_armMoveJointClient;
  std::unique_ptr<ArmStopActionClient> m_armStopClient;
  std::unique_ptr<ArmStowActionClient> m_armStowClient;
  std::unique_ptr<ArmUnstowActionClient> m_armUnstowClient;
};

#endif
