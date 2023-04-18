// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef OwlatInterface_H
#define OwlatInterface_H

// Interface to JPL's OWLAT simulator.

#include "LanderInterface.h"

// OWLAT Sim (installation required)
#include <owl_msgs/ArmFindSurfaceAction.h>
#include <owl_msgs/TaskDiscardSampleAction.h>
#include <owl_msgs/ArmTareFTSensorAction.h>
#include <owl_msgs/ArmSetToolAction.h>
#include <owl_msgs/TaskPSPAction.h>

#include <owlat_sim_msgs/ARM_MOVE_JOINTSAction.h>
#include <owlat_sim_msgs/ARM_MOVE_JOINTS_GUARDEDAction.h>
#include <owlat_sim_msgs/ARM_PLACE_TOOLAction.h>
#include <owlat_sim_msgs/TASK_SCOOPAction.h>
#include <owlat_sim_msgs/ARM_JOINT_ANGLES.h>
#include <owlat_sim_msgs/ARM_JOINT_ACCELERATIONS.h>
#include <owlat_sim_msgs/ARM_JOINT_TORQUES.h>
#include <owlat_sim_msgs/ARM_JOINT_VELOCITIES.h>
#include <owlat_sim_msgs/ARM_FT_TORQUE.h>
#include <owlat_sim_msgs/ARM_FT_FORCE.h>
#include <owlat_sim_msgs/ARM_POSE.h>
#include <owlat_sim_msgs/ARM_TOOL.h>

using ArmFindSurfaceActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmFindSurfaceAction>;
using ArmTareFTSensorActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmTareFTSensorAction>;
using TaskDiscardSampleActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskDiscardSampleAction>;
using ArmSetToolActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmSetToolAction>;
using TaskPSPActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskPSPAction>;

using OwlatArmMoveJointsActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_JOINTSAction>;
using OwlatArmMoveJointsGuardedActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_JOINTS_GUARDEDAction>;
using OwlatArmPlaceToolActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_PLACE_TOOLAction>;
using OwlatTaskScoopActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::TASK_SCOOPAction>;

class OwlatInterface : public LanderInterface
{
 public:
  static OwlatInterface* instance();
  OwlatInterface() = default;
  ~OwlatInterface() = default;
  OwlatInterface (const OwlatInterface&) = delete;
  OwlatInterface& operator= (const OwlatInterface&) = delete;

  void initialize();

  // Lander interface
  void armFindSurface (int frame,
                       bool relative,
                       const std::vector<double>& pos,
                       const std::vector<double>& normal,
                       double distance,
                       double overdrive,
                       double force_threshold,
                       double torque_threshold,
                       int id) override;
  void taskDiscard (int id);
  void owlatArmMoveJoints (bool relative,
                           const std::vector<double>& angles,
                           int id);
  void owlatArmMoveJointsGuarded (bool relative,
                                  const std::vector<double>& angles,
                                  bool retracting, double force_threshold,
                                  double torque_threshold, int id);
  void owlatArmPlaceTool (int frame,
                          bool relative,
                          const std::vector<double>& position,
                          const std::vector<double>& normal,
                          double distance, double overdrive,
                          bool retracting, double force_threshold,
                          double torque_threshold, int id);
  void armSetTool (int tool, int id);
  void armTareFTSensor (int id);
  void taskPSP (int frame, bool relative, const std::vector<double>& point,
                     const std::vector<double>& normal, double max_depth,
                     double max_force, int id);
  void owlatTaskScoop (int frame, bool relative, const std::vector<double>& point,
                       const std::vector<double>& normal, int id);

  // Lookups
  PLEXIL::Value getArmJointAngles() const;
  PLEXIL::Value getArmJointAccelerations() const;
  PLEXIL::Value getArmJointTorques() const ;
  PLEXIL::Value getArmJointVelocities() const;
  PLEXIL::Value getArmFTTorque() const;
  PLEXIL::Value getArmFTForce() const;
  PLEXIL::Value getArmPose() const;
  PLEXIL::Value getArmTool() const;
  PLEXIL::Value getPSPStopReason() const;
  PLEXIL::Value getPanDegrees() const;
  PLEXIL::Value getPanRadians() const;
  PLEXIL::Value getTiltRadians() const;
  PLEXIL::Value getTiltDegrees() const;
  PLEXIL::Value getJointTelemetry (int joint, TelemetryType type) const;

 private:
  // Actions
  void armFindSurfaceAction (int frame, bool relative,
                             const std::vector<double>& position,
                             const std::vector<double>& normal,
                             double distance, double overdrive,
                             double force_threshold, double torque_threshold,
                             int id);
  void owlatArmMoveJointsAction (bool relative, const std::vector<double>& angles,
                                 int id);
  void owlatArmMoveJointsGuardedAction (bool relative,
                                        const std::vector<double>& angles,
                                        bool retracting, double force_threshold,
                                        double torque_threshold, int id);
  void owlatArmPlaceToolAction (int frame, bool relative,
                                const std::vector<double>& position,
                                const std::vector<double>& normal,
                                double distance, double overdrive,
                                bool retracting, double force_threshold,
                                double torque_threshold, int id);
  void armSetToolAction (int tool, int id);
  void taskPSPAction (int frame, bool relative,
                           const std::vector<double>& point,
                           const std::vector<double>& normal, double max_depth,
                           double max_force, int id);
  void owlatTaskScoopAction (int frame, bool relative,
                             const std::vector<double>& point,
                             const std::vector<double>& normal, int id);

  // Callbacks
  void armJointAnglesCallback
  (const owlat_sim_msgs::ARM_JOINT_ANGLES::ConstPtr& msg);
  void armJointAccelerationsCallback
  (const owlat_sim_msgs::ARM_JOINT_ACCELERATIONS::ConstPtr&);
  void armJointTorquesCallback
  (const owlat_sim_msgs::ARM_JOINT_TORQUES::ConstPtr& msg);
  void armJointVelocitiesCallback
  (const owlat_sim_msgs::ARM_JOINT_VELOCITIES::ConstPtr& msg);
  void armFTTorqueCallback(const owlat_sim_msgs::ARM_FT_TORQUE::ConstPtr& msg);
  void armFTForceCallback(const owlat_sim_msgs::ARM_FT_FORCE::ConstPtr& msg);
  void armPoseCallback(const owlat_sim_msgs::ARM_POSE::ConstPtr& msg);
  void armToolCallback(const owlat_sim_msgs::ARM_TOOL::ConstPtr& msg);
  void panTiltCallback (const owl_msgs::PanTiltPosition::ConstPtr& msg);

  // Action Clients
  std::unique_ptr<ArmFindSurfaceActionClient> m_armFindSurfaceClient;
  std::unique_ptr<TaskDiscardSampleActionClient> m_taskDiscardClient;
  std::unique_ptr<OwlatArmMoveJointsActionClient> m_owlatArmMoveJointsClient;
  std::unique_ptr<OwlatArmMoveJointsGuardedActionClient> m_owlatArmMoveJointsGuardedClient;
  std::unique_ptr<OwlatArmPlaceToolActionClient> m_owlatArmPlaceToolClient;
  std::unique_ptr<ArmSetToolActionClient> m_armSetToolClient;
  std::unique_ptr<ArmTareFTSensorActionClient> m_armTareFTSensorClient;
  std::unique_ptr<TaskPSPActionClient> m_taskPSPClient;
  std::unique_ptr<OwlatTaskScoopActionClient> m_owlatTaskScoopClient;

  // Member variables
  std::vector<double> m_arm_joint_angles;
  std::vector<double> m_arm_joint_accelerations;
  std::vector<double> m_arm_joint_torques;
  std::vector<double> m_arm_joint_velocities;
  std::vector<double> m_arm_ft_torque;
  std::vector<double> m_arm_ft_force;
  std::vector<double> m_arm_pose;
  double m_arm_tool;
  double m_pan_radians;
  double m_tilt_radians;
};

#endif
