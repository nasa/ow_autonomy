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
#include <owl_msgs/TaskShearBevameterAction.h>
#include <owl_msgs/TaskPenetrometerAction.h>
#include <owl_msgs/TaskScoopCircularAction.h>
#include <owl_msgs/TaskScoopLinearAction.h>
#include <owl_msgs/ArmMoveJointsAction.h>
#include <owl_msgs/ArmMoveJointsGuardedAction.h>

// TODO: convert these per unified telemetry interface
#include <owlat_sim_msgs/ARM_JOINT_ANGLES.h>
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
using TaskShearBevameterActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskShearBevameterAction>;
using TaskPenetrometerActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskPenetrometerAction>;
using TaskScoopCircularActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskScoopCircularAction>;
using TaskScoopLinearActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskScoopLinearAction>;
using ArmMoveJointsGuardedActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointsGuardedAction>;
using ArmMoveJointsActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointsAction>;

#include <owl_msgs/ArmEndEffectorForceTorque.h>

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
  void taskDiscardSample (int frame, bool relative,
                          const std::vector<double>& point,
                          double height, int id) override;
  void armMoveJoints (bool relative, const std::vector<double>& angles, int id);
  void armMoveJointsGuarded (bool relative, const std::vector<double>& angles,
                             double force_threshold, double torque_threshold,
                             int id);
  void armSetTool (int tool, int id);
  void armTareFTSensor (int id);
  void taskPSP (int frame, bool relative,
                const std::vector<double>& point,
                const std::vector<double>& normal,
                double max_depth, double max_force,
                int id);
  void taskPenetrometer (int frame, bool relative,
                         const std::vector<double>& point,
                         const std::vector<double>& normal,
                         double max_depth, double max_force,
                         int id);
  void taskShearBevameter (int frame, bool relative,
                           const std::vector<double>& point,
                           const std::vector<double>& normal,
                           double preload, double max_torque,
                           int id);
  void taskScoopCircular (int frame,
                          bool relative,
                          const std::vector<double>& point,
                          const std::vector<double>& normal,
                          double depth, double scoop_angle,
                          int id);
  void taskScoopLinear (int frame,
                        bool relative,
                        const std::vector<double>& point,
                        const std::vector<double>& normal,
                        double depth, double scoop_angle,
                        int id);

  // Lookups
  PLEXIL::Value getArmJointAngles() const;
  PLEXIL::Value getArmJointTorques() const ;
  PLEXIL::Value getArmJointVelocities() const;
  PLEXIL::Value getArmFTTorque() const;
  PLEXIL::Value getArmFTForce() const;
  PLEXIL::Value getArmPose() const;
  PLEXIL::Value getArmTool() const;
  PLEXIL::Value getPanDegrees() const;
  PLEXIL::Value getPanRadians() const;
  PLEXIL::Value getTiltRadians() const;
  PLEXIL::Value getTiltDegrees() const;
  PLEXIL::Value getJointTelemetry (int joint, TelemetryType type) const;
  bool systemFault () const override;

 private:
  // Actions
  void armFindSurfaceAction (int frame, bool relative,
                             const std::vector<double>& position,
                             const std::vector<double>& normal,
                             double distance, double overdrive,
                             double force_threshold, double torque_threshold,
                             int id);
  void taskDiscardSampleAction (int frame, bool relative,
                                const std::vector<double>& point,
                                double height, int id);
  void armMoveJointsAction (bool relative, const std::vector<double>& angles,
                            int id);
  void armMoveJointsGuardedAction (bool relative,
                                   const std::vector<double>& angles,
                                   double force_threshold,
                                   double torque_threshold, int id);
  void armSetToolAction (int tool, int id);
  void taskPSPAction (int frame, bool relative,
                      const std::vector<double>& point,
                      const std::vector<double>& normal,
                      double max_depth, double max_force,
                      int id);
  void taskShearBevameterAction (int frame, bool relative,
                                 const std::vector<double>& point,
                                 const std::vector<double>& normal,
                                 double preload, double max_torque,
                                 int id);
  void taskPenetrometerAction (int frame, bool relative,
                               const std::vector<double>& point,
                               const std::vector<double>& normal,
                               double max_depth, double max_force, int id);
  void taskScoopLinearAction (int frame, bool relative,
                              const std::vector<double>& point,
                              const std::vector<double>& normal,
                              double depth, double length,
                              int id);
  void taskScoopCircularAction (int frame, bool relative,
                                const std::vector<double>& point,
                                const std::vector<double>& normal,
                                double depth, double scoop_angle,
                                int id);

  // Callbacks
  void ftCallback (const owl_msgs::ArmEndEffectorForceTorque::ConstPtr&);
  void armJointAnglesCallback
  (const owlat_sim_msgs::ARM_JOINT_ANGLES::ConstPtr& msg);
  void armJointTorquesCallback
  (const owlat_sim_msgs::ARM_JOINT_TORQUES::ConstPtr& msg);
  void armJointVelocitiesCallback
  (const owlat_sim_msgs::ARM_JOINT_VELOCITIES::ConstPtr& msg);
  void armPoseCallback(const owlat_sim_msgs::ARM_POSE::ConstPtr& msg);
  void armToolCallback(const owlat_sim_msgs::ARM_TOOL::ConstPtr& msg);
  void panTiltCallback (const owl_msgs::PanTiltPosition::ConstPtr& msg);
  void systemFaultMessageCallback (const owl_msgs::SystemFaultsStatus::ConstPtr& msg);

  // Action Clients
  std::unique_ptr<ArmFindSurfaceActionClient> m_armFindSurfaceClient;
  std::unique_ptr<TaskDiscardSampleActionClient> m_discardSampleClient;
  std::unique_ptr<ArmMoveJointsActionClient> m_armMoveJointsClient;
  std::unique_ptr<ArmMoveJointsGuardedActionClient> m_armMoveJointsGuardedClient;
  std::unique_ptr<ArmSetToolActionClient> m_armSetToolClient;
  std::unique_ptr<ArmTareFTSensorActionClient> m_armTareFTSensorClient;
  std::unique_ptr<TaskPSPActionClient> m_taskPSPClient;
  std::unique_ptr<TaskShearBevameterActionClient> m_taskShearBevameterClient;
  std::unique_ptr<TaskPenetrometerActionClient> m_taskPenetrometerClient;
  std::unique_ptr<TaskScoopCircularActionClient> m_taskScoopCircularClient;
  std::unique_ptr<TaskScoopLinearActionClient> m_taskScoopLinearClient;

  // Member variables

  // System-level faults:
  FaultMap m_systemErrors = {
    {"SYSTEM", std::make_pair(
        owl_msgs::SystemFaultsStatus::SYSTEM,false)},
    {"ARM_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::ARM_GOAL_ERROR,false)},
    {"ARM_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::ARM_EXECUTION_ERROR,false)},
    {"TASK_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::TASK_GOAL_ERROR,false)},
    {"CAM_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::CAM_GOAL_ERROR,false)},
    {"CAM_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::CAM_EXECUTION_ERROR,false)},
    {"PAN_TILT_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::PAN_TILT_GOAL_ERROR,false)},
    {"PAN_TILT_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::PAN_TILT_EXECUTION_ERROR,false)},
    {"POWER_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::POWER_EXECUTION_ERROR,false)}
  };

  std::vector<double> m_arm_joint_angles;
  std::vector<double> m_arm_joint_torques;
  std::vector<double> m_arm_joint_velocities;
  std::vector<double> m_arm_pose;
  std::vector<double> m_end_effector_ft;
  double m_arm_tool;
  double m_pan_radians;
  double m_tilt_radians;

};

#endif
