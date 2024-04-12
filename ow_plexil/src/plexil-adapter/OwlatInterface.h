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

// NOTE: The simulator still implements this older message file, and
// not the newer one in owl_msgs/ArmTool.h.
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
  OwlatInterface();
  ~OwlatInterface() = default;
  OwlatInterface (const OwlatInterface&) = delete;
  OwlatInterface& operator= (const OwlatInterface&) = delete;

  void initialize();

  // Fault-related Lookup support
  bool systemFault () const override;
  bool armGoalError () const;
  bool armExecutionError () const;
  bool taskGoalError () const;
  bool cameraGoalError () const;
  bool cameraExecutionError () const;
  bool panTiltGoalError () const;
  bool panTiltExecutionError () const;
  bool drillGoalError () const;
  bool drillExecutionError () const;
  bool landerExecutionError () const;
  bool miscSystemError () const;

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
  PLEXIL::Value getArmTool() const;
  std::vector<double> getArmEndEffectorFT () const override;
  bool   armGoalError () const;
  bool   cameraGoalError () const;
  bool   panTiltGoalError () const;
  bool   drillGoalError () const;
  bool   taskGoalError () const;

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
  void armToolCallback(const owlat_sim_msgs::ARM_TOOL::ConstPtr& msg);
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

  // See detailed explanation of FaultMap in LanderInterface.h

  FaultMap m_systemErrors =
  {  // The first flag covers faults that don't have their own flag.
   { "MiscSystemError", std::make_pair(
        owl_msgs::SystemFaultsStatus::SYSTEM,false)},
    {"ArmGoalError", std::make_pair(
        owl_msgs::SystemFaultsStatus::ARM_GOAL_ERROR,false)},
    {"ArmExecutionError", std::make_pair(
        owl_msgs::SystemFaultsStatus::ARM_EXECUTION_ERROR,false)},
    {"TaskGoalError", std::make_pair(
        owl_msgs::SystemFaultsStatus::TASK_GOAL_ERROR,false)},
    {"CameraGoalError", std::make_pair(
        owl_msgs::SystemFaultsStatus::CAMERA_GOAL_ERROR,false)},
    {"CameraExecutionError", std::make_pair(
        owl_msgs::SystemFaultsStatus::CAMERA_EXECUTION_ERROR,false)},
    {"PanTiltGoalError", std::make_pair(
        owl_msgs::SystemFaultsStatus::PAN_TILT_GOAL_ERROR,false)},
    {"PanTiltExecutionError", std::make_pair(
        owl_msgs::SystemFaultsStatus::PAN_TILT_EXECUTION_ERROR,false)},
    {"DrillGoalError", std::make_pair(
        owl_msgs::SystemFaultsStatus::DRILL_GOAL_ERROR,false)},
    {"DrillExecutionError", std::make_pair(
        owl_msgs::SystemFaultsStatus::DRILL_EXECUTION_ERROR,false)},
    {"LanderExecutionError", std::make_pair( // fault in the Stewart platform
        owl_msgs::SystemFaultsStatus::LANDER_EXECUTION_ERROR,false)}
  };

  std::vector<double> m_end_effector_ft;
  int m_arm_tool;
};

#endif
