// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Interface specific to OceanWATERS' lander.

#ifndef Ow_Interface_H
#define Ow_Interface_H

#include "LanderInterface.h"

// ROS action used only in this package

#include <ow_plexil/IdentifyLocationAction.h>

using IdentifySampleLocationActionClient =
  actionlib::SimpleActionClient<ow_plexil::IdentifyLocationAction>;


// Testbed-common ROS actions

#include <owl_msgs/ArmFindSurfaceAction.h>
#include <owl_msgs/TaskDiscardSampleAction.h>
#include <owl_msgs/PanTiltMoveCartesianAction.h>
#include <owl_msgs/TaskGrindAction.h>
#include <owl_msgs/ArmMoveJointsAction.h>
#include <owl_msgs/ArmMoveJointsGuardedAction.h>
#include <owl_msgs/TaskScoopCircularAction.h>
#include <owl_msgs/TaskScoopLinearAction.h>
#include <owl_msgs/CameraSetExposureAction.h>
#include <owl_msgs/LightSetIntensityAction.h>

using ArmFindSurfaceActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmFindSurfaceAction>;
using TaskGrindActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskGrindAction>;
using ArmMoveJointsActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointsAction>;
using ArmMoveJointsGuardedActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointsGuardedAction>;
using PanTiltMoveCartesianActionClient =
  actionlib::SimpleActionClient<owl_msgs::PanTiltMoveCartesianAction>;
using TaskScoopCircularActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskScoopCircularAction>;
using TaskScoopLinearActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskScoopLinearAction>;
using CameraSetExposureActionClient =
  actionlib::SimpleActionClient<owl_msgs::CameraSetExposureAction>;
using LightSetIntensityActionClient =
  actionlib::SimpleActionClient<owl_msgs::LightSetIntensityAction>;
using TaskDiscardSampleActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskDiscardSampleAction>;


// OceanWATERS-specific ROS actions

#include <ow_lander/GuardedMoveAction.h>
#include <ow_lander/DockIngestSampleAction.h>
#include <ow_lander/TiltAction.h>
#include <ow_lander/PanAction.h>

using GuardedMoveActionClient =
  actionlib::SimpleActionClient<ow_lander::GuardedMoveAction>;
using PanActionClient = actionlib::SimpleActionClient<ow_lander::PanAction>;
using TiltActionClient = actionlib::SimpleActionClient<ow_lander::TiltAction>;
using DockIngestSampleActionClient =
  actionlib::SimpleActionClient<ow_lander::DockIngestSampleAction>;


// Telemetry
#include <owl_msgs/ArmEndEffectorForceTorque.h>

class OwInterface : public LanderInterface
{
 public:
  static OwInterface* instance();
  OwInterface ();
  ~OwInterface () = default;
  OwInterface (const OwInterface&) = delete;
  OwInterface& operator= (const OwInterface&) = delete;
  void initialize ();

  // Operational interface

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
  void guardedMove (double x, double y, double z,
                    double direction_x, double direction_y, double direction_z,
                    double search_distance, int id);
  void armMoveJoints (bool relative, const std::vector<double>& angles, int id);
  void armMoveJointsGuarded (bool relative,
                             const std::vector<double>& angles,
                             double force_threshold,
                             double torque_threshold,
                             int id);
  std::vector<double> identifySampleLocation (int num_images,
                                              const std::string& filter_type,
                                              int id);

  void pan (double degrees, int id);
  void tilt (double degrees, int id);
  void panTiltCartesian (int frame, double x, double y, double z, int id);
  void cameraSetExposure (double exposure_secs, int id);
  void dockIngestSample (int id);
  void scoopLinear (int frame, bool relative, double x, double y, double z,
                    double depth, double length, int id);
  void scoopCircular (int frame, bool relative, double x, double y, double z,
                      double depth, bool parallel, int id);
  void grind (double x, double y, double depth, double length,
              bool parallel, double ground_pos, int id);
  void armStop (int id);
  void armStow (int id);
  void armUnstow (int id);
  void lightSetIntensity (const std::string& side, double intensity, int id);

  // State/Lookup interface
  double getPanVelocity () const;
  double getTiltVelocity () const;
  std::vector<double> getEndEffectorFT () const;
  bool   groundFound () const;
  double groundPosition () const;
  bool   anglesEquivalent (double deg1, double deg2, double tolerance);
  bool   hardTorqueLimitReached (const std::string& joint_name) const;
  bool   softTorqueLimitReached (const std::string& joint_name) const;
  bool   systemFault () const override;
  std::vector<std::string>   getActiveFaults (std::string subsystem_name) const;
  bool   getIsOperable (std::string subsystem_name) const;
  bool   getIsFaulty (std::string subsystem_name) const;
  std::vector<double> getArmEndEffectorFT () const override;

 private:
  void armFindSurfaceAction (int frame, bool relative,
                             const geometry_msgs::Point& pos,
                             const geometry_msgs::Vector3& normal,
                             double distance, double overdrive,
                             double force_threshold, double torque_threshold,
                             int id);
  void taskDiscardSampleAction (int frame, bool relative,
                                const std::vector<double>& point,
                                double height, int id);
  void grindAction (double x, double y, double depth, double length,
                    bool parallel, double ground_pos, int id);
  void guardedMoveAction (double x, double y, double z,
                          double dir_x, double dir_y, double dir_z,
                          double search_distance, int id);
  void armMoveJointsAction (bool relative, const std::vector<double>& angles,
                            int id);
  void armMoveJointsGuardedAction (bool relative,
                                   const std::vector<double>& angles,
                                   double force_threshold,
                                   double torque_threshold,
                                   int id);
  void identifySampleLocationAction (int num_images,
                                     const std::string& filter_type, int id);
  void panAction (double degrees, int id);
  void tiltAction (double degrees, int id);
  void panTiltCartesianAction (int frame, double x, double y, double z, int id);
  void scoopLinearAction (int frame, bool relative, double x, double y, double z,
                          double depth, double length, int id);
  void scoopCircularAction (int frame, bool relative, double x, double y, double z,
                            double depth, bool parallel, int id);
  void cameraSetExposureAction (double exposure_secs, int id);
  void dockIngestSampleAction (int id);
  void lightSetIntensityAction (const std::string& side, double intensity, int id);
  void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);
  void antennaOp (const std::string& opname, double degrees,
                  std::unique_ptr<ros::Publisher>&, int id);
  void ftCallback (const owl_msgs::ArmEndEffectorForceTorque::ConstPtr&);
  void systemFaultMessageCallback (const owl_msgs::SystemFaultsStatus::ConstPtr& msg);

  // System-level faults
  FaultMap m_systemErrors = {
    {"SYSTEM", std::make_pair(
        owl_msgs::SystemFaultsStatus::SYSTEM,false)},
    {"ARM_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::ARM_GOAL_ERROR,false)},
    {"ARM_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::ARM_EXECUTION_ERROR,false)},
    {"TASK_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::TASK_GOAL_ERROR,false)},
    {"CAMERA_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::CAMERA_GOAL_ERROR,false)},
    {"CAMERA_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::CAMERA_EXECUTION_ERROR,false)},
    {"PAN_TILT_GOAL_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::PAN_TILT_GOAL_ERROR,false)},
    {"PAN_TILT_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::PAN_TILT_EXECUTION_ERROR,false)},
    {"POWER_EXECUTION_ERROR", std::make_pair(
        owl_msgs::SystemFaultsStatus::POWER_EXECUTION_ERROR,false)}
  };
  bool m_fault_dependencies_on;

  // Action clients

  std::unique_ptr<ArmFindSurfaceActionClient> m_armFindSurfaceClient;
  std::unique_ptr<GuardedMoveActionClient> m_guardedMoveClient;
  std::unique_ptr<ArmMoveJointsActionClient> m_armMoveJointsClient;
  std::unique_ptr<ArmMoveJointsGuardedActionClient> m_armMoveJointsGuardedClient;
  std::unique_ptr<TaskGrindActionClient> m_grindClient;
  std::unique_ptr<PanActionClient> m_panClient;
  std::unique_ptr<TiltActionClient> m_tiltClient;
  std::unique_ptr<PanTiltMoveCartesianActionClient> m_panTiltCartesianClient;
  std::unique_ptr<TaskScoopCircularActionClient> m_scoopCircularClient;
  std::unique_ptr<TaskScoopLinearActionClient> m_scoopLinearClient;
  std::unique_ptr<CameraSetExposureActionClient> m_cameraSetExposureClient;
  std::unique_ptr<DockIngestSampleActionClient> m_dockIngestSampleClient;
  std::unique_ptr<LightSetIntensityActionClient> m_lightSetIntensityClient;
  std::unique_ptr<IdentifySampleLocationActionClient> m_identifySampleLocationClient;
  std::unique_ptr<TaskDiscardSampleActionClient> m_taskDiscardSampleClient;

  // Misc state
  std::vector<double> m_end_effector_ft;
};

#endif
