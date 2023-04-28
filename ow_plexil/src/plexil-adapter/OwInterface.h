// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface specific to OceanWATERS' lander.

// ow_plexil
#include "LanderInterface.h"
#include <ow_plexil/IdentifyLocationAction.h>

// owl_msgs (lander commands)
#include <owl_msgs/ArmFindSurfaceAction.h>
#include <owl_msgs/TaskDiscardSampleAction.h>
#include <ow_lander/PanAction.h>
#include <ow_lander/TiltAction.h>
#include <owl_msgs/PanTiltMoveCartesianAction.h>
#include <owl_msgs/TaskGrindAction.h>
#include <ow_lander/GuardedMoveAction.h>
#include <owl_msgs/ArmMoveJointsAction.h>
#include <owl_msgs/ArmMoveJointsGuardedAction.h>
#include <owl_msgs/TaskScoopCircularAction.h>
#include <owl_msgs/TaskScoopLinearAction.h>
#include <owl_msgs/CameraSetExposureAction.h>
#include <ow_lander/DockIngestSampleAction.h>
#include <owl_msgs/LightSetIntensityAction.h>

// owl_msgs (telemetry)
#include <owl_msgs/ArmJointAccelerations.h>
#include <owl_msgs/ArmFaultsStatus.h>
#include <owl_msgs/PanTiltFaultsStatus.h>
#include <owl_msgs/PowerFaultsStatus.h>
#include <owl_msgs/CameraFaultsStatus.h>
#include <owl_msgs/SystemFaultsStatus.h>
#include <owl_msgs/ArmEndEffectorForceTorque.h>
#include <owl_msgs/ArmPose.h>

using ArmFindSurfaceActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmFindSurfaceAction>;
using TaskGrindActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskGrindAction>;
using GuardedMoveActionClient =
  actionlib::SimpleActionClient<ow_lander::GuardedMoveAction>;
using ArmMoveJointsActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointsAction>;
using ArmMoveJointsGuardedActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointsGuardedAction>;
using PanActionClient = actionlib::SimpleActionClient<ow_lander::PanAction>;
using TiltActionClient = actionlib::SimpleActionClient<ow_lander::TiltAction>;
using PanTiltMoveCartesianActionClient =
  actionlib::SimpleActionClient<owl_msgs::PanTiltMoveCartesianAction>;
using TaskScoopCircularActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskScoopCircularAction>;
using TaskScoopLinearActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskScoopLinearAction>;
using CameraSetExposureActionClient =
  actionlib::SimpleActionClient<owl_msgs::CameraSetExposureAction>;
using DockIngestSampleActionClient =
  actionlib::SimpleActionClient<ow_lander::DockIngestSampleAction>;
using LightSetIntensityActionClient =
  actionlib::SimpleActionClient<owl_msgs::LightSetIntensityAction>;
using TaskDiscardSampleActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskDiscardSampleAction>;

// The only ow_plexil-defined action.
using IdentifySampleLocationActionClient =
  actionlib::SimpleActionClient<ow_plexil::IdentifyLocationAction>;


// Maps from fault name to the pair (fault value, is fault in progress?)
using FaultMap32 = std::map<std::string,std::pair<uint32_t, bool>>;
using FaultMap64 = std::map<std::string,std::pair<uint64_t, bool>>;

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
  double getTiltRadians () const;
  double getTiltDegrees () const;
  double getPanRadians () const;
  double getPanDegrees () const;
  double getPanVelocity () const;
  double getTiltVelocity () const;
  double getBatteryStateOfCharge () const;
  double getBatteryRemainingUsefulLife () const;
  double getBatteryTemperature () const;
  std::vector<double> getEndEffectorFT () const;
  std::vector<double> getArmPose () const;
  bool   groundFound () const;
  double groundPosition () const;
  bool   systemFault () const;
  bool   antennaFault () const;
  bool   antennaPanFault () const;
  bool   antennaTiltFault () const;
  bool   armFault () const;
  bool   powerFault () const;
  bool   cameraFault () const;
  bool   anglesEquivalent (double deg1, double deg2, double tolerance);
  bool   hardTorqueLimitReached (const std::string& joint_name) const;
  bool   softTorqueLimitReached (const std::string& joint_name) const;
  double jointTelemetry (int joint, TelemetryType type) const;

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
  void armJointAccelerationsCb (const owl_msgs::ArmJointAccelerations::ConstPtr&);
  void systemFaultMessageCallback (const owl_msgs::SystemFaultsStatus::ConstPtr&);
  void armFaultCallback (const owl_msgs::ArmFaultsStatus::ConstPtr&);
  void powerFaultCallback (const owl_msgs::PowerFaultsStatus::ConstPtr&);
  void antennaFaultCallback (const owl_msgs::PanTiltFaultsStatus::ConstPtr&);
  void cameraFaultCallback (const owl_msgs::CameraFaultsStatus::ConstPtr&);
  void antennaOp (const std::string& opname, double degrees,
                  std::unique_ptr<ros::Publisher>&, int id);
  void ftCallback (const owl_msgs::ArmEndEffectorForceTorque::ConstPtr&);
  void armPoseCallback (const owl_msgs::ArmPose::ConstPtr&);

  template <typename T1, typename T2>
    void updateFaultStatus (T1 msg_val, T2&,
                            const std::string& component_name,
                            const std::string& state_name); // PLEXIL Lookup name

  template <typename T>
    bool faultActive (const T& fault_map) const;

  // System level faults:

  FaultMap64 m_systemErrors = {
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

  FaultMap64 m_armErrors = {
    {"HARDWARE", std::make_pair(
        owl_msgs::ArmFaultsStatus::HARDWARE, false)},
    {"TRAJECTORY_GENERATION", std::make_pair(
        owl_msgs::ArmFaultsStatus::TRAJECTORY_GENERATION, false)},
    {"COLLISION", std::make_pair(
        owl_msgs::ArmFaultsStatus::COLLISION, false)},
    {"E_STOP", std::make_pair(
        owl_msgs::ArmFaultsStatus::E_STOP, false)},
    {"POSITION_LIMIT", std::make_pair(
        owl_msgs::ArmFaultsStatus::POSITION_LIMIT, false)},
    {"JOINT_TORQUE_LIMIT", std::make_pair(
        owl_msgs::ArmFaultsStatus::JOINT_TORQUE_LIMIT, false)},
    {"VELOCITY_LIMIT", std::make_pair(
        owl_msgs::ArmFaultsStatus::VELOCITY_LIMIT, false)},
    {"NO_FORCE_DATA", std::make_pair(
        owl_msgs::ArmFaultsStatus::NO_FORCE_DATA, false)},
    {"FORCE_TORQUE_LIMIT", std::make_pair(
        owl_msgs::ArmFaultsStatus::FORCE_TORQUE_LIMIT, false)},
  };

  FaultMap64 m_powerErrors = {
    {"LOW_STATE_OF_CHARGE", std::make_pair(
        owl_msgs::PowerFaultsStatus::LOW_STATE_OF_CHARGE, false)},
    {"INSTANTANEOUS_CAPACITY_LOSS", std::make_pair(
        owl_msgs::PowerFaultsStatus::INSTANTANEOUS_CAPACITY_LOSS, false)},
    {"THERMAL_FAULT", std::make_pair(
        owl_msgs::PowerFaultsStatus::THERMAL_FAULT, false)}
  };

  const char* FaultPanJointLocked = "PAN_JOINT_LOCKED";
  const char* FaultTiltJointLocked = "TILT_JOINT_LOCKED";

  FaultMap64 m_panTiltErrors = {
    {FaultPanJointLocked, std::make_pair(
      owl_msgs::PanTiltFaultsStatus::PAN_JOINT_LOCKED, false)},
    {"TILT_JOINT_LOCKED", std::make_pair(
      owl_msgs::PanTiltFaultsStatus::TILT_JOINT_LOCKED, false)}
  };

  const char* FaultNoImage = "NO_IMAGE";

  FaultMap64 m_cameraErrors = {
    {FaultNoImage, std::make_pair(owl_msgs::CameraFaultsStatus::NO_IMAGE, false)}
  };

  // Publishers and subscribers

  std::unique_ptr<ros::Publisher> m_antennaTiltPublisher;
  std::unique_ptr<ros::Publisher> m_antennaPanPublisher;
  std::unique_ptr<ros::Publisher> m_leftImageTriggerPublisher;

  // Generic container because the subscribers are not referenced;
  // only their callback functions are of use.
  std::vector<std::unique_ptr<ros::Subscriber>> m_subscribers;

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
  double m_currentPanRadians, m_currentTiltRadians;
  std::vector<double> m_endEffectorFT;
  std::vector<double> m_armPose;
};

#endif
