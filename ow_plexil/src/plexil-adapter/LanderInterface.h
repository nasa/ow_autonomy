// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef LanderInterface_H
#define LanderInterface_H

// Interface shared by OceanWATERS and JPL's OWLAT landers.

// ow_plexil
#include "PlexilInterface.h"

// PLEXIL
#include <Value.hh>

// owl_msgs - actions
#include <owl_msgs/ArmMoveCartesianAction.h>
#include <owl_msgs/ArmMoveCartesianGuardedAction.h>
#include <owl_msgs/ArmMoveJointAction.h>
#include <owl_msgs/ArmStopAction.h>
#include <owl_msgs/ArmStowAction.h>
#include <owl_msgs/ArmUnstowAction.h>
#include <owl_msgs/TaskDeliverSampleAction.h>
#include <owl_msgs/PanTiltMoveJointsAction.h>
#include <owl_msgs/CameraCaptureAction.h>
#include <owl_msgs/FaultClearAction.h>

// owl_msgs - telemetry
#include <owl_msgs/PanTiltPosition.h>
#include <owl_msgs/ArmFaultsStatus.h>
#include <owl_msgs/PanTiltFaultsStatus.h>
#include <owl_msgs/PowerFaultsStatus.h>
#include <owl_msgs/CameraFaultsStatus.h>
#include <owl_msgs/SystemFaultsStatus.h>
#include <owl_msgs/ArmJointAccelerations.h>
#include <owl_msgs/ArmJointVelocities.h>
#include <owl_msgs/ArmJointTorques.h>
#include <owl_msgs/ArmJointPositions.h>
#include <owl_msgs/ArmPose.h>
#include <owl_msgs/BatteryRemainingUsefulLife.h>
#include <owl_msgs/BatteryStateOfCharge.h>
#include <owl_msgs/BatteryTemperature.h>

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

// Fault Dependencies
#include "FaultDependencies.h"

// C++
#include <string>
#include <memory>
#include <inttypes.h> // for int64 support


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
using TaskDeliverSampleActionClient =
  actionlib::SimpleActionClient<owl_msgs::TaskDeliverSampleAction>;
using PanTiltMoveJointsActionClient =
  actionlib::SimpleActionClient<owl_msgs::PanTiltMoveJointsAction>;
using CameraCaptureActionClient =
  actionlib::SimpleActionClient<owl_msgs::CameraCaptureAction>;
using FaultClearActionClient =
  actionlib::SimpleActionClient<owl_msgs::FaultClearAction>;

// Maps from specific fault names to the pair:
//    <fault value, fault active?>.
// PLEXIL can support Lookups on the specific fault name.  In Release
// 12, only AntennaPanFault and AntennaTiltFault are supported.

using FaultMap = std::map<std::string,std::pair<uint64_t, bool>>;

class LanderInterface : public PlexilInterface
{
 public:
  LanderInterface();
  virtual ~LanderInterface() = default;
  LanderInterface (const LanderInterface&) = delete;
  LanderInterface& operator= (const LanderInterface&) = delete;

  void initialize();

  // Actions with a shared interface but different implementations in
  // OceanWATERS and OWLAT.

  virtual void armFindSurface (int frame,
                               bool relative,
                               const std::vector<double>& pos,
                               const std::vector<double>& normal,
                               double distance,
                               double overdrive,
                               double force_threshold,
                               double torque_threshold,
                               int id) = 0;

  virtual void taskDiscardSample (int frame, bool relative,
                                  const std::vector<double>& point,
                                  double height, int id) = 0;

  // Actions with identical interface and implementation between
  // OceanWATERS and OWLAT.

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
  void taskDeliverSample (int id);
  void panTiltMoveJoints (double pan_degrees, double tilt_degrees, int id);
  void cameraCapture (int id);
  void faultClear (int fault, int id);

  virtual bool systemFault () const = 0;
  bool antennaFault () const;
  bool antennaPanFault () const;
  bool antennaTiltFault () const;
  bool armFault () const;
  bool powerFault () const;
  bool cameraFault () const;

  // Telemetry
  virtual std::vector<double> getArmEndEffectorFT () const = 0;
  double getTiltRadians () const;
  double getPanRadians () const;
  double getArmJointAcceleration (int index) const;
  double getArmJointPosition (int index) const;
  double getArmJointTorque (int index) const;
  double getArmJointVelocity (int index) const;
  std::vector<double> getArmPose () const;
  double getBatterySOC () const;
  double getBatteryRUL() const;
  double getBatteryTemperature () const;

 protected:

  template <typename T1, typename T2>
    void updateFaultStatus (T1 msg_val, T2&,
                            const std::string& component_name,
                            // Fault's general name as a PLEXIL Lookup name.
                            const std::string& general_name);

  template <typename T>
    bool faultActive (const T& fault_map) const;


  // Fault Hierarchy for OW, needs to be in LanderInterface for use
  // with LanderInterface.tpp.
  std::unique_ptr<FaultDependencies> m_fault_dependencies;
  // Queue size for subscribers is a guess at adequacy.
  const int QueueSize = 3;

 private:

  // Action support
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
  void faultClearAction (int fault, int id);
  void panTiltMoveJointsAction (double pan_degrees, double tilt_degrees, int id);

  // Fault support
  void systemFaultMessageCb (const owl_msgs::SystemFaultsStatus::ConstPtr&);
  void armFaultCb (const owl_msgs::ArmFaultsStatus::ConstPtr&);
  void powerFaultCb (const owl_msgs::PowerFaultsStatus::ConstPtr&);
  void antennaFaultCb (const owl_msgs::PanTiltFaultsStatus::ConstPtr&);
  void cameraFaultCb (const owl_msgs::CameraFaultsStatus::ConstPtr&);

  // Joint support
  void armJointAccelCb (const owl_msgs::ArmJointAccelerations::ConstPtr&);
  void armJointPositionCb (const owl_msgs::ArmJointPositions::ConstPtr&);
  void armJointVelocityCb (const owl_msgs::ArmJointVelocities::ConstPtr&);
  void armJointTorqueCb (const owl_msgs::ArmJointTorques::ConstPtr&);

  // Battery
  void batteryLifeCb (const owl_msgs::BatteryRemainingUsefulLife::ConstPtr&);
  void batteryTempCb (const owl_msgs::BatteryTemperature::ConstPtr&);
  void batteryChargeCb (const owl_msgs::BatteryStateOfCharge::ConstPtr&);

  // Misc
  void armPoseCb (const owl_msgs::ArmPose::ConstPtr&);
  void panTiltCb (const owl_msgs::PanTiltPosition::ConstPtr&);

  FaultMap m_armErrors = {
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

  FaultMap m_powerErrors = {
    {"LOW_STATE_OF_CHARGE", std::make_pair(
        owl_msgs::PowerFaultsStatus::LOW_STATE_OF_CHARGE, false)},
    {"INSTANTANEOUS_CAPACITY_LOSS", std::make_pair(
        owl_msgs::PowerFaultsStatus::INSTANTANEOUS_CAPACITY_LOSS, false)},
    {"THERMAL_FAULT", std::make_pair(
        owl_msgs::PowerFaultsStatus::THERMAL_FAULT, false)}
  };

  FaultMap m_panTiltErrors =
    {
     {"AntennaPanFault",
      std::make_pair(owl_msgs::PanTiltFaultsStatus::PAN_JOINT_LOCKED, false)},
     {"AntennaTiltFault",
      std::make_pair(owl_msgs::PanTiltFaultsStatus::TILT_JOINT_LOCKED, false)}
    };

  FaultMap m_cameraErrors =
    {
     {"NO_IMAGE",
      std::make_pair(owl_msgs::CameraFaultsStatus::NO_IMAGE, false)}
    };

  // Telemetry

  std::vector<double> m_arm_joint_accelerations;
  std::vector<double> m_arm_joint_positions;
  std::vector<double> m_arm_joint_velocities;
  std::vector<double> m_arm_joint_torques;
  std::vector<double> m_arm_pose;
  double m_current_pan_radians;
  double m_current_tilt_radians;
  double m_battery_soc;
  double m_battery_rul;
  double m_battery_temp;

  // Action Clients

  std::unique_ptr<ArmMoveCartesianActionClient> m_armMoveCartesianClient;
  std::unique_ptr<ArmMoveCartesianGuardedActionClient> m_armMoveCartesianGuardedClient;
  std::unique_ptr<ArmMoveJointActionClient> m_armMoveJointClient;
  std::unique_ptr<ArmStopActionClient> m_armStopClient;
  std::unique_ptr<ArmStowActionClient> m_armStowClient;
  std::unique_ptr<ArmUnstowActionClient> m_armUnstowClient;
  std::unique_ptr<TaskDeliverSampleActionClient> m_taskDeliverSampleClient;
  std::unique_ptr<PanTiltMoveJointsActionClient> m_panTiltMoveJointsClient;
  std::unique_ptr<CameraCaptureActionClient> m_cameraCaptureClient;
  std::unique_ptr<FaultClearActionClient> m_faultClearClient;
};

#include "LanderInterface.tpp"

#endif
