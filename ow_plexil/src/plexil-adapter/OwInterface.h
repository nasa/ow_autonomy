// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator.  Singleton, because only once instance will
// ever be needed in the current autonomy scheme, which has one autonomy
// executive per lander.

// ow_plexil
#include "PlexilInterface.h"
#include "joint_support.h"
#include <ow_plexil/IdentifyLocationAction.h>

// ow_simulator
#include <owl_msgs/SystemFaultsStatus.h>
#include <owl_msgs/ArmJointAccelerations.h>
#include <ow_faults_detection/ArmFaults.h>
#include <ow_faults_detection/PowerFaults.h>
#include <ow_faults_detection/PTFaults.h>

// ow_simulator (ROS Actions)
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <owl_msgs/ArmFindSurfaceAction.h>
#include <owl_msgs/ArmMoveCartesianAction.h>
#include <owl_msgs/ArmUnstowAction.h>
#include <owl_msgs/ArmStowAction.h>
#include <ow_lander/GrindAction.h>
#include <ow_lander/GuardedMoveAction.h>
#include <ow_lander/ArmMoveJointAction.h>
#include <ow_lander/ArmMoveJointsAction.h>
#include <ow_lander/DigCircularAction.h>
#include <ow_lander/DigLinearAction.h>
#include <ow_lander/DeliverAction.h>
#include <ow_lander/AntennaPanTiltAction.h>
#include <ow_lander/DiscardAction.h>
#include <ow_lander/CameraCaptureAction.h>
#include <ow_lander/CameraSetExposureAction.h>
#include <ow_lander/LightSetIntensityAction.h>

// ROS
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>

// C++
#include <string>
#include <memory>

using ArmFindSurfaceActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmFindSurfaceAction>;
using ArmMoveCartesianActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveCartesianAction>;
using ArmUnstowActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmUnstowAction>;
using ArmStowActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmStowAction>;
using GrindActionClient =
  actionlib::SimpleActionClient<ow_lander::GrindAction>;
using GuardedMoveActionClient =
  actionlib::SimpleActionClient<ow_lander::GuardedMoveAction>;
using ArmMoveJointActionClient =
  actionlib::SimpleActionClient<ow_lander::ArmMoveJointAction>;
using ArmMoveJointsActionClient =
  actionlib::SimpleActionClient<ow_lander::ArmMoveJointsAction>;
using DigCircularActionClient =
  actionlib::SimpleActionClient<ow_lander::DigCircularAction>;
using DigLinearActionClient =
  actionlib::SimpleActionClient<ow_lander::DigLinearAction>;
using DeliverActionClient =
  actionlib::SimpleActionClient<ow_lander::DeliverAction>;
using PanTiltActionClient =
  actionlib::SimpleActionClient<ow_lander::AntennaPanTiltAction>;
using DiscardActionClient =
  actionlib::SimpleActionClient<ow_lander::DiscardAction>;
using CameraCaptureActionClient =
  actionlib::SimpleActionClient<ow_lander::CameraCaptureAction>;
using CameraSetExposureActionClient =
  actionlib::SimpleActionClient<ow_lander::CameraSetExposureAction>;
using LightSetIntensityActionClient =
  actionlib::SimpleActionClient<ow_lander::LightSetIntensityAction>;

// The only ow_plexil-defined action.
using IdentifySampleLocationActionClient =
  actionlib::SimpleActionClient<ow_plexil::IdentifyLocationAction>;


// Maps from fault name to the pair (fault value, is fault in progress?)
using FaultMap32 = std::map<std::string,std::pair<uint32_t, bool>>;
using FaultMap64 = std::map<std::string,std::pair<uint64_t, bool>>;

class OwInterface : public PlexilInterface
{
 public:
  static OwInterface* instance();
  OwInterface ();
  ~OwInterface () = default;
  OwInterface (const OwInterface&) = delete;
  OwInterface& operator= (const OwInterface&) = delete;
  void initialize ();

  // Operational interface

  void armFindSurface (int frame, bool relative,
                       double pos_x, double pos_y, double z,
                       double norm_x, double norm_y, double norm_z,
                       double distance, double overdrive,
                       double force_threshold, double torque_threshold, int id);
  void armMoveCartesian (int frame, bool relative,
			 double x, double y, double z,
			 double orient_x, double orient_y,
			 double orient_z, int id);
  void armMoveCartesian (int frame, bool relative,
			 double x, double y, double z,
			 double orient_x, double orient_y,
			 double orient_z, double orient_w, int id);
  void guardedMove (double x, double y, double z,
                    double direction_x, double direction_y, double direction_z,
                    double search_distance, int id);
  void armMoveJoint (bool relative, int joint, double angle, int id);
  void armMoveJoints (bool relative, const std::vector<double>& angles, int id);
  std::vector<double> identifySampleLocation (int num_images,
                                              const std::string& filter_type,
                                              int id);

  void panTiltAntenna (double pan_degrees, double tilt_degrees, int id);
  void cameraCapture (int id);
  void cameraSetExposure (double exposure_secs, int id);
  void digLinear (double x, double y, double depth, double length,
                  double ground_pos, int id);
  void digCircular (double x, double y, double depth,
                    double ground_pos, bool parallel, int id);
  void grind (double x, double y, double depth, double length,
              bool parallel, double ground_pos, int id);
  void armStow (int id);
  void armUnstow (int id);
  void deliver (int id);
  void discard (double x, double y, double z, int id);
  void lightSetIntensity (const std::string& side, double intensity, int id);

  // State/Lookup interface
  double getTiltRadians () const;
  double getTiltDegrees () const;
  double getPanRadians () const;
  double getPanDegrees () const;
  double getPanVelocity () const;
  double getTiltVelocity () const;
  double getStateOfCharge () const;
  double getRemainingUsefulLife () const;
  double getBatteryTemperature () const;
  bool   groundFound () const;
  double groundPosition () const;
  bool   systemFault () const;
  bool   antennaFault () const;
  bool   armFault () const;
  bool   powerFault () const;
  bool   anglesEquivalent (double deg1, double deg2, double tolerance);
  bool   hardTorqueLimitReached (const std::string& joint_name) const;
  bool   softTorqueLimitReached (const std::string& joint_name) const;
  double jointTelemetry (int joint, TelemetryType type) const;
  int    actionGoalStatus (const std::string& action_name) const;

 private:
  void addSubscriber (const std::string& topic, const std::string& operation);
  void armFindSurfaceAction (int frame, bool relative,
                             const geometry_msgs::Point& pos,
                             const geometry_msgs::Vector3& normal,
                             double distance, double overdrive,
                             double force_threshold, double torque_threshold,
                             int id);
  void armMoveCartesianAux (int frame, bool relative,
                            double x, double y, double z,
                            const geometry_msgs::Quaternion& q, int id);
  void armMoveCartesianAction (int frame,
                               bool relative,
                               const geometry_msgs::Pose& pose,
                               int id);
  void armStowAction (int id);
  void armUnstowAction (int id);
  void grindAction (double x, double y, double depth, double length,
                    bool parallel, double ground_pos, int id);
  void guardedMoveAction (double x, double y, double z,
                          double dir_x, double dir_y, double dir_z,
                          double search_distance, int id);
  void armMoveJointAction (bool relative, int joint,
                           double angle, int id);
  void armMoveJointsAction (bool relative, const std::vector<double>& angles,
                            int id);
  void identifySampleLocationAction (int num_images,
                                     const std::string& filter_type, int id);
  void digCircularAction (double x, double y, double depth,
                          double ground_pos, bool parallel, int id);
  void digLinearAction (double x, double y, double depth, double length,
                        double ground_pos, int id);
  void panTiltAntennaAction (double pan_degrees, double tilt_degrees, int id);
  void deliverAction (int id);
  void discardAction (double x, double y, double z, int id);
  void cameraCaptureAction (int id);
  void cameraSetExposureAction (double exposure_secs, int id);
  void lightSetIntensityAction (const std::string& side, double intensity, int id);
  void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);
  void armJointAccelerationsCb (const owl_msgs::ArmJointAccelerations::ConstPtr&);
  void systemFaultMessageCallback (const owl_msgs::SystemFaultsStatus::ConstPtr&);
  void armFaultCallback (const ow_faults_detection::ArmFaults::ConstPtr&);
  void powerFaultCallback (const ow_faults_detection::PowerFaults::ConstPtr&);
  void antennaFaultCallback (const ow_faults_detection::PTFaults::ConstPtr&);
  void antennaOp (const std::string& opname, double degrees,
                  std::unique_ptr<ros::Publisher>&, int id);
  void actionGoalStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr&,
                                 const std::string);

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

  FaultMap32 m_armErrors = {
    {"HARDWARE_ERROR", std::make_pair(1, false)},
    {"TRAJECTORY_GENERATION_ERROR", std::make_pair(2, false)},
    {"COLLISION_ERROR", std::make_pair(3, false)},
    {"ESTOP_ERROR", std::make_pair(4, false)},
    {"POSITION_LIMIT_ERROR", std::make_pair(5, false)},
    {"TORQUE_LIMIT_ERROR", std::make_pair(6, false)},
    {"VELOCITY_LIMIT_ERROR", std::make_pair(7, false)},
    {"NO_FORCE_DATA_ERROR", std::make_pair(8, false)}
  };

  FaultMap32 m_powerErrors = {
    {"HARDWARE_ERROR", std::make_pair(1, false)}
  };

  FaultMap32 m_panTiltErrors = {
    {"HARDWARE_ERROR", std::make_pair(1, false)},
    {"JOINT_LIMIT_ERROR", std::make_pair(2, false)}
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
  std::unique_ptr<ArmMoveCartesianActionClient> m_armMoveCartesianClient;
  std::unique_ptr<GuardedMoveActionClient> m_guardedMoveClient;
  std::unique_ptr<ArmMoveJointActionClient> m_armMoveJointClient;
  std::unique_ptr<ArmMoveJointsActionClient> m_armMoveJointsClient;
  std::unique_ptr<ArmUnstowActionClient> m_armUnstowClient;
  std::unique_ptr<ArmStowActionClient> m_armStowClient;
  std::unique_ptr<GrindActionClient> m_grindClient;
  std::unique_ptr<DigCircularActionClient> m_digCircularClient;
  std::unique_ptr<DigLinearActionClient> m_digLinearClient;
  std::unique_ptr<DeliverActionClient> m_deliverClient;
  std::unique_ptr<PanTiltActionClient> m_panTiltClient;
  std::unique_ptr<DiscardActionClient> m_discardClient;
  std::unique_ptr<CameraCaptureActionClient> m_cameraCaptureClient;
  std::unique_ptr<CameraSetExposureActionClient> m_cameraSetExposureClient;
  std::unique_ptr<LightSetIntensityActionClient> m_lightSetIntensityClient;

  std::unique_ptr<IdentifySampleLocationActionClient> m_identifySampleLocationClient;

  // Antenna state
  double m_currentPanRadians, m_currentTiltRadians;
};

#endif
