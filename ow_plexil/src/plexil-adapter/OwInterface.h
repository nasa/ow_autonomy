// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator.  Singleton, because only once instance will
// ever be needed in the current autonomy scheme, which has one autonomy
// executive per lander.

#include <memory>
#include <ros/ros.h>

// ROS Actions - OceanWATERS
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <ow_lander/UnstowAction.h>
#include <ow_lander/StowAction.h>
#include <ow_lander/GrindAction.h>
#include <ow_lander/GuardedMoveAction.h>
#include <ow_lander/ArmMoveJointAction.h>
#include <ow_lander/ArmMoveJointsAction.h>
#include <ow_lander/DigCircularAction.h>
#include <ow_lander/DigLinearAction.h>
#include <ow_lander/DeliverAction.h>
#include <ow_lander/DiscardAction.h>
#include <ow_plexil/IdentifyLocationAction.h>

#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <string>

#include <ow_faults_detection/SystemFaults.h>
#include <ow_faults_detection/ArmFaults.h>
#include <ow_faults_detection/PowerFaults.h>
#include <ow_faults_detection/PTFaults.h>

#include "PlexilInterface.h"

using UnstowActionClient =
  actionlib::SimpleActionClient<ow_lander::UnstowAction>;
using StowActionClient =
  actionlib::SimpleActionClient<ow_lander::StowAction>;
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
using DiscardActionClient =
  actionlib::SimpleActionClient<ow_lander::DiscardAction>;
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

  void guardedMove (double x, double y, double z,
                    double direction_x, double direction_y, double direction_z,
                    double search_distance, int id);
  void armMoveJoint (bool relative, int joint, double angle,
                     int id);
  void armMoveJoints (bool relative,
                      const std::vector<double>& angles,
                      int id); 
  std::vector<double> identifySampleLocation (int num_images,
                                              const std::string& filter_type,
                                              int id);
  void tiltAntenna (double degrees, int id);
  void panAntenna (double degrees, int id);
  void takePicture (int id);
  void digLinear (double x, double y, double depth, double length,
                  double ground_pos, int id);
  void digCircular (double x, double y, double depth,
                    double ground_pos, bool parallel, int id);
  void grind (double x, double y, double depth, double length,
              bool parallel, double ground_pos, int id);
  void stow (int id);
  void unstow (int id);
  void deliver (int id);
  void discard (double x, double y, double z, int id);
  void setLightIntensity (const std::string& side, double intensity, int id);

  // State/Lookup interface
  double getTilt () const;
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

  bool hardTorqueLimitReached (const std::string& joint_name) const;
  bool softTorqueLimitReached (const std::string& joint_name) const;

  int actionGoalStatus (const std::string& action_name) const;

 private:
  template<typename Service>
  void callService (ros::ServiceClient, Service, std::string name, int id);

  void unstowAction (int id);
  void stowAction (int id);
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
  void deliverAction (int id);
  void discardAction (double x, double y, double z, int id);
  void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);
  void cameraCallback (const sensor_msgs::Image::ConstPtr&);
  void pointCloudCallback (const sensor_msgs::PointCloud2::ConstPtr&);
  void managePanTilt (const std::string& opname,
                      double current, double goal,
                      const ros::Time& start);
  void systemFaultMessageCallback (const ow_faults_detection::SystemFaults::ConstPtr&);
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

  FaultMap64 m_systemErrors =
  {
    {"ARM_EXECUTION_ERROR", std::make_pair(4,false)},
    {"POWER_EXECUTION_ERROR", std::make_pair(512,false)},
    {"PT_EXECUTION_ERROR", std::make_pair(128,false)}
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

  std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;

  // Publishers and subscribers

  std::unique_ptr<ros::Publisher> m_antennaTiltPublisher;
  std::unique_ptr<ros::Publisher> m_antennaPanPublisher;
  std::unique_ptr<ros::Publisher> m_leftImageTriggerPublisher;

  std::unique_ptr<ros::Subscriber> m_jointStatesSubscriber;
  std::unique_ptr<ros::Subscriber> m_cameraSubscriber;
  std::unique_ptr<ros::Subscriber> m_pointCloudSubscriber;
  std::unique_ptr<ros::Subscriber> m_socSubscriber;
  std::unique_ptr<ros::Subscriber> m_rulSubscriber;
  std::unique_ptr<ros::Subscriber> m_batteryTempSubscriber;
  std::unique_ptr<ros::Subscriber> m_systemFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_armFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_powerFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_ptFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_unstowStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_stowStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_grindStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_guardedMoveStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_armMoveJointStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_armMoveJointsStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_digCircularStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_digLinearStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_deliverStatusSubscriber;
  std::unique_ptr<ros::Subscriber> m_discardStatusSubscriber;

  // Action clients
  std::unique_ptr<GuardedMoveActionClient> m_guardedMoveClient;
  std::unique_ptr<ArmMoveJointActionClient> m_armMoveJointClient;
  std::unique_ptr<ArmMoveJointsActionClient> m_armMoveJointsClient;
  std::unique_ptr<UnstowActionClient> m_unstowClient;
  std::unique_ptr<StowActionClient> m_stowClient;
  std::unique_ptr<GrindActionClient> m_grindClient;
  std::unique_ptr<DigCircularActionClient> m_digCircularClient;
  std::unique_ptr<DigLinearActionClient> m_digLinearClient;
  std::unique_ptr<DeliverActionClient> m_deliverClient;
  std::unique_ptr<DiscardActionClient> m_discardClient;
  std::unique_ptr<IdentifySampleLocationActionClient> m_identifySampleLocationClient;

  // Antenna state - note that pan and tilt can be concurrent.
  double m_currentPan, m_currentTilt;
  double m_goalPan, m_goalTilt;      // commanded pan/tilt values
  bool m_pointCloudRecieved;
  ros::Time m_panStart, m_tiltStart; // pan/tilt start times
};

#endif
