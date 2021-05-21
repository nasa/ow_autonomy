// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator.  Singleton, because only once instance will
// ever be needed in the current autonomy scheme, which has one autonomy
// executive per lander.

#include <ros/ros.h>

// ROS Actions
#include <actionlib/client/simple_action_client.h>
#include <ow_lander/UnstowAction.h>
#include <ow_lander/StowAction.h>
#include <ow_lander/GrindAction.h>
#include <ow_lander/GuardedMoveAction.h>
#include <ow_lander/DigCircularAction.h>
#include <ow_lander/DigLinearAction.h>
#include <ow_lander/DeliverAction.h>

#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <string>

#include <ow_faults/SystemFaults.h>
#include <ow_faults/ArmFaults.h>
#include <ow_faults/PowerFaults.h>
#include <ow_faults/PTFaults.h>

using UnstowActionClient =
  actionlib::SimpleActionClient<ow_lander::UnstowAction>;
using StowActionClient =
  actionlib::SimpleActionClient<ow_lander::StowAction>;
using GrindActionClient =
  actionlib::SimpleActionClient<ow_lander::GrindAction>;
using GuardedMoveActionClient =
  actionlib::SimpleActionClient<ow_lander::GuardedMoveAction>;
using DigCircularActionClient =
  actionlib::SimpleActionClient<ow_lander::DigCircularAction>;
using DigLinearActionClient =
  actionlib::SimpleActionClient<ow_lander::DigLinearAction>;
using DeliverActionClient =
  actionlib::SimpleActionClient<ow_lander::DeliverAction>;

template<int OpIndex, typename T>
  using t_action_done_cb = void (*)(const actionlib::SimpleClientGoalState&,
                                    const T& result_ignored);

template<int OpIndex, typename T>
void default_action_done_cb
(const actionlib::SimpleClientGoalState& state,
 const T& result_ignored);

// Maps from fault name to the pair (fault value, is fault in progress?)
using FaultMap32 = std::map<std::string,std::pair<uint32_t, bool>>;
using FaultMap64 = std::map<std::string,std::pair<uint64_t, bool>>;

class OwInterface
{
 public:
  static OwInterface* instance();
  OwInterface ();
  ~OwInterface ();
  OwInterface (const OwInterface&) = delete;
  OwInterface& operator= (const OwInterface&) = delete;
  void initialize ();

  // Operational interface

  void guardedMove (double x, double y, double z,
                    double direction_x, double direction_y, double direction_z,
                    double search_distance, int id);
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
  void deliver (double x, double y, double z, int id);
  void takePanorama (double elev_lo, double elev_hi,
                     double lat_overlap, double vert_overlap);

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

  // Is the given operation (as named in .cpp file) running?
  bool running (const std::string& name) const;

  bool hardTorqueLimitReached (const std::string& joint_name) const;
  bool softTorqueLimitReached (const std::string& joint_name) const;

  // Command feedback
  void setCommandStatusCallback (void (*callback) (int, bool));


 private:
  template <int OpIndex, class ActionClient, class Goal,
            class ResultPtr, class FeedbackPtr>
    void runAction (const std::string& opname,
                    std::unique_ptr<ActionClient>&,
                    const Goal&, int id,
                    t_action_done_cb<OpIndex, ResultPtr> done_cb =
                    default_action_done_cb<OpIndex, ResultPtr>);
  void unstowAction (int id);
  void stowAction (int id);
  void grindAction (double x, double y, double depth, double length,
               bool parallel, double ground_pos, int id);
  void guardedMoveAction (double x, double y, double z,
                     double direction_x, double direction_y, double direction_z,
                     double search_distance, int id);
  void digCircularAction (double x, double y, double depth,
                     double ground_pos, bool parallel, int id);
  void digLinearAction (double x, double y, double depth, double length,
                   double ground_pos, int id);
  void deliverAction (double x, double y, double z, int id);
  bool operationRunning (const std::string& name) const;
  void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);
  void tiltCallback (const control_msgs::JointControllerState::ConstPtr&);
  void panCallback (const control_msgs::JointControllerState::ConstPtr&);
  void cameraCallback (const sensor_msgs::Image::ConstPtr&);
  void managePanTilt (const std::string& opname,
                      double position, double velocity,
                      double current, double goal,
                      const ros::Time& start);
  void systemFaultMessageCallback (const ow_faults::SystemFaults::ConstPtr&);
  void armFaultCallback (const ow_faults::ArmFaults::ConstPtr&);
  void powerFaultCallback (const ow_faults::PowerFaults::ConstPtr&);
  void antennaFaultCallback (const ow_faults::PTFaults::ConstPtr&);

  template <typename T1, typename T2>
    void faultCallback (T1 msg_val, T2&, const std::string& name);

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

  static OwInterface* m_instance;
  ros::NodeHandle* m_genericNodeHandle;

  // Publishers and subscribers

  ros::Publisher*  m_antennaTiltPublisher;
  ros::Publisher*  m_antennaPanPublisher;
  ros::Publisher*  m_leftImageTriggerPublisher;

  ros::Subscriber* m_antennaPanSubscriber;
  ros::Subscriber* m_antennaTiltSubscriber;
  ros::Subscriber* m_jointStatesSubscriber;
  ros::Subscriber* m_cameraSubscriber;
  ros::Subscriber* m_socSubscriber;
  ros::Subscriber* m_rulSubscriber;
  ros::Subscriber* m_batteryTempSubscriber;
  std::unique_ptr<ros::Subscriber> m_systemFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_armFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_powerFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_ptFaultMessagesSubscriber;

  // Action clients
  std::unique_ptr<GuardedMoveActionClient> m_guardedMoveClient;
  std::unique_ptr<UnstowActionClient> m_unstowClient;
  std::unique_ptr<StowActionClient> m_stowClient;
  std::unique_ptr<GrindActionClient> m_grindClient;
  std::unique_ptr<DigCircularActionClient> m_digCircularClient;
  std::unique_ptr<DigLinearActionClient> m_digLinearClient;
  std::unique_ptr<DeliverActionClient> m_deliverClient;

  // Antenna state - note that pan and tilt can be concurrent.
  double m_currentPan, m_currentTilt;
  double m_goalPan, m_goalTilt;      // commanded pan/tilt values
  ros::Time m_panStart, m_tiltStart; // pan/tilt start times
};

#endif
