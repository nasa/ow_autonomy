// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator.  Singleton, because only once instance will
// ever be needed in the current autonomy scheme, which has one autonomy
// executive per lander.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ow_autonomy/GuardedMoveAction.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <string>
// #include <utility>

#include <ow_faults/SystemFaults.h>
#include <ow_faults/ArmFaults.h>
#include <ow_faults/PowerFaults.h>
#include <ow_faults/PTFaults.h>

using GuardedMoveActionClient =
  actionlib::SimpleActionClient<ow_autonomy::GuardedMoveAction>;

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

  // The defaults currently match those of the activity.  When all are used,
  // this function matches guardedMoveDemo above.
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
  void deliverSample (double x, double y, double z, int id);
  void takePanorama (double elev_lo, double elev_hi,
                     double lat_overlap, double vert_overlap);

  // Temporary, proof of concept for ROS Actions
  void guardedMoveActionDemo (const geometry_msgs::Point& start,
                              const geometry_msgs::Point& normal,
                              double search_distance,
                              int id);

  // State interface
  double getTilt () const;
  double getPanDegrees () const;
  double getPanVelocity () const;
  double getTiltVelocity () const;
  double getVoltage () const;
  double getRemainingUsefulLife () const;
  bool   groundFound () const;
  double groundPosition () const;

  // Is the given operation (as named in .cpp file) running?
  bool running (const std::string& name) const;

  bool hardTorqueLimitReached (const std::string& joint_name) const;
  bool softTorqueLimitReached (const std::string& joint_name) const;

  // Command feedback
  void setCommandStatusCallback (void (*callback) (int, bool));


 private:
                                           
  // Temporary, support for public version above
  void guardedMoveActionDemo1 (const geometry_msgs::Point& start,
                               const geometry_msgs::Point& normal,
                               double search_distance,
                               int id);

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
  bool checkFaultMessages(std::string fault_component, 
                                        uint32_t msg_val, 
                                        std::string key, 
                                        uint32_t value, 
                                        bool b );

//////////////////// FAULTS FOR SYSTEM LEVEL ////////////////////////
// structure of all maps for faults is the following:
// key = (string) fault name
// value = (pair) ( (int) numberical fault value, booleon of if fault exists)
  std::map<std::string,std::pair<uint64_t, bool>> systemErrors = 
  {
    {"ARM_EXECUTION_ERROR", std::make_pair(4,false)},
    {"POWER_EXECUTION_ERROR", std::make_pair(512,false)},
    {"PT_EXECUTION_ERROR", std::make_pair(128,false)}
  };

  std::map<std::string,std::pair<uint32_t, bool>> armErrors = {
    {"HARDWARE_ERROR", std::make_pair(1, false)},
    {"TRAJECTORY_GENERATION_ERROR", std::make_pair(2, false)},
    {"COLLISION_ERROR", std::make_pair(3, false)},
    {"ESTOP_ERROR", std::make_pair(4, false)},
    {"POSITION_LIMIT_ERROR", std::make_pair(5, false)},
    {"TORQUE_LIMIT_ERROR", std::make_pair(6, false)},
    {"VELOCITY_LIMIT_ERROR", std::make_pair(7, false)},
    {"NO_FORCE_DATA_ERROR", std::make_pair(8, false)}
  };

  std::map<std::string,std::pair<uint32_t, bool>> powerErrors = {
    {"HARDWARE_ERROR", std::make_pair(1, false)}
  };

  std::map<std::string,std::pair<uint32_t, bool>> ptErrors = {
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
  ros::Subscriber* m_guardedMoveSubscriber;

  std::unique_ptr<ros::Subscriber> m_systemFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_armFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_powerFaultMessagesSubscriber;
  std::unique_ptr<ros::Subscriber> m_ptFaultMessagesSubscriber;

  // Action clients
  std::unique_ptr<GuardedMoveActionClient> m_guardedMoveClient;

  // Antenna state - note that pan and tilt can be concurrent.
  double m_currentPan, m_currentTilt;
  double m_goalPan, m_goalTilt;      // commanded pan/tilt values
  ros::Time m_panStart, m_tiltStart; // pan/tilt start times
};

#endif
