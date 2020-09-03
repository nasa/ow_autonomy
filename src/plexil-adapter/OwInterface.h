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
#include <string>

class OwInterface
{
 public:
  static OwInterface* instance();
  OwInterface ();
  ~OwInterface ();
  OwInterface (const OwInterface&) = delete;
  OwInterface& operator= (const OwInterface&) = delete;
  void initialize ();

  // "Demo" functions, perhaps temporary.
  void digCircularDemo();
  void guardedMoveDemo();
  void guardedMoveActionDemo(); // temporary, proof of concept
  void publishTrajectoryDemo();

  // Operational interface

  // The defaults currently match those of the activity.  When all are used,
  // this function matches guardedMoveDemo above.
  void guardedMove (double x = 2,
                    double y = 0,
                    double z = 0.3,
                    double direction_x = 0,
                    double direction_y = 0,
                    double direction_z = 1,
                    double search_distance = 0.5,
                    bool delete_prev_traj = false);

  // Temporary, until guardedMove is just a ROS action.
  void guardedMoveAction (double x = 2,
                          double y = 0,
                          double z = 0.3,
                          double direction_x = 0,
                          double direction_y = 0,
                          double direction_z = 1,
                          double search_distance = 0.5,
                          bool delete_prev_traj = false);

  bool tiltAntenna (double degrees);
  bool panAntenna (double degrees);
  void takePicture ();
  void digLinear (double x, double y, double depth, double length,
                  double ground_position);
  void digCircular (double x, double y, double depth,
                    double ground_position, bool radial);
  void grind (double x, double y, double depth, double length,
              double ground_position);
  void stow();
  void unstow();
  void deliverSample (double x, double y, double z);
  void takePanorama (double elev_lo, double elev_hi,
                     double lat_overlap, double vert_overlap);

  // State interface
  double getTilt () const;
  double getPanDegrees () const;
  double getPanVelocity () const;
  double getTiltVelocity () const;
  bool imageReceived () const;

  // These methods apply to all "lander operations", and hide their ROS
  // implementation, which could be services, actions, or ad hoc messaging.
  bool running (const std::string& name) const;
  bool finished (const std::string& name) const;
  bool hardTorqueLimitReached (const std::string& joint_name) const;
  bool softTorqueLimitReached (const std::string& joint_name) const;

 private:
  // temporary, proof of concept
  void guardedMoveActionAux (double x,
                             double y,
                             double z,
                             double direction_x,
                             double direction_y,
                             double direction_z,
                             double search_distance,
                             bool delete_prev_traj);
  bool operationRunning (const std::string& name) const;
  bool operationFinished (const std::string& name) const;

  void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);
  void tiltCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void panCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void managePanTilt (const std::string& opname,
                      double position, double velocity,
                      double current, double goal,
                      const ros::Time& start);

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

  // Action clients
  actionlib::SimpleActionClient<ow_autonomy::GuardedMoveAction>
    m_guardedMoveClient;

  // Antenna state - note that pan and tilt can be concurrent.
  double m_currentPan, m_currentTilt;
  double m_goalPan, m_goalTilt;      // commanded pan/tilt values
  ros::Time m_panStart, m_tiltStart; // pan/tilt start times
};

#endif
