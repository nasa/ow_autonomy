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
  // void downlinkDemo(); 

  // Operational interface

  // The defaults currently match those of the activity.  When all are used,
  // this function matches guardedMoveDemo above.
  void guardedMove (double target_x = 2,
                    double target_y = 0,
                    double surf_norm_x = 0,
                    double surf_norm_y = 0,
                    double surf_norm_z = 1,
                    double offset_dist = 0.2,
                    double overdrive_dist = 0.2,
                    bool delete_prev_traj = false);

  // Temporary, until guardedMove is just a ROS action.
  void guardedMoveAction (double target_x = 2,
                          double target_y = 0,
                          double surf_norm_x = 0,
                          double surf_norm_y = 0,
                          double surf_norm_z = 1,
                          double offset_dist = 0.2,
                          double overdrive_dist = 0.2,
                          bool delete_prev_traj = false);

  bool tiltAntenna (double degrees);
  bool panAntenna (double degrees);
  void takePicture ();
  void digLinear (double x, double y, double depth,
                  double length, double width, double pitch, double yaw,
                  double dumpx, double dumpy, double dumpz);
  void takePanorama (double elev_lo, double elev_hi,
                     double lat_overlap, double vert_overlap);

  void downlinkTarget();
  void downlinkImage();

  // State interface
  double getTilt () const;
  double getPanDegrees () const;
  double getPanVelocity () const;
  double getTiltVelocity () const;
  double getXTarget () const;
  double getYTarget () const;
  double getZTarget () const;
  void updateTarget () const;
  void timeoutTarget () const;
  void requestFwdLink () const;
  bool getWait () const;
  bool getWaitForGroundTimeout() const;
  bool imageReceived () const;

  // These methods apply to all "lander operations", and hide their ROS
  // implementation, which could be services, actions, or ad hoc messaging.
  bool running (const std::string& name) const;
  bool finished (const std::string& name) const;
  bool hardTorqueLimitReached (const std::string& joint_name) const;
  bool softTorqueLimitReached (const std::string& joint_name) const;
  void tiltCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void panCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void stopOperation (const std::string& name) const;

 private:
  // temporary, proof of concept
  void guardedMoveActionAux (double target_x,
                             double target_y,
                             double surf_norm_x,
                             double surf_norm_y,
                             double surf_norm_z,
                             double offset_dist,
                             double overdrive_dist,
                             bool delete_prev_traj);
  bool operationRunning (const std::string& name) const;
  bool operationFinished (const std::string& name) const;

  static void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);

  static OwInterface* m_instance;
  ros::NodeHandle* m_genericNodeHandle;

  // Publishers and subscribers

  ros::Publisher*  m_antennaTiltPublisher;
  ros::Publisher*  m_antennaPanPublisher;
  ros::Publisher*  m_leftImageTriggerPublisher;
  ros::Publisher*  m_groundControlPublisher;
  ros::Publisher*  m_groundControlImagePublisher;
  ros::Publisher*  m_groundRequestPublisher;

  ros::Subscriber* m_antennaPanSubscriber;
  ros::Subscriber* m_antennaTiltSubscriber;
  ros::Subscriber* m_jointStatesSubscriber;
  ros::Subscriber* m_cameraSubscriber;
  ros::Subscriber* m_groundFwdLinkSubscriber;
  ros::Subscriber* m_groundOnboardDecisionSubscriber;

  // Action clients
  actionlib::SimpleActionClient<ow_autonomy::GuardedMoveAction>
    m_guardedMoveClient;

};


#endif
