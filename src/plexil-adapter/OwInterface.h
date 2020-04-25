#ifndef Ow_Interface_H
#define Ow_Interface_H

// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// Interface to lander simulator, and hopefully in time, the physical testbed.
// This class is a singleton because only once instance will ever be needed, in
// the current overall autonomy scheme.

#include <ros/ros.h>
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
  void startPlanningDemo();
  void moveGuardedDemo();
  void publishTrajectoryDemo();

  // Operational interface

  // The defaults currently match those of the activity.  When all are used,
  // this function matches moveGuardedDemo above.
  void moveGuarded (double target_x = 2,
                    double target_y = 0,
                    double target_z = 0.02,
                    double surf_norm_x = 0,
                    double surf_norm_y = 0,
                    double surf_norm_z = 1,
                    double offset_dist = 0.2,
                    double overdrive_dist = 0.2,
                    bool delete_prev_traj = false,
                    bool retract = false);

  void tiltAntenna (double);
  void panAntenna (double);
  void takePicture ();
  void digTrench (double x, double y, double z, double depth,
                  double length, double width, double pitch, double yaw,
                  double dumpx, double dumpy, double dumpz);
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
  void tiltCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void panCallback (const control_msgs::JointControllerState::ConstPtr& msg);


 private:
  // Predicates for ROS services
  bool serviceRunning (const std::string& name) const;
  bool serviceFinished (const std::string& name) const;
  
  static void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);

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
};


#endif
