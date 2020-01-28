#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator, and hopefully in time, the physical testbed.
// This class is a singleton because only once instance will ever be needed, in
// the current overall autonomy scheme.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>

class OwInterface
{
 public:
  static OwInterface* instance();
  OwInterface ();
  ~OwInterface ();
  OwInterface (const OwInterface&) = delete;
  OwInterface& operator= (const OwInterface&) = delete;
  void initialize ();

  // Temporary "demo" functions
  void startPlanningDemo();
  void moveGuardedDemo();
  void publishTrajectoryDemo();

  // Operational interface
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
  void tiltCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void panCallback (const control_msgs::JointControllerState::ConstPtr& msg);


 private:
  void checkSubscribers (const ros::Publisher*) const;
  static OwInterface* m_instance;
  ros::NodeHandle* m_genericNodeHandle;

  // Publishers and subscribers

  ros::Publisher*  m_antennaTiltPublisher;
  ros::Publisher*  m_antennaPanPublisher;
  ros::Publisher*  m_leftImageTriggerPublisher;

  ros::Subscriber* m_antennaPanSubscriber;
  ros::Subscriber* m_antennaTiltSubscriber;
  ros::Subscriber* m_jointStatesSubscriber;
};


#endif
