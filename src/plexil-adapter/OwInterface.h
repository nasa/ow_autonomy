#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator, and hopefully in time, the physical testbed.
// This class is a singleton because only once instance will ever be needed, in
// the current overall autonomy scheme.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include <ros/ros.h>

class OwInterface
{
 public:
  static OwInterface* instance();
  OwInterface ();
  ~OwInterface ();
  void initialize ();

  // Temporary "demo" functions
  void startPlanningDemo();
  void moveGuardedDemo();
  void publishTrajectoryDemo();

  void tiltAntenna (double);
  void panAntenna (double);
  void takePicture ();

 private:
  OwInterface (const OwInterface&);            // undefined (singleton)
  OwInterface& operator= (const OwInterface&); // undefined (singleton)
  void checkSubscribers (const ros::Publisher*) const;
  static OwInterface* m_instance;
  ros::NodeHandle* m_genericNodeHandle;
  ros::Publisher*  m_antennaTiltPublisher;
  ros::Publisher*  m_antennaPanPublisher;
  ros::Publisher*  m_leftImageTriggerPublisher;
};


#endif
