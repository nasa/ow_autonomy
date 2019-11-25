#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator, and hopefully in time, the physical testbed.

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
  OwInterface (const OwInterface&);            // undefined, no copying
  OwInterface& operator= (const OwInterface&); // undefined, no assignment
  void checkSubscribers (const ros::Publisher*) const;
  ros::NodeHandle* m_genericNodeHandle;
  ros::Publisher*  m_antennaTiltPublisher;
  ros::Publisher*  m_antennaPanPublisher;
  ros::Publisher*  m_leftImageTriggerPublisher;
};


#endif
