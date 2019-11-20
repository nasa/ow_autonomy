#ifndef Ow_Interface_H
#define Ow_Interface_H

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
  bool initialize ();

  // Temporary "demo" functions
  void startPlanningDemo();
  void moveGuardedDemo();
  void publishTrajectoryDemo();

  void tiltAntenna (double);

 private:
  OwInterface (const OwInterface&);            // undefined, no copying
  OwExecuitve& operator= (const OwExecutive&); // undefined, no assignment
  ros::NodeHandle* m_genericNodeHandle;
  ros::Publisher*  m_antennaTiltPublisher;
};


#endif
