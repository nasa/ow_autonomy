// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Hardware_H
#define Ow_Hardware_H

// Interface to JPL's OWLAT (Ocean Worlds Lander Autonomy Tesbed).
// Based on ROS messages/actions defined in owlat_msgs package.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <owlat_msgs/ARM_MOVE_CARTESIANAction.h>

class OwHardware
{
  OwHardware ();
  OwHardware (const OwHardware&) = delete;
  OwHardware& operator= (const OwHardware&) = delete;
  // Using default destructor

  void cartesianArmMove (bool relative,
                         double pos_x, double pos_y, double pos_z,
                         double norm_x, double norm_y, double norm_z,
                         double distance, double overdrive, int id);
  
 private:
  // Action clients
  actionlib::SimpleActionClient<owlat_msgs::ARM_MOVE_CARTESIANAction>
    m_armMoveCartesianClient;
};

#endif
