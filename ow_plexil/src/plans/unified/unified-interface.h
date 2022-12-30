#ifndef Unified_Interface_H
#define Unified_Interface_H

// Unified interface to OceanWATERS and OWLAT.
// Under construction!

// Utility commands.
// These issue ROS_INFO, ROS_WARN, ROS_ERROR, and ROS_DEBUG, respectively.
Command log_info (...);
Command log_warning (...);
Command log_error (...);
Command log_debug (...);

// Joint names, defined by their indices in the /joint_states ROS message.

#define ANTENNA_PAN    0
#define ANTENNA_TILT   1
#define DISTAL_PITCH   2
#define GRINDER        3
#define HAND_YAW       4
#define PROXIMAL_PITCH 5
#define SCOOP_YAW      6
#define SHOULDER_PITCH 7
#define SHOULDER_YAW   8


////////////// Telemetry //////////////////

// Antenna
Real Lookup PanRadians;
Real Lookup PanDegrees;
Real Lookup TiltRadians;
Real Lookup TiltDegrees;

// Arm
Real Lookup JointAcceleration (Integer joint);
Real Lookup JointVelocity     (Integer joint);
Real Lookup JointPosition     (Integer joint);
Real Lookup JointEffort       (Integer joint);

#endif
