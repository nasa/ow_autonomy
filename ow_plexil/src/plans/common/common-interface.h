// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Common_Interface_H
#define Ow_Common_Interface_H

// PLEXIL interface shared by OceanWATERS and OWLAT: commands,
// lookups, library plans, PLEXIL utilities

#include "common-commands.h"

#define BASE_FRAME 0
#define TOOL_FRAME 1

// Utility commands; issue ROS_INFO, ROS_WARN, and ROS_ERROR, respectively.
Command log_info (...);
Command log_warning (...);
Command log_error (...);
Command log_debug (...);

LibraryAction ArmStow ();
LibraryAction ArmUnstow ();
LibraryAction ArmStop ();

LibraryAction ArmMoveJoint(In Boolean Relative,
                           In Integer Joint,
                           In Real Angle);


LibraryAction ArmMoveCartesian (In Integer Frame,
                                In Boolean Relative,
                                In Real Position[3],
                                In Real Orientation[3]);

LibraryAction ArmMoveCartesian_Q (In Integer Frame,
                                  In Boolean Relative,
                                  In Real Position[3],
                                  In Real Orientation[4]);

LibraryAction ArmMoveCartesianGuarded (In Integer Frame,
                                       In Boolean Relative,
                                       In Real Position[3],
                                       In Real Orientation[3],
                                       In Boolean Retracting,
                                       In Real ForceThreshold,
                                       In Real TorqueThreshold);

LibraryAction ArmMoveCartesianGuarded_Q (In Integer Frame,
                                         In Boolean Relative,
                                         In Real Position[3],
                                         In Real Orientation[4],
                                         In Boolean Retracting,
                                         In Real ForceThreshold,
                                         In Real TorqueThreshold);

LibraryAction ArmFindSurface (In Integer Frame,
                              In Boolean Relative,
                              In Real Position[3],
                              In Real Normal[3],
                              In Real Distance,
                              In Real Overdrive,
                              In Real ForceThreshold,
                              In Real TorqueThreshold);

// Lander queries and telemetry

// Antenna
Real Lookup PanRadians;
Real Lookup PanDegrees;
Real Lookup TiltRadians;
Real Lookup TiltDegrees;

// Joints
Real Lookup JointAcceleration (Integer joint);
Real Lookup JointVelocity     (Integer joint);
Real Lookup JointPosition     (Integer joint);
Real Lookup JointEffort       (Integer joint);

// Misc

Real[6] Lookup ArmEndEffectorForceTorque;
Real[7] Lookup ArmPose;
Boolean Lookup UsingOceanWATERS;
Boolean Lookup UsingOWLAT;

// Query whether a given operation is running.  Uses the operation names as
// defined in OwInterface.cpp.  Generally not needed, but supports more
// fine-grained control of concurrency.
Boolean Lookup Running (String operation_name);

// Query the goal status of the ROS action corresponding to a given library action
Integer Lookup ActionGoalStatus (String action_name);

// Function
Boolean Lookup AnglesEquivalent (Real deg1, Real deg2, Real tolerance);


//////// PLEXIL Utilities

// Predefined, PLEXIL variable for current time.
Real Lookup time;

// String operations
String Lookup ToString(...);
Boolean Lookup StringToBoolean(String);
Integer Lookup StringToInteger(String);
Integer Lookup StringToReal(String);
String Lookup substr(...);
Integer Lookup find_first_of(...);
Integer Lookup find_last_of(...);

// Checkpointing interface
Command set_checkpoint(String,Boolean,String);
Command flush_checkpoints;
Command set_boot_ok();
Integer Lookup CheckpointWhen(String);
Integer Lookup NumberOfUnhandledBoots;
Boolean Lookup IsBootOK(Integer);
Boolean Lookup DidCrash;

#endif
