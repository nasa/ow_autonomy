// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plan_Interface_H
#define Ow_Plan_Interface_H

// PLEXIL interface to lander: commands, lookups, library plans, PLEXIL utilities

#include "lander-commands.h"

// Utility commands; issue ROS_INFO, ROS_WARN, and ROS_ERROR, respectively.
Command log_info (...);
Command log_warning (...);
Command log_error (...);


// PLEXIL library for lander operations.

LibraryAction SetLightIntensity (In String Side,
				 In Real Intensity);
LibraryAction Tilt (In Real Degrees);
LibraryAction Pan  (In Real Degrees);
LibraryAction Stow ();
LibraryAction Unstow ();
LibraryAction GuardedMove (In Real X,
                           In Real Y,
                           In Real Z,
                           In Real DirX,
                           In Real DirY,
                           In Real DirZ,
                           In Real SearchDistance);

LibraryAction ArmMoveJoint (In Boolean Relative,
                            In Integer Joint,
                            In Real Angle);

LibraryAction ArmMoveJoints (In Boolean Relative,
                             In Real Angles[6]);

LibraryAction Grind (In Real X,
                     In Real Y,
                     In Real Depth,
                     In Real Length,
                     In Boolean Parallel,
                     In Real GroundPos);

LibraryAction DigCircular (In Real X,
                           In Real Y,
                           In Real Depth,
                           In Real GroundPos,
                           In Boolean Parallel);

LibraryAction DigLinear (In Real X,
                         In Real Y,
                         In Real Depth,
                         In Real Length,
                         In Real GroundPos);

LibraryAction Deliver ();

LibraryAction Discard (In Real X,
                       In Real Y,
                       In Real Z);

// Lander queries

Real Lookup StateOfCharge;
Real Lookup RemainingUsefulLife;
Real Lookup BatteryTemperature;
Boolean Lookup HardTorqueLimitReached (String joint_name);
Boolean Lookup SoftTorqueLimitReached (String joint_name);

// Faults
Boolean Lookup SystemFault;
Boolean Lookup AntennaFault;
Boolean Lookup ArmFault;
Boolean Lookup PowerFault;

// Relevant with GuardedMove only:
Boolean Lookup GroundFound;
Real    Lookup GroundPosition;
Real    Lookup PanDegrees;
Real    Lookup TiltDegrees;


// Misc

// Query whether a given operation is running.  Uses the operation names as
// defined in OwInterface.cpp.  Generally not needed, but supports more
// fine-grained control of concurrency.
Boolean Lookup Running (String operation_name);

// Query the goal status of the ROS action corresponding to a given library action
Integer Lookup ActionGoalStatus (String action_name);

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
Command set_checkpoint(...);
Command flush_checkpoints;
Command set_boot_ok();
Integer Lookup CheckpointWhen(String);
Integer Lookup NumberOfUnhandledBoots;
Boolean Lookup IsBootOK(Integer);
Boolean Lookup DidCrash;

#endif
