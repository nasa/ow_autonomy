// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plan_Interface_H
#define Ow_Plan_Interface_H

// PLEXIL interface: commands, lookups, library plans

#include "lander-commands.h"

// Utility commands; issue ROS_INFO, ROS_WARN, and ROS_ERROR, respectively.
Command log_info (...);
Command log_warning (...);
Command log_error (...);


// PLEXIL library for lander operations.

LibraryAction Stow ();
LibraryAction Unstow ();
LibraryAction GuardedMove (In Real X,
                           In Real Y,
                           In Real Z,
                           In Real DirX,
                           In Real DirY,
                           In Real DirZ,
                           In Real SearchDistance);

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

LibraryAction DeliverSample (In Real X,
                             In Real Y,
                             In Real Z);


// Lander queries

Real Lookup Voltage;
Real Lookup RemainingUsefulLife;
Real Lookup BatteryTemperature;
Boolean Lookup HardTorqueLimitReached (String joint_name);
Boolean Lookup SoftTorqueLimitReached (String joint_name);

// Relevant with GuardedMove only:
Boolean Lookup GroundFound;
Real    Lookup GroundPosition;


// Misc

// Query whether a given operation is running.  Uses the operation names as
// defined in OwInterface.cpp.  Generally not needed, but supports more
// fine-grained control of concurrency.
Boolean Lookup Running (String operation_name);

// Does nothing, useful as placeholder for real plan.
LibraryAction Stub (In String description);

#endif
