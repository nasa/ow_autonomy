// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Owlat_Plan_Interface_H
#define Owlat_Plan_Interface_H

// PLEXIL interface to lander: commands, lookups, library plans, PLEXIL utilities
#include "owlat-lander-commands.h"

// Utility commands; issue ROS_INFO, ROS_WARN, and ROS_ERROR, respectively.
Command log_info (...);
Command log_warning (...);
Command log_error (...);


// PLEXIL library for lander operations in OWLAT.
LibraryAction OwlatUnstow();
LibraryAction OwlatStow();
LibraryAction OwlatArmMoveCartesian(In Integer Frame,
                                    In Boolean Relative,
                                    In Real Position[3],
                                    In Real Orientation[4]);
LibraryAction owlatArmMoveCartesianGuarded(In Integer frame,In Boolean relative, 
                                           In Real position[3], In Real orientation[4],
                                           In Boolean retracting, In Real force_threshold,
                                           In Real torque_threshold); 
LibraryAction owlatArmMoveJoint(In Boolean relative, In Integer joint, In Real angle);
LibraryAction owlatArmMoveJoints(In Boolean relative, In Real angles[7]);
LibraryAction owlatArmMoveJointsGuarded(In Boolean relative, In Real angles[7],
                                        In Boolean retracting, In Real force_threshold,
                                        In Real torque_threshold);
LibraryAction owlatArmPlaceTool(In Boolean relative, In Real position[3], In Real normal[3],
                                In Real distance, In Real overdrive, In Real force_threshold,
                                In Real torque_threshold);
LibraryAction owlatArmSetTool(In Integer tool);
LibraryAction owlatArmStop();
LibraryAction owlatArmTareFS();
LibraryAction owlatTaskDropoff(In Integer frame, In Boolean relative, In Real point[3]);
LibraryAction owlatTaskPSP(In Integer frame, In Boolean relative, In Real point[3],
                           In Real normal[3], In Real max_depth, In Real max_force);
LibraryAction owlatTaskScoop(In Integer frame, In Boolean relative, In Real point[3],
                             In Real normal[3]);
LibraryAction owlatTaskShearBevameter(In Integer frame, In Boolean relative, In Real point[3],
                                      In Real normal[3], In Real preload, In Real max_torque);




// Misc
// Query whether a given operation is running.  Uses the operation names as
// defined in OwInterface.cpp.  Generally not needed, but supports more
// fine-grained control of concurrency.
Boolean Lookup Running (String operation_name);

//////// PLEXIL Utilities

// Predefined, unitless PLEXIL variable for current time.
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
