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
LibraryAction OwlatArmMoveCartesianGuarded(In Integer Frame,In Boolean Relative, 
                                           In Real Position[3], In Real Orientation[4],
                                           In Boolean Retracting, In Real ForceThreshold,
                                           In Real TorqueThreshold); 
LibraryAction OwlatArmMoveJoint(In Boolean Relative, In Integer Joint, In Real Angle);
LibraryAction OwlatArmMoveJoints(In Boolean Relative, In Real Angles[7]);
LibraryAction OwlatArmMoveJointsGuarded(In Boolean Relative, In Real Angles[7],
                                        In Boolean Retracting, In Real ForceThreshold,
                                        In Real TorqueThreshold);
LibraryAction OwlatArmPlaceTool(In Integer Frame, In Boolean Relative, In Real Position[3], In Real Normal[3],
                                In Real Distance, In Real Overdrive, In Boolean Retracting, In Real ForceThreshold,
                                In Real TorqueThreshold);
LibraryAction OwlatArmSetTool(In Integer Tool);
LibraryAction OwlatArmStop();
LibraryAction OwlatArmTareFS();
LibraryAction OwlatTaskDropoff(In Integer Frame, In Boolean Relative, In Real Point[3]);
LibraryAction OwlatTaskPSP(In Integer Frame, In Boolean Relative, In Real Point[3],
                           In Real Normal[3], In Real MaxDepth, In Real MaxForce);
LibraryAction OwlatTaskScoop(In Integer Frame, In Boolean Relative, In Real Point[3], In Real Normal[3]);
LibraryAction OwlatTaskShearBevameter(In Integer Frame, In Boolean Relative, In Real Point[3],
                                      In Real Normal[3], In Real Preload, In Real MaxTorque);
LibraryAction PrintNodeStart(In String NodeName);
LibraryAction PrintNodeFinish(In String NodeName);

Real Lookup ArmJointAngles;
Real Lookup ArmJointAccelerations;
Real Lookup ArmJointTorques;
Real Lookup ArmJointVelocities;
Real Lookup ArmFTTorque;
Real Lookup ArmFTForce;
Real Lookup ArmPose;
Real Lookup ArmTool;
Real Lookup PSPStopReason;
Real Lookup ShearBevameterStopReason;

// Misc
// Query whether a given operation is running.  Uses the operation names as
// defined in OwInterface.cpp.  Generally not needed, but supports more
// fine-grained control of concurrency.
Boolean Lookup Running (String operation_name);

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
