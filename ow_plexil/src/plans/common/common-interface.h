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

LibraryAction CameraCapture ();

LibraryAction PanTiltMoveJoints (In Real PanDegrees, In Real TiltDegrees);

LibraryAction TaskDeliverSample ();

LibraryAction TaskDiscardSample (In Integer Frame,
                                 In Boolean Relative,
                                 In Real Point[3],
                                 In Real Height);

// Convenience
LibraryAction StowSequence();


// Lander queries and telemetry

// Antenna
Real Lookup PanRadians;
Real Lookup PanDegrees;
Real Lookup TiltRadians;
Real Lookup TiltDegrees;

// Arm Joints
Real Lookup ArmJointAcceleration (Integer joint);
Real Lookup ArmJointVelocity     (Integer joint);
Real Lookup ArmJointPosition     (Integer joint);
Real Lookup ArmJointTorque       (Integer joint);

// Battery
Real Lookup BatteryStateOfCharge;
Real Lookup BatteryRemainingUsefulLife;
Real Lookup BatteryTemperature;

// Misc

Real[6] Lookup ArmEndEffectorForceTorque;
Real[7] Lookup ArmPose;
Boolean Lookup UsingOceanWATERS;
Boolean Lookup UsingOWLAT;

// Query whether a given operation is running.  Uses the operation names as
// defined in OwInterface.cpp.  Generally not needed, but supports more
// fine-grained control of concurrency.
Boolean Lookup Running (String operation_name);

// Query the goal status of the ROS action corresponding to a given
// library action.
Integer Lookup ActionGoalStatus (String action_name);

// Function
Boolean Lookup AnglesEquivalent (Real deg1, Real deg2, Real tolerance);


//////// PLEXIL Utilities

// Predefined, PLEXIL variable for current time.
Real Lookup time;

// PLEXIL string operations: see
// $PLEXIL_HOME/examples/checkpoint/StringAdapter.cc for details, and
// many more lookups that could be added here if needed.

String  Lookup ToString (...);
Boolean Lookup StringToBoolean (String);
Integer Lookup StringToInteger (String);
Integer Lookup StringToReal (String);
String  Lookup substr (...);
Integer Lookup find_first_of (...);
Integer Lookup find_last_of (...);

// PLEXIL checkpointing interface.
// For details see:
// https://plexil-group.github.io/plexil_docs/PLEXILExecution/PlanPersistenceandCheckpoints.html

// Overrides any existing checkpoint, returns the previous state of
// the checkpoint (Unknown if checkpoint did not exist)
Boolean set_checkpoint (String name, Boolean value=true, String info="");

// Flushes all changes (including set_boot_ok) to disk.
Boolean flush_checkpoints ();

// Sets IsBootOK, returns previous value
Integer set_boot_ok (Boolean state=True, Integer boot=0);

// Returns the total number of boots ever logged.
Integer NumberOfTotalBoots ();

// Returns number of boot entries available to the plan.
Integer NumberOfAccessibleBoots ();

// Returns number of boot entries which are not marked as is_ok.
Integer NumberOfUnhandledBoots ();

// Returns true if IsOK(1)==false, false otherwise. A true value
// indicates that the executive shut down without setting is_ok.
Boolean DidCrash ();

// If a lookup attempts to query a boot that doesn't exist, Unknown is
// returned.

// Returns the is_ok flag in the selected boot, which starts off as
// false when the boot begins and may be affected by the OKOnExit
// configuration.
Boolean IsBootOK (Integer boot=0);

// Returns the time that CheckpointAdapter.start() is called by the
// interface adapter, Unknown if time not available.
Integer TimeOfBoot (Integer boot=0);

// Returns the time of the last save to disk.
Integer TimeOfLastSave (Integer boot=0);

// If a lookup attempts to query a checkpoint that doesn't exist,
// Unknown is returned.

// If checkpoint not set, Unknown.
Boolean CheckpointState (String name, Integer boot=0);

// Returns last modified time.
Real CheckpointTime (String name, Integer boot=0);

// User defined string.
String CheckpointInfo (String name, Integer boot=0);

// Return the most recent boot number where the checkpoint has been
// set, Unknown if none.
Integer CheckpointWhen (String name);

#endif
