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

// Action goal statuses.
// NOTE: the last five are not supported in OceanWATERS, partly
// because it uses ROS's SimpleActionServer.
#define ACTION_INACTIVE -1
#define ACTION_PENDING 0
#define ACTION_ACTIVE 1
#define ACTION_PREEMPTED 2
#define ACTION_SUCCEEDED 3
#define ACTION_ABORTED 4
#define ACTION_REJECTED 5
#define ACTION_PREEMPTING 6
#define ACTION_RECALLING 7
#define ACTION_RECALLED 8
#define ACTION_LOST 9

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
LibraryAction SafeStow();


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

// Faults
Boolean Lookup ArmGoalError;
Boolean Lookup CameraGoalError;
Boolean Lookup PanTiltGoalError;
Boolean Lookup TaskGoalError;

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
// Actual interface:
// Boolean Command set_checkpoint (String name, Boolean value=true, String info="");
// which is declared as:
Boolean Command set_checkpoint (...);

// Flushes all changes (including set_boot_ok) to disk.
Boolean Command flush_checkpoints ();

// Marks the given boot number is_ok.  This command has optional arguments:
//   Command set_boot_ok (Boolean state=True, Integer boot=0);
// But due to a bug in its implementation, it requires at least one of
// these (either one).  The effective way to declare this command is as follows.
Command set_boot_ok (...);

// Returns the total number of boots ever logged.
Integer Lookup NumberOfTotalBoots ();

// Returns number of boot entries available to the plan.
Integer Lookup NumberOfAccessibleBoots ();

// Returns number of boot entries which are not marked as is_ok.
Integer Lookup NumberOfUnhandledBoots ();

// Returns true if IsOK(1)==false, false otherwise. A true value
// indicates that the executive shut down without setting is_ok.
Boolean Lookup DidCrash ();

// If a lookup attempts to query a boot that doesn't exist, Unknown is
// returned.

// Returns the is_ok flag in the selected boot, which starts off as
// false when the boot begins and may be affected by the OKOnExit
// configuration. Actual:
//Boolean Lookup IsBootOK (Integer boot=0);
Boolean Lookup IsBootOK (...);

// Returns the time that CheckpointAdapter.start() is called by the
// interface adapter, Unknown if time not available. Actual:
//Integer Lookup TimeOfBoot (Integer boot=0);
Integer Lookup TimeOfBoot (...);

// Returns the time of the last save to disk. Actual:
//Integer Lookup TimeOfLastSave (Integer boot=0);
Integer Lookup TimeOfLastSave (...);

// If a lookup attempts to query a checkpoint that doesn't exist,
// Unknown is returned.

// If checkpoint not set, Unknown.  Actual:
//Boolean Lookup CheckpointState (String name, Integer boot=0);
Boolean Lookup CheckpointState (...);

// Returns last modified time.  Actual:
//Real Lookup CheckpointTime (String name, Integer boot=0);
Real Lookup CheckpointTime (...);

// User defined string. Actual:
//String Lookup CheckpointInfo (String name, Integer boot=0);
String Lookup CheckpointInfo (...);

// Return the most recent boot number where the checkpoint has been
// set, Unknown if none.
Integer Lookup CheckpointWhen (String name);

#endif
