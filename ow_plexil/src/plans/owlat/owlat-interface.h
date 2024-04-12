// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// OWLAT-specific PLEXIL interface (commands, lookups) and definitions.

#ifndef Owlat_Plan_Interface_H
#define Owlat_Plan_Interface_H

#include "../common/common-interface.h"
#include "owlat-commands.h"

// System Fault flags for OWLAT
#define NO_FAULT 0
#define SYSTEM_ERROR 1
#define ARM_GOAL_ERROR 2
#define ARM_EXECUTION_ERROR 4
#define TASK_GOAL_ERROR 8
#define CAMERA_GOAL_ERROR 16
#define CAMERA_EXECUTION_ERROR 32
#define PAN_TILT_GOAL_ERROR 64
#define PAN_TILT_EXECUTION_ERROR 128
#define DRILL_GOAL_ERROR 256
#define DRILL_EXECUTION_ERROR 512
#define LANDER_EXECUTION_ERROR 1024

// OWLAT end effector tool enumeration
#define TOOL_NONE 0
#define TOOL_PSP  1
#define TOOL_BEVAMETER 2
#define TOOL_SCOOP 3
#define TOOL_PENETROMETER 4
#define TOOL_DRILL 5

// Note that in cases where OceanWATERS has an action with the same
// name (but different signature, else it would be in ../common), the
// one here is prefixed "Owlat".

LibraryAction ArmSetTool(In Integer Tool);

LibraryAction ArmTareFTSensor();

LibraryAction OwlatArmMoveCartesianGuarded(In Integer Frame,
                                           In Boolean Relative,
                                           In Real Position[3],
                                           In Real Orientation[4],
                                           In Boolean Retracting,
                                           In Real ForceThreshold,
                                           In Real TorqueThreshold);

LibraryAction OwlatArmMoveJoints(In Boolean Relative,
                                 In Real Angles[7]);

LibraryAction OwlatArmMoveJointsGuarded(In Boolean Relative,
                                        In Real Angles[7],
                                        In Boolean Retracting,
                                        In Real ForceThreshold,
                                        In Real TorqueThreshold);

LibraryAction OwlatTaskScoopCircular (In Integer Frame,
                                      In Boolean Relative,
                                      In Real Point[3],
                                      In Real Normal[3],
                                      In Real Depth,
                                      In Real ScoopAngle);

LibraryAction OwlatTaskScoopLinear (In Integer Frame,
                                    In Boolean Relative,
                                    In Real Point[3],
                                    In Real Normal[3],
                                    In Real Depth,
                                    In Real Length);

LibraryAction PrintNodeStart(In String NodeName);

LibraryAction PrintNodeFinish(In String NodeName);

LibraryAction TaskPSP (In Integer Frame,
		       In Boolean Relative,
		       In Real Point[3],
		       In Real Normal[3],
		       In Real MaxDepth,
                       In Real MaxForce);

LibraryAction TaskShearBevameter (In Integer Frame,
                                  In Boolean Relative,
                                  In Real Point[3],
                                  In Real Normal[3],
                                  In Real Preload,
                                  In Real MaxTorque);

LibraryAction TaskPenetrometer (In Integer Frame,
                                In Boolean Relative,
                                In Real Point[3],
                                In Real Normal[3],
                                In Real MaxDepth,
                                In Real MaxForce);

LibraryAction OwlatFaultClear (In Integer fault);

// Lookups

Boolean Lookup DrillGoalError;
Real Lookup ArmJointAngles;
Real Lookup ArmJointAccelerations;
Real Lookup ArmJointTorques;
Real Lookup ArmJointVelocities;
Real Lookup ArmFTTorque;
Real Lookup ArmFTForce;
Integer Lookup ArmTool;
Real Lookup PSPStopReason;
Real Lookup ShearBevameterStopReason;

// OWLAT-specific system faults
Boolean Lookup DrillGoalError;
Boolean Lookup DrillExecutionError;
Boolean Lookup LanderExecutionError; // fault in Stewart platform



#endif
