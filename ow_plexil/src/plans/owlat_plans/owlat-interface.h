// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Owlat_Plan_Interface_H
#define Owlat_Plan_Interface_H

// PLEXIL interface to lander: commands, lookups, library plans, PLEXIL utilities
#include "../common/common-interface.h"
#include "owlat-lander-commands.h"

// PLEXIL library for lander operations specific to OWLAT.

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

LibraryAction OwlatArmPlaceTool(In Integer Frame,
                                In Boolean Relative,
                                In Real Position[3],
                                In Real Normal[3],
                                In Real Distance,
                                In Real Overdrive,
                                In Boolean Retracting,
                                In Real ForceThreshold,
                                In Real TorqueThreshold);

LibraryAction ArmSetTool(In Integer Tool);

LibraryAction ArmTareFTSensor();

LibraryAction OwlatTaskDropoff(In Integer Frame,
                               In Boolean Relative,
                               In Real Point[3]);

LibraryAction OwlatTaskPSP(In Integer Frame,
                           In Boolean Relative,
                           In Real Point[3],
                           In Real Normal[3],
                           In Real MaxDepth,
                           In Real MaxForce);

LibraryAction OwlatTaskScoop(In Integer Frame,
                             In Boolean Relative,
                             In Real Point[3],
                             In Real Normal[3]);

LibraryAction OwlatTaskShearBevameter(In Integer Frame,
                                      In Boolean Relative,
                                      In Real Point[3],
                                      In Real Normal[3],
                                      In Real Preload,
                                      In Real MaxTorque);

LibraryAction PrintNodeStart(In String NodeName);

LibraryAction PrintNodeFinish(In String NodeName);


// Telemetry

Real Lookup ArmJointAngles;
Real Lookup ArmJointAccelerations;
Real Lookup ArmJointTorques;
Real Lookup ArmJointVelocities;
Real Lookup ArmFTTorque;
Real Lookup ArmFTForce;
Real Lookup ArmTool;
Real Lookup PSPStopReason;
Real Lookup ShearBevameterStopReason;


#endif
