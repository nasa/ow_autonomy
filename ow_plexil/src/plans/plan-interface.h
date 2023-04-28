// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plan_Interface_H
#define Ow_Plan_Interface_H

// OceanWATERS-specific PLEXIL interface (commands, lookups) and definitions.

#include "common/common-interface.h"
#include "lander-commands.h"

// Reference frames
#define BASE_LINK 0
#define SCOOP_TIP 1

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


// PLEXIL library for lander operations.

LibraryAction LightSetIntensity (In String Side, In Real Intensity);
LibraryAction Pan (In Real Degrees);
LibraryAction Tilt (In Real Degrees);
LibraryAction PanTiltMoveCartesian  (In Integer Frame,
                                     In Real X, In Real Y, In Real Z);
LibraryAction GuardedMove (In Real X,
                           In Real Y,
                           In Real Z,
                           In Real DirX,
                           In Real DirY,
                           In Real DirZ,
                           In Real SearchDistance);

LibraryAction ArmMoveJoints (In Boolean Relative,
                             In Real Angles[6]);

LibraryAction ArmMoveJointsGuarded (In Boolean Relative,
                                    In Real Angles[6],
                                    In Real ForceThreshold,
                                    In Real TorqueThreshold);

LibraryAction DockIngestSample ();

LibraryAction TaskGrind (In Real X,
                         In Real Y,
                         In Real Depth,
                         In Real Length,
                         In Boolean Parallel,
                         In Real GroundPos);

LibraryAction TaskScoopCircular (In Integer Frame,
                                 In Boolean Relative,
                                 In Real X,
                                 In Real Y,
                                 In Real Z,
                                 In Real Depth,
                                 In Boolean Parallel);

LibraryAction TaskScoopLinear (In Integer Frame,
                               In Boolean Relative,
                               In Real X,
                               In Real Y,
                               In Real Z,
                               In Real Depth,
                               In Real Length);

LibraryAction CameraSetExposure (In Real Seconds);
LibraryAction CameraCapture ();

// Lander queries

Real Lookup StateOfCharge;
Real Lookup RemainingUsefulLife;
Real Lookup BatteryTemperature;
Boolean Lookup HardTorqueLimitReached (String joint_name);
Boolean Lookup SoftTorqueLimitReached (String joint_name);

// Faults
Boolean Lookup SystemFault;
Boolean Lookup AntennaFault;
Boolean Lookup AntennaPanFault;
Boolean Lookup AntennaTiltFault;
Boolean Lookup ArmFault;
Boolean Lookup PowerFault;
Boolean Lookup CameraFault;

// Relevant with GuardedMove only:
Boolean Lookup GroundFound;
Real    Lookup GroundPosition;

#endif
