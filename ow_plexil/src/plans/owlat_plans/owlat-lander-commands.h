// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Owlat_commands_H
#define Owlat_commands_H

// All available PLEXIL commands in OWLAT to the lander.  This is essentially the
// lander's command interface.

#include "../common/common-commands.h"

// Note that PLEXIL commands are asynchronous by design.  In general, autonomy
// plans should not call these directly, but rather user the Library interface
// that wraps these commands; this is defined in plan-interface.h.

// More in depth explanation of these Commands can be found in the OWLAT user guide.

// Move all joints to specified angles.
Command owlat_arm_move_joints (Boolean relative, Real angles[7]);

// Guarded version of owlat_arm_move_joints.
Command owlat_arm_move_joints_guarded(Boolean relative, Real angles[7],
                                      Boolean retracting, Real force_threshold,
                                      Real torque_threshold);

// Perform a guarded pose move to the specified position moving along the given normal
// vector until meeting a force threshold.
Command owlat_arm_place_tool(Integer Frame, Boolean relative, Real position[3], Real normal[3],
                             Real distance, Real overdrive, Boolean retracting, Real force_threshold,
                             Real torque_threshold);

// Perform scoop dropoff over target point in a given frame.
Command owlat_task_dropoff(Integer frame, Boolean relative, Real point[3]);

// Perform pressure sinkage plate operation at the specified point with a given surface
// normal, until achieving a target force or depth.
Command owlat_task_psp(Integer frame, Boolean relative, Real point[3],
                       Real normal[3], Real max_depth, Real max_force);

// Perform a scoop motion at a target in the given frame.
Command owlat_task_scoop(Integer frame, Boolean relative, Real point[3],
                         Real normal[3]);

// Task for Shear Bevameter, more info in user guide.
Command owlat_task_shear_bevameter(Integer frame, Boolean relative, Real point[3],
                                   Real normal[3], Real preload, Real max_torque);


#endif
