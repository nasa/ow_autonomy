// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Owlat_commands_H
#define Owlat_commands_H

// Declarations for commands supported by OWLAT, which include the common ones:

#include "../common/common-commands.h"

// and those that are OWLAT-specific, which follow.

// Move all joints to specified angles.
Command owlat_arm_move_joints (Boolean relative, Real angles[7]);

// Guarded version of owlat_arm_move_joints.
Command owlat_arm_move_joints_guarded(Boolean relative, Real angles[7],
                                      Boolean retracting, Real force_threshold,
                                      Real torque_threshold);

// Perform a guarded pose move to the specified position moving along
// the given normal vector until meeting a force threshold.
Command owlat_arm_place_tool(Integer Frame,
			     Boolean relative,
			     Real position[3],
			     Real normal[3],
                             Real distance,
			     Real overdrive,
			     Boolean retracting,
			     Real force_threshold,
                             Real torque_threshold);

// Perform scoop dropoff over target point in a given frame.
Command owlat_task_dropoff(Integer frame, Boolean relative, Real point[3]);

Command task_scoop_linear (Integer frame,
                           Boolean relative,
                           Real point[3],
                           Real normal[3],
                           Real depth,
                           Real length);

Command task_scoop_circular (Integer frame,
                             Boolean relative,
                             Real point[3],
                             Real normal[3],
                             Real depth,
                             Real scoop_angle);

Command task_shear_bevameter(Integer frame,
                             Boolean relative,
                             Real point[3],
                             Real normal[3],
                             Real preload,
                             Real max_torque);

Command task_psp (Integer frame, Boolean relative, Real point[3],
		  Real normal[3], Real max_depth, Real max_force);

Command task_penetrometer (Integer frame, Boolean relative, Real point[3],
                           Real normal[3], Real max_depth, Real max_force);

#endif
