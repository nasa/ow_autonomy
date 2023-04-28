// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_commands_H
#define Ow_commands_H

// Lander commands available in OceanWATERS.  These include the common
// ones:

#include "common/common-commands.h"

// and the OceanWATERS-specific ones that follow.

Command camera_set_exposure (Real seconds);

Command pan (Real degrees);
Command tilt (Real degrees);
Command pan_tilt_cartesian (Integer frame, Real x, Real y, Real z);

Command arm_move_joints (Boolean relative,
                         Real angles[6]);

Command arm_move_joints_guarded (Boolean relative,
                                 Real angles[6],
                                 Real force_threshold,
                                 Real torque_threshold);

Command dock_ingest_sample();

Command scoop_circular (Integer frame,
                        Boolean relative,
                        Real x,
                        Real y,
                        Real z,
                        Real depth,
                        Boolean parallel); // true means parallel to lander arm

Command scoop_linear (Integer frame,
                      Boolean relative,
                      Real x,
                      Real y,
                      Real z,
                      Real depth,
                      Real length);

Command grind (Real x,
               Real y,
               Real depth,
               Real length,
               Boolean parallel,
               Real ground_pos);

Command guarded_move (Real x,
                      Real y,
                      Real z,
                      Real dir_x,
                      Real dir_y,
                      Real dir_z,
                      Real search_distance);

// Processes number of images already taken with the stereo camera to
// find the 3d point to sample.  filter_type can either be "Dark" or
// "Brown".  (Dark chooses dark spots, brown chooses brown spots).
Real [3] Command identify_sample_location(Integer num_images, String filter_type);

// Set spotlight intensity
Command light_set_intensity (String side,     // "left" or "right"
			     Real intensity); // 0.0 to 1.0.  0 is off.

#endif
