// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_commands_H
#define Ow_commands_H

// All available PLEXIL commands to the lander.  This is essentially the
// lander's command interface.

// Note that in PLEXIL, commands are asynchronous by design.
// Typically, OceanWATERS plans should not call the following commands
// directly, but instead use the Library interface (defined in
// plan-interface.h) that wraps these commands in a way that they
// called synchronously.

Command tilt_antenna (Real degrees);
Command pan_antenna (Real degrees);
Command take_picture();

Command arm_move_joint (Boolean relative,
                        Integer joint,
                        Real angle);

Command arm_move_joints (Boolean relative, 
                         Real angles[6]);

Command dig_circular (Real x,
                      Real y,
                      Real depth,
                      Real ground_pos,
                      Boolean parallel); // true means parallel to lander arm

Command dig_linear (Real x,
                    Real y,
                    Real depth,
                    Real length,
                    Real ground_pos);

Command deliver ();

Command discard (Real x,
                 Real y,
                 Real z);

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

// Move from stowed position to a "ready" position
Command unstow();

// Move from "ready" position to stowed position; requires unstow() first
Command stow();

// Processes number of images already taken with the stereo camera to
// find the 3d point to sample.  filter_type can either be "Dark" or
// "Brown".  (Dark chooses dark spots, brown chooses brown spots).
Real [3] Command identify_sample_location(Integer num_images, String filter_type);

// Set spotlight intensity
Command set_light_intensity (String side,     // "left" or "right"
			     Real intensity); // 0.0 to 1.0.  0 is off.

#endif
