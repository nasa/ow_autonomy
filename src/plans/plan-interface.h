// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plan_Interface_H
#define Ow_Plan_Interface_H

// PLEXIL interface: commands, lookups, library plans

// Lander commands

Command dig_linear (Real x, Real y, Real depth, Real length,
                    Real ground_pos);
Command dig_circular (Real x, Real y, Real depth, Real ground_pos,
                      Boolean radial);
Command deliver_sample (Real x, Real y, Real z);
Command grind (Real x, Real y, Real depth, Real length, Real ground_pos);
Command guarded_move (Real x, Real y, Real z,
                      Real dir_x, Real dir_y, Real dir_z,
                      Real search_distance);
Command stow();
Command unstow();
Command publish_trajectory();

// Utility commands

Command log_info (...);
Command log_warning (...);
Command log_error (...);


// Lookups

Boolean Lookup Running (String operation_name);
Boolean Lookup Finished (String operation_name);


// Plan Library

LibraryAction Unstow();


#endif
