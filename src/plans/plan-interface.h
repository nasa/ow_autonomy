// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plan_Interface_H
#define Ow_Plan_Interface_H

// Lander commands

Command dig_linear (Real X, Real Y, Real Depth, Real Length,
                    Real GroundPosition);
Command dig_circular (Real x, Real y, Real depth, Real ground_pos,
                      Boolean radial);

// Plan execution support

Boolean Lookup Running (String operation_name);
Boolean Lookup Finished (String operation_name);

// ROS Utilities

Command log_info (...);
Command log_warning (...);
Command log_error (...);


#endif
