// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_commands_H
#define Ow_commands_H

// All available PLEXIL commands to the lander.  This is essentially the
// lander's command interface.

// Note that PLEXIL commands are asynchronous by design.  In general, autonomy
// plans should not call these directly, but rather user the Library interface
// that wraps these commands; this is defined in plan-interface.h.

Command tilt_antenna (Real degrees);
Command pan_antenna (Real degrees);
Command take_picture();

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

// This dumps the scoop at the given location; can be used for removing tailings
// as well as delivering a sample to the receptacle.
Command deliver (Real x,
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

#endif
