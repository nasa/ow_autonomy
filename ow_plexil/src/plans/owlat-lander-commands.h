// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Owlat_commands_H
#define Owlat_commands_H

// All available PLEXIL commands in OWLAT to the lander.  This is essentially the
// lander's command interface.

// Note that PLEXIL commands are asynchronous by design.  In general, autonomy
// plans should not call these directly, but rather user the Library interface
// that wraps these commands; this is defined in plan-interface.h.







// Move from stowed position to a "ready" position
Command owlat_unstow();
// Move from "ready" position to stowed position; requires unstow() first
Command owlat_stow();
Command owlat_arm_move_cartesian(Integer frame, Boolean relative, Real position[3], Real orientation[4]); 

#endif
