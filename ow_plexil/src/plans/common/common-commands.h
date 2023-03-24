// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef common_commands_H
#define common_commands_H

// All available PLEXIL commands to the lander in both OceanWATERS and
// OWLAT.  This is essentially the lander's command interface.

// Note that in PLEXIL, commands are asynchronous by design.
// Typically, OceanWATERS plans should not call the following commands
// directly, but instead use the Library interface (defined in
// plan-interface.h) that wraps these commands in a way that they
// called synchronously.

Command arm_unstow();
Command arm_stow();

#endif
