The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

OWLAT PLEXIL plans
============

This directory contains PLEXIL plans that implement onboard autonomy
for Ocean World landers, as supported by the OWLAT Simulator, release
0.5.4.  This is closed-source proprietary software from NASA JPL that
is not available to the general public.

See ow_plexil/README.md for instructions for selecting and executing plans.

Only the _top level_ plans may be executed directly.  See `README.md`
in the parent directory for an explanation.

The 'include' file `owlat-interface.h` declares the OWLAT-specific
PLEXIL interface.

Thorough documentation is provided for all lander actions in the OWLAT
Simulator user guide, which comes with its software distribution.
Test sequences described in the user guide are implemented
collectively by the `TestOwlatActions.plp` plan.  Note that some of
these test sequences will cause the arm to clip through the ground plane.
