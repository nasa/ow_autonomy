The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

OWLAT PLEXIL plans
============

This directory contains PLEXIL plans that implement onboard autonomy for Ocean
World landers, as supported by the OWLAT software testbed. 

See ow_plexil/README.md for instructions for selecting and executing plans.

Not all of these plans may be executed directly, because of some of them are
library plans.  Only _top level_ plans may be run directly.  A top level plan is
one having no parameters, i.e. no `In` or `InOut` variable declarations near the
top.

Thorough documentation is provided for all lander actions in the OWLAT user guide.
Test sequence plans found in this directory (ie: OwlatTestSequence1-6) are also documented
in the user guide and include comments within the plans explaining where to find more 
information. Some of these test sequences will cause the arm to clip through the ground
plane or end in an aborted state. This is known behavior and newer versions of OWLAT are 
working on fixing these issues. Visually these TestSequences will still perform the operations
but may report aborted.

Known Issues:
  -TestSequence1: Clipping through ground plane, Aborted due to not reaching force threshold
  -TestSequence2: None 
  -TestSequence3: Clipping through ground plane, Aborted move
  -TestSequence4: Clipping through ground plane, Aborted move
  -TestSequence5: Clipping through ground plane, Aborted move
  -TestSequence6: Clipping through ground plane, Aborted move, Do not run back to back; can
                  cause OWLAT to throw extraneous velocity error which will freeze the sim and
                  prevent any further plans from running.

