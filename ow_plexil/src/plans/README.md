The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

PLEXIL plans
============

This directory contains PLEXIL plans that implement onboard autonomy
for Ocean World landers, as supported by the OceanWATERS and OWLAT
testbeds.  This directory directly contains plans specific to
OceanWATERS.  Plans shared by OceanWATERS and OWLAT are found in the
`common` subdirectory. and those specific to OWLAT are found in
`owlat`.

See ow_plexil/README.md for instructions for selecting and executing
plans.

Not all of these plans may be executed directly, because of some of
them are library plans.  Only _top level_ plans may be run directly.
A top level plan is one having no parameters, i.e. no `In` or `InOut`
variable declarations.

Descriptions of some key plans, and other files of interest, are as
follows.  See comments inside the plans for more information.

* ReferenceMission1, ReferenceMission2 : both plans model a portion of
  Sol 0 of the Europa Lander reference mission defined by JPL.  The
  second plan is the same as the first one except it adds simple
  checks for the battery and implements basic fault detection and
  handling.

* EuropaMission: a variant of the above that includes some additional
  stubbed mission operations, as well as _checkpointing_, a new and
  experimental PLEXIL feature that supports robust plan resumption
  after a reboot.  Checkpoint files are saved to ~/.ros by default;
  the location can be customized in `ow-config.xml`.

* Demo: Exercises a short sequence of arm and antenna operations.

* TestAntennaCamera: A short panoramic imaging demo.

* TestAntennaMovement: A more thorough test of pan/tilt operations.

* TorqueTest: Overtorque detection.  This plan attempts to push the
  scoop into the ground, which creates joint over-torquing warnings
  and errors.

* Continuous: non-terminating plan that performs continuous
  operations, useful as a stress/load test.

* IdentifySampleLocationDemo: This plan demonstrates the
  IdentifySampleTarget plan which uses the stereocamera to find a
  desirable sample location. It runs two tests, one with the "Brown"
  filter and one with the "Dark" filter. These filters find areas in
  the workspace that have the highest color concentration (Brown or
  Dark colors based on the filter used) and returns a 3d point that
  represents the sample target.  The plan then makes a guarded move to
  these sample targets. When a point is found the action server
  publishes two visualization topics which are described in testing
  plans below.

* FaultHandlingPatternN: A series of illustrative fault-handling patterns.

* Lander operation library.  There is a (library) plan for each lander
  operation, and each takes the arguments (interface variables) as
  required for the operation.  While these operations can be directly
  invoked as PLEXIL commands, these library plans wrap the command to
  provide the following features:

  - The command is invoked as a SynchronousCommand, which means the
    plan waits for it to complete before the plan itself finishes.

  - In some cases, a check is made for relevant faults prior to
    executing the command.  The plan waits for an existing fault to
    clear before executing the command.  NOTE: an existing fault will
    block the calling plan; this can be worked around by customizing
    these plans, e.g. adding a timeout.  The lander operation library
    plans at the time of this writing are the following.

* Simulation interface.  The entire simulation interface available to
  PLEXIL, which includes the operations listed in the previous
  section, telemetry lookups, and other useful features such as
  logging functions, are encoded in `ow-interface.h`, which may be
  included (using a C `#include` statement) in any plan for
  convenience.

* Misc files.  `plexil-defs.h` contains constants used in various
  plans.  `ow-config.xml` is the PLEXIL configuration file for this
  application.  It does not need editing for general use of this
  package.

There are numerous other useful plans in this directory that are not
described here.  Have a look!


Plan Details
============

### ReferenceMission2 ###

The following represent the success criteria of the Reference Mission 2 plan:
- Left running the plan completes
  - Upon completion, the plan will print "ReferenceMission2 plan complete."
  - To more easily view this message when it occurs, add the following
    after the launch command: ` | grep -i "ReferenceMission2 plan complete."`

- The plan is interruptable through fault injection, i.e. pauses in
  response to a fault
  - Faults can be injected via rqt, a python script, or the command
    line.  See `ow_simulator/ow_faults_injection/README.md

### IdentifySampleLocationDemo ###

The following represent the success criteria of the IdentifySampleLocationDemo
plan.  Note that the Plexil node must first be started (with or without a plan
specified) to enable the RQT and RViz additions specified below.  And you'll
need to first refresh the topic menu in RQT.
- In the Rqt image viewer select the `/sample_location` topic (last entry at the
  time of this writing).  Once a point is selected the outlined contours of
  potential sample locations and the chosen target should show up.  The green
  dot represents the selected point.
- In Rviz to vizualize the 3d point and to see if the guarded move went to the
  correct location:
  - Select Add in the left menu under "Displays"
  - Select "By topic"
  - Scroll down and select the marker topic `/sample_point_visualization` and
    select OK.
  - Once a point is selected you should see a green sphere appear at the 3d
    point location.
- Both IdentifySampleTarget calls find a point and the guarded move attempts to
  move to the locations. Note that due to a bug in guarded move it may fail
  occasionally, but as long as it is close to the point selected then
  IdentifySampleLocationDemo was succesful.
- In rare cases no sample points can be found. If this happens success is
  defined by a graceful exit.
