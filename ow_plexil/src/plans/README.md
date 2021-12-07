The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

PLEXIL plans
============

This directory contains PLEXIL plans that implement onboard autonomy for Ocean
World landers, as supported by the OceanWATERS software testbed. All PLEXIL plans
for the OWLAT testbed are kept in the owlat_plans subdirectory.

See ow_plexil/README.md for instructions for selecting and executing plans.

Not all of these plans may be executed directly, because of some of them are
library plans.  Only _top level_ plans may be run directly.  A top level plan is
one having no parameters, i.e. no `In` or `InOut` variable declarations near the
top.

Descriptions of some key plans, and other files of interest, are as follows.
See the comments inside all the plans for more information.

1. ReferenceMission1, ReferenceMission2 : both plans model a portion of Sol 0 of
   the Europa Lander reference mission defined by JPL.  The second plan is the
   same as the first one except it adds simple checks for the battery and
   implements basic fault detection and handling.

2. EuropaMission: a variant of the above that includes some additional stubbed
   mission operations, as well as _checkpointing_, a new and experimental PLEXIL
   feature that supports robust plan resumption after a reboot.  Checkpoint
   filers are saved to ~/.ros by default; the location can be customized in
   `ow-config.xml`.

3. Demo: Exercises a short sequence of arm and antenna operations.

4. TestAntennaCamera: A short panoramic imaging demo.

5. TestAntennaMovement: A more thorough test of pan/tilt operations.

6. TorqueTest: Overtorque detection.  This plan attempts to push the scoop into
   the ground, which creates joint over-torquing warnings and errors.

7. Continuous: non-terminating plan that performs continuous operations, useful
   as a stress/load test.

8. IdentifySampleLocationDemo: This plan demonstrates the IdentifySampleTarget
   plan which uses the stereocamera to find a desirable sample location. It runs
   two tests, one with the "Brown" filter and one with the "Dark" filter. These
   filters find areas in the workspace that have the highest color concentration
   (Brown or Dark colors based on the filter used) and returns a 3d point that
   represents the sample target.  The plan then makes a guarded move to these
   sample targets. When a point is found the action server publishes two
   visualization topics which are described in testing plans below.

9. FaultHandlingPatternN: A series of illustrative fault-handling patterns.

Plan Details
============

### ReferenceMission2 ###

The following represent the success criteria of the Reference Mission 2 plan:
- Left running the plan completes
	- Upon completion, the plan will print "ReferenceMission2 plan
	complete."
	- To more easily view this message when it occurs, add the following
 	after the launch command:<br/>
	` | grep -i "ReferenceMission2 plan complete."`
- The plan is interruptable through fault injection, i.e. pauses in
response to a fault
	- Faults can be injected via rqt, a python script, or the command line
 ([Fault Injection Tutorial](https://github.com/nasa/ow_simulator/blob/master/ow_faults_injection/README.md))


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

### FaultHandlingPatternN ###

Each plan contains comments that describe its behavior and how to test
the pattern.  Note that these plans do not always behave exactly as
described within, for two reasons.  First, the library plans Pan,
Unstow, and Stow currently implement a "permissive" kind of fault
handling at that level, in that they _wait_ for any fault in progress
to resolve before commencing.  Second, aborted ROS Actions, which
result from faults, are not yet detectable in PLEXIL; PLEXIL only
knows when an action terminates.  These subtleties are subjects for
future work and refinement.

