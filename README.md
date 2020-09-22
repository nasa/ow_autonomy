Notices:
--------
Copyright © 2020 United States Government as represented by the Administrator of
the National Aeronautics and Space Administration.  All Rights Reserved.

Disclaimers
-----------
No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF
ANY KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY
IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL BE ERROR
FREE, OR ANY WARRANTY THAT DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE
SUBJECT SOFTWARE. THIS AGREEMENT DOES NOT, IN ANY MANNER, CONSTITUTE AN
ENDORSEMENT BY GOVERNMENT AGENCY OR ANY PRIOR RECIPIENT OF ANY RESULTS,
RESULTING DESIGNS, HARDWARE, SOFTWARE PRODUCTS OR ANY OTHER APPLICATIONS
RESULTING FROM USE OF THE SUBJECT SOFTWARE.  FURTHER, GOVERNMENT AGENCY
DISCLAIMS ALL WARRANTIES AND LIABILITIES REGARDING THIRD-PARTY SOFTWARE, IF
PRESENT IN THE ORIGINAL SOFTWARE, AND DISTRIBUTES IT "AS IS."

Waiver and Indemnity:  RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST THE
UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY
PRIOR RECIPIENT.  IF RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN ANY
LIABILITIES, DEMANDS, DAMAGES, EXPENSES OR LOSSES ARISING FROM SUCH USE,
INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING FROM, RECIPIENT'S USE
OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE UNITED
STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR
RECIPIENT, TO THE EXTENT PERMITTED BY LAW.  RECIPIENT'S SOLE REMEDY FOR ANY SUCH
MATTER SHALL BE THE IMMEDIATE, UNILATERAL TERMINATION OF THIS AGREEMENT.


ow_autonomy
===========

This package contains a candidate onboard autonomy component for an Ocean
Worlds lander, namely a ROS node embedding a PLEXIL application.

It's called the `autonomy node`, and only one instance of this node in an
OceanWATERS application/mission is envisioned at this time.


Contents
--------

src/plans directory contains the PLEXIL plans.

src/plexil-adapter contains the supporting code needed to run them, and also the
ROS node implementation (for now).

See the README files in each subdirectory for more information.


Build
-----

A prerequisite for building and running this application is a working PLEXIL
installation, which has its own prerequisites.  For instructions see the
"Development Environment" Confluence page. The environment variable PLEXIL_HOME
must be set to PLEXIL's installation pathname.

Your ROS environment should also first be set up:

```bash
 source <catkin-workspace>/devel/setup.sh
```

Assumed is this directory filed properly within an OceanWATERS ROS workspace
(see Conftluence for instructions).  Build the entire workspace with:

```bash
 catkin build
```

Build just the ow_autonomy package with:

```bash
 catkin build ow_autonomy
```

NOTE: If any new PLEXIL plans (.plp or .ple files) have been added since your
last build, a clean rebuild of ow_autonomy is needed.  See bottom of this file
for instructions.


Run
---

1. Start the simulator:

```bash
  roslaunch ow europa_test_dem.launch arm_sim_enable:=true
```
   NOTES:
    - for faster performance, add `gzclient:=false`
    - for better terrain, use `europa_terminator.launch`

2. (optional, as needed) Start rqt for visualization and monitoring.

   `rqt`

   If you wish to view camera images:

   a) Select `Plugins/Visualization/Image View`
	 b) In the topic pulldown menu, select `/StereoCamera/left/image_raw`

   Note that `/StereoCamera/left/image_raw_mouse_left` appears in the box below.

3. Start the autonomy node.  To use the default plan:

   `roslaunch ow_autonomy autonomy_node.launch`

   This invocation loads the default PLEXIL plan, Demo.plx, which demonstrates a
   variety of lander functions: arm planning, guarded move, and panoramic
   imaging.  It is equivalent to:

   `roslaunch ow_autonomy autonomy_node.launch plan:=Demo.plx`

   Alternatively, you may specify a plan to load, by setting the 'plan' argument
   to a .plx file found in:

   `<ow_workspace>/devel/etc/plexil`


Plans
-----

Here are some interesting autonomy plans.  Run them as shown, substituting for
`Foo.plx`:

  `roslaunch ow_autonomy autonomy_node.launch plan:=Foo.plx`

1. A Europa reference mission Sol 0 prototype: `ReferenceMission1.plx`

2. A short panoramic imaging demo: `TestAntennaCamera.plx`

   A small area is imaged, with an iteration of tilts and pans.  To try other
   regions, edit `ow_autonomy/src/plans/TestAntennaCamera.plp` and enter

   `catkin build ow_autonomy`

   before restarting the autonomy node.  You don't need to restart the simulator.

3. Overtorque detection: `TorqueTest.plx`

   This plan attempts to push the scoop into the ground, which creates joint
   over-torquing warnings and errors.

4. Concurrency demo: `TestPlanning.plx`

   This runs the arm planning ROS service, while concurrently printing an "in
   progress" message.  This was not possible before service calls were threaded.

5. ROS Action demo: `TestActions.plx`

   This runs a dummy GuardedMove action (not connected to simulator)
   concurrently with the real GuardedMove service.
   
6. Demonstration of checkpointed plans: `EuropaMission.plx`


Fault Detection
---------------

A rudimentary version does nothing more than check relevant fault parameters for
each lander operation.  Run any plan you like, while setting fault parameters as
desired (see `ow_faults/README.md` for instructions).  Faults are reported as
ROS warnings.


Clean
-----

To clean (remove all build products from) just the ow_autonomy package:

 `cd <ow_workspace>/build`
 `rm -rf ow_autonomy`

To clean the entire ROS workspace (not needed if you're just rebuilding
ow_autonomy):

  `catkin clean`
