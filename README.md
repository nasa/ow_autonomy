Notices:
--------
Copyright © 2020-2021 United States Government as represented by the
Administrator of the National Aeronautics and Space Administration.  All Rights
Reserved.

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
Worlds lander, namely a ROS node (named `autonomy node`) embedding a PLEXIL plan
executive.


Contents
--------

`src/plans` directory contains the PLEXIL plans.

`src/plexil-adapter` contains the supporting code needed to run the PLEXIL plans,
and also the ROS node implementation.

See the `README.md` files in each subdirectory for more information.


Build
-----

See [Getting
Started](https://github.com/nasa/ow_simulator/blob/master/README.md) for
complete installation and build instructions as well as hardware/software
requirements.  This file recaps a few key points and has some supplementary
information.

A prerequisite for building and running this package is a working PLEXIL
installation, which has its own prerequisites. The environment variable
PLEXIL_HOME must be set to PLEXIL's installation pathname.

Your ROS environment should also first be set up:

```bash
 source <catkin-workspace>/devel/setup.sh
```

Assumed is this directory filed properly within an OceanWATERS ROS workspace
(see `ow_simulator/oceanwaters/setup_oceanwaters.md` for instructions).  Build
the entire workspace with:

```bash
 catkin build
```

Build just the ow_autonomy package with:

```bash
 catkin build ow_autonomy
```

_NOTE: If any new PLEXIL plans (.plp or .ple files) have been added since your
last build, a clean rebuild of ow_autonomy is needed.  See bottom of this file
for instructions._


Start the autonomy node
-----------------------

1. First you must start the simulator, e.g.

```bash
  roslaunch ow europa_terminator.launch
```
   NOTES:
    - to omit the Gazebo GUI for faster performance, add `gzclient:=false`
    - for alternate terrains, other launch files are available:
      `atacama_y1a.launch`, `europa_terminator_workspace.launch`,
      `europa_test_dem.launch`.

2. Next start the autonomy node.  Starting the autonomy node always runs a
   PLEXIL plan.  The simplest version is:

   `roslaunch ow_autonomy autonomy_node.launch`

   This invocation loads the default PLEXIL plan, `Demo.plx`.  A specific plan
   may be run by adding it to the command line, e.g.

   `roslaunch ow_autonomy autonomy_node.launch plan:=ReferenceMission1.plx`

   The argument given to the `plan` parameter must be a file found in :

   `<ow_workspace>/devel/etc/plexil`

   See `plans/README.md` for a description of the PLEXIL plans.


Clean
-----

To clean (remove all build products from) just the ow_autonomy package:

 `cd <ow_workspace>/build`
 `rm -rf ow_autonomy`

To clean the entire ROS workspace (not needed if you only want to rebuild
ow_autonomy):

  `catkin clean`
