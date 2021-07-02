The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of this
repository.

This directory contains the ROS package `ow_plexil`, which provides:
 - a ROS node named `plexil_node` that embeds a PLEXIL plan executive
 - sample autonomy plans written in PLEXIL
 - code that interfaces the PLEXIL plans with the OceanWATERS simulator

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

Build just the ow_plexil package with:

```bash
 catkin build ow_plexil
```

_NOTE: If any new PLEXIL plans (.plp or .ple files) have been added since your
last build, a clean rebuild of ow_plexil is needed.  See bottom of this file
for instructions._


Start the Plexil node
---------------------

1. First you must start the simulator, e.g.

```bash
  roslaunch ow europa_terminator.launch
```
   NOTES:
    - to omit the Gazebo GUI for faster performance, add `gzclient:=false`
    - for alternate terrains, other launch files are available:
      `atacama_y1a.launch`, `europa_terminator_workspace.launch`,
      `europa_test_dem.launch`.

2. Next start the Plexil node.  Starting the Plexil node always runs a
   PLEXIL plan.  The simplest version is:

   `roslaunch ow_plexil plexil_node.launch`

   This invocation loads the default PLEXIL plan, `Demo.plx`.  A specific plan
   may be run by adding it to the command line, e.g.

   `roslaunch ow_plexil plexil_node.launch plan:=ReferenceMission1.plx`

   The argument given to the `plan` parameter must be a file found in :

   `<ow_workspace>/devel/etc/plexil`

   See `plans/README.md` for a description of the PLEXIL plans.


Clean
-----

To clean (remove all build products from) just the ow_plexil package:

 `cd <ow_workspace>/build`
 `rm -rf ow_plexil`

To clean the entire ROS workspace (not needed if you only want to rebuild
ow_plexil):

  `catkin clean`
