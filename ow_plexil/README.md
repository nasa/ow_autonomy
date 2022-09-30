The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of this
repository.

This directory contains the ROS package `ow_plexil`, which provides:
 - launch file for OceanWATERS plan execution (ow_exec.launch)
 - launch file for OWLAT plan execution (owlat_exec.launch)
 - ROS nodes supporting the above launch files
 - sample autonomy plans written in PLEXIL
 - code that interfaces the PLEXIL plans with the testbeds

Note: OWLAT (Ocean Worlds Lander Autonomy Testbed) is a proprietery,
closed-source simulator developed at JPL.  Instructions for use of
this testbed are not provided with OceanWATERS.

Contents
--------

`src/plans` directory contains the PLEXIL plans.

`src/plexil-adapter` contains the supporting code needed to run the PLEXIL plans,
and also the ROS node implementations.

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

NOTE: If you have OWLAT, define the OWLAT flag, i.e.

  `catkin build -DOWLAT=ON`

or

  `catkin build ow_plexil -DOWLAT=ON`.

_IMPORTANT: If any PLEXIL plans (.plp files) have been added or removed since
your last build, a clean rebuild of ow_plexil is needed.  This is due to a build
deficiency.  See bottom of this file for instructions._



Launch the plan executive
-------------------------

1. First you must start the simulator, e.g.

  For OceanWATERS:
  ```bash
  roslaunch ow europa_terminator.launch
  ```
   NOTES:
    - to omit the Gazebo GUI for faster performance, add `gzclient:=false`
    - for alternate terrains, other launch files are available:
      `atacama_y1a.launch`, `europa_terminator_workspace.launch`,
      `europa_test_dem.launch`.
    - the environment variable `$PLEXIL_PLAN_DIR` is required to locate the
      `.plx` files used by the executive. This environment varialbe is exported
      by the env-hooks which are sourced when sourcing the catkin setup script,
      and should point to a directory in the catkin develspace where the `.plx`
      files are deployed.

  For OWLAT:
  Start ROS: ``` roscore ```
  Then start OWLAT in a separate tab: ```launch_owlat Scoop```

2. Next launch the desired executive.  For OceanWATERS:

   `roslaunch ow_plexil ow_exec.launch`

   For OWLAT:

   `roslaunch ow_plexil owlat_exec.launch`

   NOTE: only one of these may be launched at a time.

   You will be prompted for a plan to execute.  This must be a `.plx` file
	 that exists in `<ow_workspace>/devel/etc/plexil`.

   NOTE: you must select a plan that is valid for the given testbed.  An OWLAT
   plan will not run with OceanWATERS, or vice versa -- the executive node will
   crash.

   Optionally, a plan may be specified on the command line, e.g.

   `roslaunch ow_plexil ow_exec.launch plan:=ReferenceMission1.plx`

   See `plans/README.md` for a description of the PLEXIL plans.

   Optionally, plans may be selected and queued for execution using the Plan
   Selection GUI in rqt.


Clean
-----

To clean (remove all build products from) just the ow_plexil package:

 `cd <ow_workspace>/build`
 `rm -rf ow_plexil`

To clean the entire ROS workspace (not needed if you only want to rebuild
ow_plexil):

  `catkin clean`

Caveats
-------

When using the OWLAT simulator, some operations, in particular the arm stow and
unstow, may be reported as completed well before the arm has come to rest.  This
is a known issue in the OWLAT simulator.
