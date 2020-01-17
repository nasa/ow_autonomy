This directory contains a candidate onboard autonomy component for the Ocean
Worlds lander, namely a ROS node embedding a PLEXIL application.

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

     % source <catkin-workspace>/devel/setup.sh

Assumed is this directory filed properly within an OceanWATERS ROS workspace
(see Conftluence for instructions).  Build the entire workspace with:

     % catkin build

Build just the ow_autonomy package with:

     % catkin build ow_autonomy


Run
---

First launch ow_lander as follows.

     % roslaunch ow arm_sim.launch

Then launch the autonomy node.  The default launch looks like this:

     % roslaunch ow_autonomy autonomy_node.launch

This invocation loads the default PLEXIL plan, Demo.plx, which
demonstrates a variety of lander functions: arm planning, guarded move,
publishing trajectory (for Gazebo), and panoramic imaging.

Alternatively, you may specify a plan to load.  The default launch is
equivalent to:

     % roslaunch ow_autonomy autonomy_node.launch plan:=Demo.plx

Use the 'plan' argument to specify a .plx file found in

     <ow_workspace>/devel/etc/plexil


Plans
-----

Here are the meaningful plans currently available.

1. The Europa reference mission Sol 0 prototype:

   % roslaunch ow_autonomy autonomy_node.launch plan:=OceanWorldMission.plx

   At present, this plan is stubbed except for the 360 degree landing site
   imaging that happens near the beginning.  It takes many minutes!

2. A short panoramic imaging demo:

   % roslaunch ow_autonomy autonomy_node.launch plan:=TestAntennaCamera.plx

   A small area is imaged, with an iteration of tilts and pans.  To try other
   regions, edit ow_autonomy/src/plans/TestAntennaCamera.plp and enter 'catkin
   build ow_autonomy' before restarting the autonomy node.  You don't need to
   restart ow_lander.
