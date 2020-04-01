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

1. Start the simulator, using a launch file that supports arm planning.  The
   simplest one is:

   % roslaunch ow arm_sim.launch

   This one adds the 'faults' node and rqt:

   % roslaunch ow_autonomy autonomy_test.launch

   This one further adds a terrain model (requires good graphics performance):

   % roslaunch ow europa_test_dem.launch arm_sim_enable:=true

2. (optional, as needed) Start rqt for visualization and monitoring.

   % rqt

   If you wish to view camera images:

   a) Select Plugins/Visualization/Image View
	 b) In the topic pulldown menu, select /StereoCamera/left/image_raw

   Note that /StereoCamera/left/image_raw_mouse_left appears in the box below.

3. Start the autonomy node.  To use the default plan:

   % roslaunch ow_autonomy autonomy_node.launch

   This invocation loads the default PLEXIL plan, Demo.plx, which demonstrates a
   variety of lander functions: arm planning, guarded move, publishing
   trajectory (for Gazebo), and panoramic imaging.  It is equivalent to:

   % roslaunch ow_autonomy autonomy_node.launch plan:=Demo.plx

   Alternatively, you may specify a plan to load, by setting the 'plan' argument
   to a .plx file found in:

   <ow_workspace>/devel/etc/plexil


Plans
-----

Here are the meaningful plans currently available.

1. The Europa reference mission Sol 0 prototype:

   % roslaunch ow_autonomy autonomy_node.launch plan:=OceanWorldMission.plx

   At present, this plan is stubbed except for the (shortened) panoramic landing
   site imaging that happens near the beginning.

2. A short panoramic imaging demo:

   % roslaunch ow_autonomy autonomy_node.launch plan:=TestAntennaCamera.plx

   A small area is imaged, with an iteration of tilts and pans.  To try other
   regions, edit ow_autonomy/src/plans/TestAntennaCamera.plp and enter 'catkin
   build ow_autonomy' before restarting the autonomy node.  You don't need to
   restart the simulator.

3. Overtorque detection:

   % roslaunch ow_autonomy autonomy_node.launch plan:=TorqueTest.plx

   This plan attempts to push the scoop into the ground, which creates joint
   over-torquing warnings and errors.


Clean
-----

To clean (remove all build products from) the entire ROS workspace:

  % catkin clean

To clean just the ow_autonomy package:

  % cd <ow_workspace>/build
	% rm -rf ow_autonomy
