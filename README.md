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

     % catkin build

A prerequisite for building and running this application is a working PLEXIL
installation, which has its own prerequisites.  For instructions see the
"Development Environment" Confluence page. The environment variable PLEXIL_HOME
must be set to PLEXIL's installation pathname.

Your ROS environment should also first be set up:

     % source <catkin-workspace>/devel/setup.sh


Run
---

First launch ow_lander as follows.

     % roslaunch ow arm_sim.launch

Then launch the autonomy node.  The default launch looks like this:

     % roslaunch ow_autonomy autonomy_node.launch

This invocation loads the default PLEXIL plan, Demo.plx, which
demonstrates a variety of lander functions: arm planning, guarded move,
publishing trajectory (for Gazebo), antenna and camera functions.

Alternatively, you may specify a plan to load.  The default launch is
equivalent to:

     % roslaunch ow_autonomy autonomy.launch plan:="Demo.plx"

Use the 'plan' argument to specify a .plx file found in

     <ow_workspace>/devel/etc/plexil

An interesting choice is the Europa reference mission prototype:

     % roslaunch ow_autonomy autonomy.launch plan:="OceanWorldMission.plx"