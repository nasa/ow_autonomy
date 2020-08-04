The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

PLEXIL adapter
==============
This directory contains C++ classes and code implementing the interface between
the PLEXIL plans (found in ../plans) and the external world (lander,
environment), as well as the ROS autonomy node.

It also contains a dummy stand-in ROS action server for GuardedMove, as well as
a standalone node for testing it.  (GuardedMoveServer, GuardedMoveClient).  Run
the following sequence, in three separate shells.


```bash
  roscore
  GuardedMoveServer
  GuardedMoveClient
  ```

See ow_autonomy/README.md for build instructions.
