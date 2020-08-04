The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

PLEXIL plans
============
This directory contains PLEXIL plans that implement onboard autonomy for Ocean
World landers, as supported by the OceanWATERS software testbed.

In particular, EuropaMission.plp implements the Europa Reference Mission as
described in project documentation.

See ow_autonomy/README.md for build instructions.

GroundControl node/GetTrenchTarget.plp
======================================
A few options when using this plan or the GroundControl node. 

If you want to use ROS time instead of system time, open 
ow_autonomy/launch/autonomy_node.launch and set the "/use_sim_time" parameter 
to true. 

If you want the "GroundControl" node to use the target set from ground instead of 
the target aquired  onboard the lander, set the "/use_onboard_target" parameter 
to false. 

You can also set the length of time you want the "communications_delay", 
"/decision_duration", and "/timeout_duration" to be in seconds. 
"/communications_delay" is the delay for message travel time between the lander and
earth. "/decision_duration" is the time for a decision to be made on "GroundControl" 
before sending the message back to the lander. "/timeout_duration" is how long the 
lander should wait before moving on without waiting for a response from "GroundControl." 

If you choose for Ground Control to send a new target instead of using the onboard 
target by setting "/use_onboard_target" to false, you can set the xyz values to 
anything you would like to test. You do this by adjusting the "/x_coordinate", 
"/y_coordinate", and "/z_coordinate" values in the file, autonomy_node.launch.
