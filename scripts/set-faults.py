#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

# This is a utility script for fault injection.  The default script injects
# every available fault.  Edit your local copy as needed for your testing
# purposes.

import rospy
import dynamic_reconfigure.client

if __name__ == "__main__":
  rospy.init_node("faults_client")
  client = dynamic_reconfigure.client.Client('/faults')
  params = { 'ant_pan_encoder_failure' : 'True',
             'ant_tilt_effort_failure' : 'True',
             'ant_pan_effort_failure' : 'True',
             'ant_tilt_encoder_failure' : 'True',
             'dist_pitch_encoder_failure' : 'True',
             'dist_pitch_effort_failure' : 'True',
             'hand_yaw_encoder_failure' : 'True',
             'hand_yaw_effort_failure' : 'True',
             'prox_pitch_encoder_failure' : 'True',
             'prox_pitch_effort_failure' : 'True',
             'scoop_yaw_encoder_failure' : 'True',
             'scoop_yaw_effort_failure' : 'True',
             'shou_pitch_encoder_failure' : 'True',
             'shou_pitch_effort_failure' : 'True',
             'shou_yaw_encoder_failure' : 'True',
             'shou_yaw_effort_failure' : 'True' }

  rate = rospy.Rate(0.1)
  while not rospy.is_shutdown():
    # config = client.update_configuration(params)
    rate.sleep()
