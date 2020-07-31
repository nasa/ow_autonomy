#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import dynamic_reconfigure.client

if __name__ == "__main__":
  rospy.init_node("faults_client")
  client = dynamic_reconfigure.client.Client('/faults')
  params = { 'ant_pan_encoder_failure' : 'True',
             'ant_tilt_torque_sensor_failure' : 'True',
             'ant_pan_torque_sensor_failure' : 'True',
             'ant_tilt_encoder_failure' : 'True',
             'dist_pitch_encoder_failure' : 'True',
             'dist_pitch_torque_sensor_failure' : 'True',
             'hand_yaw_encoder_failure' : 'True',
             'hand_yaw_torque_sensor_failure' : 'True',
             'prox_pitch_encoder_failure' : 'True',
             'prox_pitch_torque_sensor_failure' : 'True',
             'scoop_yaw_encoder_failure' : 'True',
             'scoop_yaw_torque_sensor_failure' : 'True',
             'shou_pitch_encoder_failure' : 'True',
             'shou_pitch_torque_sensor_failure' : 'True',
             'shou_yaw_encoder_failure' : 'True',
             'shou_yaw_torque_sensor_failure' : 'True' }

  rate = rospy.Rate(0.1)
  while not rospy.is_shutdown():
    config = client.update_configuration(params)
    rate.sleep()
