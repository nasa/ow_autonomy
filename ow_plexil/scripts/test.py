#!/usr/bin/env python2
import rospy
import image_geometry
import sys
from sensor_msgs.msg import Image, CameraInfo

class IdentifySampleLocation:

  def __init__(self):
    left_camera_info_subscriber = rospy.Subscriber("/StereoCamera/left/camera_info", CameraInfo, self.left_camera_info_callback)
    right_camera_info_subscriber = rospy.Subscriber("/StereoCamera/right/camera_info", CameraInfo, self.right_camera_info_callback)
    rectified_image_subscriber = rospy.Subscriber("/StereoCamera/left/image_rect", Image, self.rectified_image_callback)

    self.stereo_camera_model = None
    self.right_info = None
    self.left_info = None
    


  def left_camera_info_callback(self, msg):
    if self.left_info == None:
      self.left_info = msg
      print("HERE")
 
  def right_camera_info_callback(self, msg):
    if self.right_info == None:
      self.right_info = msg
      print("HERE2")
   
  def rectified_image_callback(self, msg):
    print("MADE IT")
    






if __name__ == '__main__':
  rospy.init_node('test_node', argv=sys.argv)
  IdentifySampleLocation()
  camera = image_geometry.StereoCameraModel()
  rospy.spin()
