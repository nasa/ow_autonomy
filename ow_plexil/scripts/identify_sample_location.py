#!/usr/bin/env python2
import rospy
import sys
import cv2 as cv
import tf
import actionlib
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
import sensor_msgs.point_cloud2
import message_filters
from collections import namedtuple
from ow_plexil.msg import IdentifyLocationAction, IdentifyLocationGoal, IdentifyLocationFeedback, IdentifyLocationResult


from visualization_msgs.msg import Marker, MarkerArray

class IdentifySampleLocation:

  def __init__(self):

    #setting up synchronized subscribers for image_rect and point cloud
    rectified_image_subscriber = message_filters.Subscriber("/StereoCamera/left/image_rect", Image)
    point_cloud_subscriber = message_filters.Subscriber("/StereoCamera/points2", PointCloud2)
    time_synchronizer = message_filters.TimeSynchronizer([rectified_image_subscriber, point_cloud_subscriber], 10)
    time_synchronizer.registerCallback(self.site_imaging_callback)

    #publishes modified opencv image with the contours and sample location outlined
    self.publish_photo = rospy.Publisher('sample_location', Image, queue_size=10)

    #action server setup
    self.sample_location_action_server = actionlib.SimpleActionServer("IdentifySampleLocation", 
                                              IdentifyLocationAction, self.sample_location_callback, False) 
    self.sample_location_action_server.start()

    #listender and CvBridge
    self.listener = tf.TransformListener()
    self.bridge = CvBridge()
    
    #Named tuple and array of named tuples to keep a history of previous images
    self.SampleLocation = namedtuple("SampleLocation", "image timestamp size location")
    self.location_history = []
    self.image_history = []

    #member variables used to construct named tuples
    self.largest_area = None
    self.best_sample_location = None
    self.sample_location_image = None
    self.uv = None


  def sample_location_callback(self, goal):

    self.largest_area = None
    self.best_sample_location = None
    self.sample_location_image = None
    self.uv = None

    if len(self.image_history) > 0 and goal.num_images > 0:
      if goal.num_images > len(self.image_history):
        print("Goal is larger than total images recorded, only " + str(len(self.image_history)) + " images will be processed.")
      subset = self.image_history[-goal.num_images:]
      for i in subset:
        sample_points_2d, contour_areas, cv_image = self.identify_dark_sample_spots(i[0])
        sample_point_3d, sample_point_area, uv = self.get_3d_sample_point(i[1], sample_points_2d, contour_areas)
        if sample_point_area > self.largest_area and sample_point_3d is not None:
          self.largest_area = sample_point_area
          self.best_sample_location = sample_point_3d
          self.sample_location_image = cv_image
          self.uv = uv

      if self.best_sample_location == None:
        print("could not find valid sample")
        self.sample_location_action_server.set_succeeded(IdentifyLocationResult(success=False, sample_location=[]))
      else:
        print("SUCCESS")
        print(self.largest_area, self.best_sample_location)
        self.publish_chosen_location_image()
        self.sample_location_action_server.set_succeeded(IdentifyLocationResult(success=True, sample_location=self.best_sample_location ))
        
    else:
      print("No images have been recorded")
      self.sample_location_action_server.set_succeeded(IdentifyLocationResult(success=False, sample_location=[]))

      
  def site_imaging_callback(self, rect_img_msg, points2_msg):
    #get both messages and save them to our image history for later processing
    self.image_history.append((rect_img_msg, points2_msg))

    #prevent our image history from getting too large saves most recent 50 images
    if len(self.image_history) > 50:
      self.image_history = self.image_history[-50:]
    print("HERE")


  def publish_chosen_location_image(self):
    cv.circle(self.sample_location_image, self.uv, 7, (0,255,0),-1)
    try:
      image = self.bridge.cv2_to_imgmsg(self.sample_location_image, "bgr8")
    except CvBridgeError, err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      return
    self.publish_photo.publish(image)


  def identify_dark_sample_spots(self, location_image):
    try:
      image = self.bridge.imgmsg_to_cv2(location_image, "bgr8")
    except CvBridgeError, err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      return None, None, None

    #filtering and finding contours of dark spots on self.image
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 15, 255,cv.THRESH_BINARY_INV)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours.sort(reverse=True, key=cv.contourArea)

    sample_points_2d = []
    contour_areas = []
    #draw all contours over a certain size; 5000 seems good for now
    for i in contours:
      area = cv.contourArea(i)
      if area >= 5000 and area <= 1000000:
        moment = cv.moments(i)
        cx = int(moment["m10"] / moment["m00"])
        cy = int(moment["m01"] / moment["m00"])
        cv.drawContours(image, [i], 0, 255, 3)
        sample_points_2d.append((cx, cy))
        contour_areas.append(area)
      if len(contour_areas) >= 5:
        break

    return sample_points_2d, contour_areas, image


  def get_3d_sample_point(self, point_cloud, sample_points_2d, contour_areas):
    
    if sample_points_2d:
      site_coordinate = None
      index = 0
      for i in sensor_msgs.point_cloud2.read_points(point_cloud, uvs=sample_points_2d):
        if(np.isnan(i[0]) or np.isnan(i[1]) or np.isnan([2])):
          index+=1
        else:
          site_coordinate = (i[0], i[1], i[2])
          break

      if site_coordinate == None:
        print("COULDNT FIND")
        return None, None

    else:
      print("No sample points found")
      return None, None
      
    identified_location = PointStamped()
    identified_location.header.frame_id = "StereoCameraLeft_optical_frame"
    identified_location.header.stamp = rospy.Time()
    identified_location.point.x = site_coordinate[0]
    identified_location.point.y = site_coordinate[1] 
    identified_location.point.z = site_coordinate[2]

    try:
      self.listener.waitForTransform("StereoCameraLeft_optical_frame", "base_link", rospy.Time.now(), rospy.Duration(5))
      transformed_location = self.listener.transformPoint("base_link", identified_location)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("HAD AN EXCEPTION THROWN")
      return None, None

    sample_point = [transformed_location.point.x, transformed_location.point.y, transformed_location.point.z]

    return sample_point, contour_areas[index], sample_points_2d[index]

if __name__ == '__main__':
  rospy.init_node('test_node', argv=sys.argv)
  IdentifySampleLocation()
  rospy.spin()
