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

    #action server setup
    self.sample_location_action_server = actionlib.SimpleActionServer("identify_sample_location", 
                                              IdentifyLocationAction, self.sample_location_callback, False) 
    self.sample_location_action_server.start()

    #point visualization
    self.publisher = rospy.Publisher('breadcrumbs', MarkerArray, queue_size=10)
    self.breadcrumbs = MarkerArray()

    #publishes modified opencv image with the contours and sample location outlined
    self.publish_photo = rospy.Publisher('sample_location', Image, queue_size=10)

    #listender and CvBridge
    self.listener = tf.TransformListener()
    self.bridge = CvBridge()
    
    #Named tuple and array of named tuples to keep a history of previous images
    self.SampleLocation = namedtuple("SampleLocation", "image timestamp size location")
    self.location_history = []
    self.image_history = []

    #member variables used to construct named tuples
    self.largest_area = None
    self.sample_location = None


  def sample_location_callback(self, goal):
    #makes sure callbacks have finished processing
    if len(self.image_history) >= goal.num_images:
      subset = self.image_history[-goal.num_images:]
      for i in subset:
        sample_points_2d, contour_areas = self.identify_dark_sample_spots(i[0])
        sample_point_3d, sample_point_area = self.get_3d_sample_point(i[1], sample_points_2d, contour_areas)
        if(sample_point_area > self.largest_area):
          self.largest_area = sample_point_area
          self.sample_location = sample_point_3d
        print(sample_point_3d, sample_point_area)
      print("COMPLETE")
      print(self.largest_area, self.sample_location)
      self.largest_area = None
      self.sample_location = None
        
    else:
      print("Not enough images for goal")
      self.sample_location_action_server.set_succeeded(IdentifyLocationResult(success=False))
      
  def site_imaging_callback(self, rect_img_msg, points2_msg):
    #get both messages and save them to our image history for later processing
    self.image_history.append((rect_img_msg, points2_msg))

    #prevent our image history from getting too large saves most recent 50 images
    if len(self.image_history) > 50:
      self.image_history = self.image_history[-50:]
    print("HERE")


  def publish_chosen_location_image(self):
    try:
      self.image = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
    except CvBridgeError, err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      self.image = None

    self.publish_photo.publish(self.image)


  def identify_dark_sample_spots(self, location_image):
    try:
      image = self.bridge.imgmsg_to_cv2(location_image, "bgr8")
    except CvBridgeError, err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      return

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
        sample_points_2d.append((cx, cy))
        contour_areas.append(area)
      if len(valid_contours) >= 5:
        break

    return sample_points_2d, contour_areas


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
        return

    else:
      print("No sample points found")
      return
      
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
      
    print("PRE:")
    print(identified_location)
    print("POST:")
    print(transformed_location)

    return transformed_location, contour_areas[index] 


  def vizualize_sample_point(self):
    identified_location = PointStamped()
    identified_location.header.frame_id = "StereoCameraLeft_optical_frame"
    identified_location.header.stamp = rospy.Time()
    identified_location.point.x = x
    identified_location.point.y = y
    identified_location.point.z = z
    print(identified_location)
    destination = self.listener.transformPoint("base_link", identified_location)
    print(destination)
    print(self.stereo_camera_model.project3dToPixel([x,y,z]))

    print("MADE IT2")

    crumb = Marker()

    # Each of the markers has to have a unique ID number.
    crumb.id = 0

    # Set the frame ID, type.
    crumb.header.frame_id = 'base_link'
    crumb.header.stamp = rospy.Time()
    crumb.type = crumb.SPHERE

    crumb.action = crumb.ADD

    crumb.scale.x = 0.1
    crumb.scale.y = 0.1
    crumb.scale.z = 0.1

    crumb.color.r = 0.0
    crumb.color.g = 1.0
    crumb.color.b = 0.0
    crumb.color.a = 1.0

    # Copy the transformed pose.  This is already in the map frame, after the TransformPose call.
    crumb.pose.position.x = destination.point.x
    crumb.pose.position.y = destination.point.y
    crumb.pose.position.z = destination.point.z

    # Add the marker to the marker array, and publish it out.
    self.breadcrumbs.markers.append(crumb)
    self.publisher.publish(self.breadcrumbs)



if __name__ == '__main__':
  rospy.init_node('test_node', argv=sys.argv)
  IdentifySampleLocation()
  rospy.spin()
