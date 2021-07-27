#!/usr/bin/env python2
import rospy
import sys
import cv2 as cv
import tf
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
import sensor_msgs.point_cloud2
import message_filters
from collections import namedtuple


from visualization_msgs.msg import Marker, MarkerArray

class IdentifySampleLocation:

  def __init__(self):

    rectified_image_subscriber = message_filters.Subscriber("/StereoCamera/left/image_rect", Image)
    point_cloud_subscriber = message_filters.Subscriber("/StereoCamera/points2", PointCloud2)
    time_synchronizer = message_filters.TimeSynchronizer([rectified_image_subscriber, point_cloud_subscriber], 10)
    time_synchronizer.registerCallback(self.site_imaging_callback)
    #visualization
    self.publisher = rospy.Publisher('breadcrumbs', MarkerArray, queue_size=10)
    self.breadcrumbs = MarkerArray()

    listener = tf.TransformListener()
    self.bridge = CvBridge()

    self.SampleLocation = namedtuple("SampleLocation", "image timestamp size location")
    self.location_history = []

    self.cx = None
    self.cy = None
    self.image = None
    self.size = None
    self.location = None
   
  def site_imaging_callback(self, rect_img_msg, points2_msg):
    try:
      self.image = self.bridge.imgmsg_to_cv2(rect_img_msg, "bgr8")
    except CvBridgeError, err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      return

    print("Identifying sample site")
    self.identify_sample_location()
    print("Getting 3d point")
    self.get_3d_sample_point(points2_msg)

    #add new location
    new_location = self.SampleLocation(self.image, rect_img_msg.header.stamp.secs, self.size, self.location)
    self.location_history.append(new_location)

    print(self.location_history)

  def publish_chosen_location_image(self):
     try:
      self.image = self.bridge.cv2_to_imgmsg(self.image, "passthrough")
    except CvBridgeError, err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      self.image = None


  def identify_sample_location(self):
    #filtering and finding contours of dark spots on self.image
    gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 30, 255,cv.THRESH_BINARY_INV)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    #draw all contours over a certain size; 5000 seems good for now
    for i in contours:
      if(cv.contourArea(i) >= 5000):
        cv.drawContours(self.image, [i], 0, 255, 3)
     
    #find max contour and draw its contour and bounding box
    max_contour = max(contours, key=cv.contourArea)
    self.size = cv.contourArea(max_contour)
    x,y,w,h = cv.boundingRect(max_contour)
    cv.rectangle(self.image, (x,y), (x+w, y+h), (0,255,0),3)
    cv.drawContours(self.image, [max_contour], 0, 255, 3)

    #Find the center of the maximum contour in pixel coordinates
    moment = cv.moments(max_contour)
    self.cx = int(moment["m10"] / moment["m00"])
    self.cy = int(moment["m01"] / moment["m00"])

    #draw a circle on the self.image to show our proposed dig site
    cv.circle(self.image, (self.cx, self.cy), 7, (0,255,0),-1)

  def get_3d_sample_point(self, point_cloud):
    site_coordinate = None
    for i in sensor_msgs.point_cloud2.read_points(point_cloud, uvs=[(self.cx, self.cy)],skip_nans=False):
      if(np.isnan(i[0]) or np.isnan(i[1]) or np.isnan([2])):
        print("FOUND A NAN")
        self.location = None
        return
      else:
        site_coordinate = (i[0], i[1], i[2])
        print(site_coordinate)

    if(site_coordinate == None):
      print("COULDNT FIND")
      self.location = None
      return
      
    identified_location = PointStamped()
    identified_location.header.frame_id = "StereoCameraLeft_optical_frame"
    identified_location.header.stamp = rospy.Time()
    identified_location.point.x = site_coordinate[0]
    identified_location.point.y = site_coordinate[1] 
    identified_location.point.z = site_coordinate[2]

    try:
      listener.waitForTransform("StereoCameraLeft_optical_frame", "base_link", rospy.Time.now(), rospy.Duration(5))
      transformed_location = listener.transformPoint("base_link", identified_location)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("HAD AN EXCEPTION THROWN")
      self.location = None
      
    print("PRE:")
    print(identified_location)
    print("POST:")
    print(transformed_location)
    self.location = transformed_location


  def vizualize_sample_point(self):
    identified_location = PointStamped()
    identified_location.header.frame_id = "StereoCameraLeft_optical_frame"
    identified_location.header.stamp = rospy.Time()
    identified_location.point.x = x
    identified_location.point.y = y
    identified_location.point.z = z
    print(identified_location)
    destination = listener.transformPoint("base_link", identified_location)
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