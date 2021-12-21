#!/usr/bin/env python3

#The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
#Research and Simulation can be found in README.md in the root directory of
#this repository.

import rospy
import sys
import tf
import actionlib
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, Point
import sensor_msgs.point_cloud2
from ow_plexil.msg import IdentifyLocationAction, IdentifyLocationGoal, IdentifyLocationFeedback, IdentifyLocationResult
from visualization_msgs.msg import Marker 
import message_filters
from collections import deque

class IdentifySampleLocation:
  #Constant defining the maximum length of our image history deque
  HISTORY_MAX_LENGTH = 50

  def __init__(self):

    #setting up synchronized subscribers for image_rect and point cloud
    rectified_image_subscriber = message_filters.Subscriber("/StereoCamera/left/image_rect_color", Image)
    point_cloud_subscriber = message_filters.Subscriber("/StereoCamera/points2", PointCloud2)
    time_synchronizer = message_filters.TimeSynchronizer([rectified_image_subscriber, point_cloud_subscriber], 10)
    time_synchronizer.registerCallback(self.site_imaging_callback)

    #publishes modified opencv image with the contours and sample location outlined
    self.publish_photo = rospy.Publisher('sample_location', Image, queue_size=10)

    #point visualization
    self.point_visualization_publisher = rospy.Publisher('sample_point_visualization', Marker, queue_size=10)

    #action server setup
    self.sample_location_action_server = actionlib.SimpleActionServer("IdentifySampleLocation",
                                              IdentifyLocationAction, self.sample_location_callback, False)
    self.sample_location_action_server.start()

    #listener and CvBridge
    self.listener = tf.TransformListener()
    self.bridge = CvBridge()

    #member variables
    self.image_history = []
    self.largest_area = None
    self.best_sample_location = None
    self.sample_location_image = None
    self.uv = None


  def sample_location_callback(self, goal):
    '''Action Server, goal contains the number of images to be processed as well as the filter type to be used. 
    Each image gets processed and extracts a list of possible 2d sample coordinates and the corresponding countours.
    The point with the largest contour is then projected to 3d and transformed to the base_link frame. Only the largest
    point will be sent back as a result and the outlined contour and corresponding 2d point on the original image is republished.'''
    #resetting our member variables
    self.largest_area = -1.0
    self.best_sample_location = None
    self.sample_location_image = None
    self.uv = None

    #checking if we have enough images to process and if our goal is greater than 0
    if len(self.image_history) > 0 and goal.num_images > 0:
      if goal.num_images > len(self.image_history):
        rospy.logwarn("Goal is larger than total images recorded, only %s images will be processed.", str(len(self.image_history)))

      #we take a subset of length goal.num_images of the images in image history for  processing
      subset = self.image_history[-goal.num_images:]
      for i in subset:
        #get our list of pixel coordinates then get the corresponding 3d point with the largest contour
        sample_points_2d, contour_areas, cv_image = self.identify_sample_spots(i[0], goal.filter_type)
        sample_point_3d, sample_point_area, uv = self.get_3d_sample_point(i[1], sample_points_2d, contour_areas)
        #save the 3d point info with the largest contour area
        if sample_point_3d is not None and sample_point_area > self.largest_area:
          self.largest_area = sample_point_area
          self.best_sample_location = sample_point_3d
          self.sample_location_image = cv_image
          self.uv = uv

      #if succesful we return our point, otherwise we return an empty list
      if self.best_sample_location == None:
        rospy.loginfo("Could not find valid sample location")
        self.sample_location_action_server.set_succeeded(IdentifyLocationResult(success=False, sample_location=None))
      else:
        #publishes the modified image showing our sample location choice
        self.publish_chosen_location_image()
        self.visualize_sample_point(self.best_sample_location)
        self.sample_location_action_server.set_succeeded(IdentifyLocationResult(success=True, sample_location=self.best_sample_location ))

    else:
      rospy.loginfo("No images have been recorded")
      self.sample_location_action_server.set_succeeded(IdentifyLocationResult(success=False, sample_location=None))


  def site_imaging_callback(self, rect_img_msg, points2_msg):
    '''Synchronizes the pointcloud and the rectified image topic and saves them as a tuple to the image_history list.
    Starts to delete old messages at a length of 50.'''
    #get both messages and save them to our image history for later processing
    self.image_history.append((rect_img_msg, points2_msg))

    #prevent our image history from getting too large saves most recent HISTORY_MAX_LENGTH images
    if len(self.image_history) > self.HISTORY_MAX_LENGTH:
      self.image_history = self.image_history[-self.HISTORY_MAX_LENGTH:]

  def publish_chosen_location_image(self):
    '''Publishes the original image with our drawn on contours and chosen sample location. 
    Used to visualize the chosen sample point.'''
    #draws a circle where our chosen sample location is located
    cv.circle(self.sample_location_image, self.uv, 7, (0,255,0),-1)
    #converts back to a ros image msg before publishing
    try:
      image = self.bridge.cv2_to_imgmsg(self.sample_location_image, "bgr8")
    except CvBridgeError as err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      return
    self.publish_photo.publish(image)


  def identify_sample_spots(self, location_image, filter_type):
    '''Uses OpenCv to process the given image. For filter type Brown we isolate a range of brown colors 
    on the image and draw the contours. For Dark filter (the default filter) we isolate the darkest spots 
    in the image (usually shadows) and find the countours. The 2d center location is taken for these contours
    and are sorted in descending order, we then draw/store the largest 5 contours/center points to return.'''
    try:
      image = self.bridge.imgmsg_to_cv2(location_image, "bgr8")
    except CvBridgeError as err:
      rospy.logerror("CV Bridge error: {0}".format(err))
      return None, None, None

    #defaults to Dark Spot if argument doesnt match or none is given
    if filter_type.lower() == "brown":
     #filtering brown spots on image
      hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
      lower = np.array([0,16,32])
      upper = np.array([19,255,255])
      thresh = cv.inRange(hsv, lower, upper)
    elif filter_type.lower() == "dark":
      #filtering dark spots on image
      gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
      ret, thresh = cv.threshold(gray, 15, 255,cv.THRESH_BINARY_INV)
    else:
      rospy.logerror("Unknown/Unsupported filter type specified!")
      return None, None, None

    #find contours and sort them largest to smallest
    contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours.sort(reverse=True, key=cv.contourArea)
    sample_points_2d = []
    contour_areas = []

    #draw all contours over a certain size; 5000 seems good for now, only taking the largest 5
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
    '''Given a list of 2d points we find the corresponding 3d point within the pointcloud. 
    Takes the first valid point (largest contour with valid point) and transforms it into the
    base_link frame. Returns the new 3d sample point, its corresponding contour and the original
    2d location on the image.'''
    if sample_points_2d:
      site_coordinate = None
      index = 0
      #finds corresponding 3d point in point cloud from 2d pixel point takes the largest (first valid) point
      for i in sensor_msgs.point_cloud2.read_points(point_cloud, uvs=sample_points_2d):
        if np.isnan(i[0]) or np.isnan(i[1]) or np.isnan(i[2]):
          #recording the index so that we can also get the corresponding contour
          index+=1
        else:
          site_coordinate = (i[0], i[1], i[2])
          break
      #making sure our site_coordinate is not none
      if site_coordinate == None:
        return None, None, None
    else:
      return None, None, None

    #create a point stamped message in current frame for transformation to base_link frame
    identified_location = PointStamped()
    identified_location.header.frame_id = "StereoCameraLeft_optical_frame"
    identified_location.header.stamp = rospy.Time()
    identified_location.point.x = site_coordinate[0]
    identified_location.point.y = site_coordinate[1]
    identified_location.point.z = site_coordinate[2]

    try:
      #wait for listener then transform point to base_link frame
      self.listener.waitForTransform("StereoCameraLeft_optical_frame", "base_link", rospy.Time.now(), rospy.Duration(5))
      transformed_location = self.listener.transformPoint("base_link", identified_location)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo("Could not transform point to base_link frame")
      return None, None, None

    
    sample_point = Point()
    sample_point = transformed_location.point
    #returns our sample point, its contour and the original uv of the point
    return sample_point, contour_areas[index], sample_points_2d[index]

  def visualize_sample_point(self, sample_point):
    '''Publishes a marker showing the location of our sample point'''
    #header and action info
    visualized_point = Marker()
    visualized_point.id = 0
    visualized_point.header.frame_id = 'base_link'
    visualized_point.header.stamp = rospy.Time()
    visualized_point.type = visualized_point.SPHERE
    visualized_point.action = visualized_point.ADD

    #scaling and color
    visualized_point.scale.x = 0.1
    visualized_point.scale.y = 0.1
    visualized_point.scale.z = 0.1
    visualized_point.color.r = 0.0
    visualized_point.color.g = 1.0
    visualized_point.color.b = 0.0
    visualized_point.color.a = 1.0

    #point location
    visualized_point.pose.position.x = sample_point.x
    visualized_point.pose.position.y = sample_point.y
    visualized_point.pose.position.z = sample_point.z

    # Add the marker, and publish it out.
    self.point_visualization_publisher.publish(visualized_point)


if __name__ == '__main__':
  rospy.init_node('test_node', argv=sys.argv)
  IdentifySampleLocation()
  rospy.spin()
