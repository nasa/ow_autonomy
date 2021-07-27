#!/usr/bin/env python2
import rospy
import image_geometry
import sys
import cv2 as cv
import tf
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
import sensor_msgs.point_cloud2


from visualization_msgs.msg import Marker, MarkerArray

listener = None

class IdentifySampleLocation:

  def __init__(self):
    left_camera_info_subscriber = rospy.Subscriber("/StereoCamera/left/camera_info", CameraInfo, self.left_camera_info_callback)
    right_camera_info_subscriber = rospy.Subscriber("/StereoCamera/right/camera_info", CameraInfo, self.right_camera_info_callback)
    rectified_image_subscriber = rospy.Subscriber("/StereoCamera/left/image_rect", Image, self.rectified_image_callback)
    depth_image_subscriber = rospy.Subscriber("/StereoCamera/points2", PointCloud2, self.depth_image_callback)
    self.publisher = rospy.Publisher('breadcrumbs', MarkerArray, queue_size=10)


    self.bridge = CvBridge()
    self.stereo_camera_model = None
    self.right_info = None
    self.left_info = None
    self.breadcrumbs = MarkerArray()
    self.camera_model_setup()

    self.cx = None
    self.cy = None
    self.depth_image = None

  def camera_model_setup(self):
    #waiting for both righ and left camera info before moving on
    while self.right_info == None or self.left_info == None:
      rospy.sleep(1)

    #initializing our stereo camera model
    self.stereo_camera_model = image_geometry.StereoCameraModel()
    self.stereo_camera_model.fromCameraInfo(self.left_info, self.right_info)
    print("ALSO MADE IT")

  def left_camera_info_callback(self, msg):
    if self.left_info == None:
      self.left_info = msg
      print("HERE")
 
  def right_camera_info_callback(self, msg):
    if self.right_info == None:
      self.right_info = msg
      print("HERE2")
   
  def identify_sites(self, image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 30, 255,cv.THRESH_BINARY_INV)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    max_contour = max(contours, key=cv.contourArea)
    area = cv.contourArea(max_contour)
    x,y,w,h = cv.boundingRect(max_contour)
    for i in contours:
      if(cv.contourArea(i) >= 5000):
        cv.drawContours(image, [i], 0, 255, 3)
        M = cv.moments(i)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv.circle(image, (cx, cy), 7, (0,255,0),-1)

     
    cv.drawContours(image, [max_contour], 0, 255, 3)
    M = cv.moments(max_contour)
    cx = int(M["m10"] / M["m00"])
    self.cx = cx
    cy = int(M["m01"] / M["m00"])
    self.cy = cy
    print(self.cx, self.cy)
    cv.rectangle(image, (x,y), (x+w, y+h), (0,255,0),3)
    cv.circle(image, (cx, cy), 7, (0,255,0),-1)

    cv.imshow("image", image)
    cv.imshow("image2", thresh)
    cv.waitKey(0)
    cv.destroyAllWindows()

  def rectified_image_callback(self, msg):
    try:
      image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
      rospy.logwarn("CV Bridge error: {0}".format(e))
    self.identify_sites(image)
    print("MADE IT")
    
  def depth_image_callback(self, msg):
    point_step = msg.point_step / 8
    height = msg.height
    width = msg.width

    while(self.cy == None):
      rospy.sleep(1)

    t = [self.cx, self.cy]
    
    print(self.cx)
    print(t)
    g = sensor_msgs.point_cloud2.read_points(msg, uvs=[t],skip_nans=False)
    
    for i in g:
      x = i[0]
      y = i[1]
      z = i[2]


    source = PointStamped()
    source.header.frame_id = "StereoCameraLeft_optical_frame"
    source.header.stamp = rospy.Time()
    source.point.x = x
    source.point.y = y
    source.point.z = z
    print(source)
    destination = listener.transformPoint("base_link", source)
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
  listener = tf.TransformListener()
  IdentifySampleLocation()
  camera = image_geometry.StereoCameraModel()
  rospy.spin()
