#!/usr/bin/env python2

import os, sys
from cone_detection import *

import numpy as np
import rospy

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Pose, PoseStamped

from cv_bridge import CvBridge, CvBridgeError

class ConeDetector:
  RGB_TOPIC = rospy.get_param('visual_servoing/rgb_topic')
  DEBUG_IMG_TOPIC = rospy.get_param('visual_servoing/debug_img_topic')
  CONE_POSE_TOPIC = rospy.get_param('visual_servoing/cone_pose_topic')
  CAMERA_INFO_TOPIC = rospy.get_param('visual_servoing/camera_info_topic')

  CAMERA_POSITION_REL_FRONT_BUMPER = [-0.1, 0.06, 0]

  def __init__(self):
    self.image_sub = rospy.Subscriber(self.RGB_TOPIC, Image, self.image_callback)
    self.debug_image_pub = rospy.Publisher(self.DEBUG_IMG_TOPIC, Image, queue_size=1)
    self.cone_pose_pub = rospy.Publisher(self.CONE_POSE_TOPIC, PoseStamped, queue_size=10)
    self.camera_info_sub = rospy.Subscriber(self.CAMERA_INFO_TOPIC, CameraInfo, self.camera_info_callback)
    self.bridge = CvBridge()
    self.camera_info = None
    self.cone_height_at_1meter = rospy.get_param('visual_servoing/cone_height_at_1meter')
    self.desired_distance = rospy.get_param('visual_servoing/desired_distance')

  def camera_info_callback(self, cinfo):
    self.camera_info = cinfo

  def image_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Get the bounding box of the cone.
    # Swap out the cone bounding box functions below.

    bbox = cd_color_segmentation_hsv(cv_image)
    # bbox = cd_color_segmentation_contours(cv_image)

    # Draw the vertical-horizontal bounding box.
    minx, miny = bbox[0]
    maxx, maxy = bbox[1]
    cv2.rectangle(cv_image, (minx, miny), (maxx, maxy), (0, 255, 0), 2)

    # Extract cone height to determine depth.
    cone_height = abs(maxy-miny)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'base_link'

    # If cone disappears, return desired distance.
    if cone_height < 5:
      msg.pose.position.x = self.desired_distance
      self.cone_pose_pub.publish(msg)
      return

    centroid = np.array([(minx+maxx)/2, (miny+maxy)/2, 1])

    K = np.reshape(self.camera_info.K, (3, 3)) # Reshape camera info into a 3x3.

    # Get the centroid of the bbox in camera frame (RDF).
    centroid_3d_camera = np.matmul(np.linalg.inv(K), centroid)

    # Transformation from RDF into FLU.
    T_CAMERA_WORLD = np.array([[0, 0,  1],
                              [-1, 0, 0],
                              [0, -1, 0]])

    centroid_3d_robot = np.matmul(T_CAMERA_WORLD, centroid_3d_camera) * self.cone_height_at_1meter / cone_height

    # Publish the pose of the cone.
    msg.pose.position.x = centroid_3d_robot[0] + self.CAMERA_POSITION_REL_FRONT_BUMPER[0]
    msg.pose.position.y = centroid_3d_robot[1] + self.CAMERA_POSITION_REL_FRONT_BUMPER[1]
    msg.pose.position.z = centroid_3d_robot[2] + self.CAMERA_POSITION_REL_FRONT_BUMPER[2]
    # print('Centroid:', centroid_3d_robot)
    self.cone_pose_pub.publish(msg)

    try:
      self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

if __name__ == "__main__":
  rospy.init_node('cone_detector')
  node = ConeDetector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
