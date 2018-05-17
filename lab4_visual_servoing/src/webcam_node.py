#!/usr/bin/env python2
import os, sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class WebcamNode:
  RGB_TOPIC = rospy.get_param('visual_servoing/rgb_topic')
  RGB_TOPIC_COMPRESSED = '/zed/rgb/image_rect_color_compressed'

  def __init__(self):
    self.pub = rospy.Publisher(self.RGB_TOPIC, Image, queue_size=1)
    self.pub_compressed = rospy.Publisher(self.RGB_TOPIC_COMPRESSED, CompressedImage, queue_size=1)
    self.cap = cv2.VideoCapture(0) # Use webcam.
    self.bridge = CvBridge()

  def run(self):
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
      ret, img = self.cap.read()

      if ret:
        self.pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
      
      rate.sleep()

if __name__ == "__main__":
  rospy.init_node('webcam')
  node = WebcamNode()
  node.run()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
