#!/usr/bin/env python

import math

import rospy
import numpy as np

from std_msgs.msg import String, Header, Float32MultiArray, Float32
from sensor_msgs.msg import LaserScan, Imu

from geometry_msgs.msg import (
  Point, Pose, PoseStamped, PoseArray, Quaternion,
  PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped,
  PointStamped, TransformStamped, Vector3, Vector3Stamped
)

from nav_msgs.msg import Odometry

from utils import quaternion_to_angle

class AngleTester():
  
  def __init__(self):
    self.odom_sub  = rospy.Subscriber("/vesc/odom", Odometry, self.odomCB, queue_size=10)
    self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imuCB, queue_size=10)
    self.mag_sub = rospy.Subscriber("/imu/mag", Vector3Stamped, self.magCB, queue_size=10)

    self.odom_angle = 0.0
    self.imu_angle = 0.0
    self.mag_angle = 0.0
    self.x = 0.0
    self.y = 0.0

  def odomCB(self, msg):
    self.odom_angle = quaternion_to_angle(msg.pose.pose.orientation)
    
  def imuCB(self, msg):
    self.imu_angle = quaternion_to_angle(msg.orientation)

  def magCB(self, msg):
    self.mag_angle = math.degrees(math.atan2(msg.vector.y, msg.vector.x))
    self.x = msg.vector.x * 1e5
    self.y = msg.vector.y * 1e5

  def log_data(self):
    rospy.loginfo("ODOM angle: %f; IMU angle: %f; MAG angle: %f; Vec_X: %f; Vec_Y: %f", self.odom_angle, self.imu_angle, self.mag_angle, self.x, self.y)
    
if __name__=="__main__":
  rospy.init_node("angle_tester")
  a = AngleTester()
  rate = rospy.Rate(2)
  while not rospy.is_shutdown():
    a.log_data()
    rate.sleep()

  rospy.spin()
