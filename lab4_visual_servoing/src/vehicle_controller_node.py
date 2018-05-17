#!/usr/bin/env python2

import math
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from point import *

import math

class Controller:
  # Import ROS parameters from the "params.yaml" file.
  DRIVE_TOPIC = rospy.get_param("visual_servoing/drive_topic")
  CONE_POSE_TOPIC = rospy.get_param("visual_servoing/cone_pose_topic")
  WAYPOINT_TOPIC = rospy.get_param("visual_servoing/line_follower/waypoint_topic")
  GOAL_OFFSET = rospy.get_param("visual_servoing/desired_distance") # meters
  TEST = rospy.get_param("visual_servoing/current_test")
  
  # Physical Vehicle Limitations/Controls
  MIN_SPEED = 0.1
  MAX_SPEED = 2.0
  MAX_TURN = math.pi/12
  TURN_SPEED = 1.0
  ANGLE_ERROR_THRESHOLD = 0.1
  DIST_ERROR_THRESHOLD = 0.1
  
  # Physical Vehicle Measurements
  AXLE_SPACING = 0.33
  
  # proportional gain values
  ANGLE_GAIN = 0.4
  DIST_GAIN = 1.0

  def __init__(self):
    # drive publsiher
    self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
    
    # init goal pose
    self.goal_pose = Point(0, 0, 0)
    
    # cone pose subscriber
    if(self.TEST == "cone_park"):
      self.goal_pose_sub = rospy.Subscriber(self.CONE_POSE_TOPIC, PoseStamped, self.goalPoseMsgCallback)
      self.goal_pose.setPose(self.GOAL_OFFSET, 0, 0)
    else:
      self.goal_pose_sub = rospy.Subscriber(self.WAYPOINT_TOPIC, PoseStamped, self.goalPoseMsgCallback)
    
    # drive rate publisher
    self.drive_msg_rate = rospy.Rate(10) # publish new drive msg at 10Hz
    
    # initialize drive msg
    self.drive_msg = AckermannDriveStamped()
    # header init
    self.drive_msg.header.frame_id = "base_link"
    # drive params init
    self.drive_msg.drive.steering_angle = 0.0
    self.drive_msg.drive.speed = 0.0
    # don't limit the steering vel and acceleration of the vehicle
    self.drive_msg.drive.steering_angle_velocity = 0
    self.drive_msg.drive.acceleration = 0
    self.drive_msg.drive.jerk = 0

  def goalPoseMsgCallback(self, msg):
    # collect the rect coords of the cone from the msg
    self.goal_pose.setPose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

  def getDistToGoal(self):
    # return distance to goal
    return self.goal_pose.getVecMagnitude(planar=True)

  def publishDrive(self, angle, speed):
    # publish drive msg to update steering angle, vel stays constant
    self.drive_msg.drive.steering_angle = angle
    self.drive_msg.drive.speed = speed
    self.drive_pub.publish(self.drive_msg)

  def desiredAngleToSteerAngle(self, des_angle):
    # translate the car centroid angle to a drive steering angle
    return -atan2(self.AXLE_SPACING*math.sin(des_angle), ((self.getDistToGoal()/2.0) + (self.AXLE_SPACING/2.0)*math.cos(des_angle)))
  
  @staticmethod
  def getSign(x):
    if(x < 0):
      return -1
    else:
      return 1
    
  def run(self):
    while not rospy.is_shutdown():
      
      # determine the error to goal pt
      if(self.TEST == "cone_park")  
        dist_error = self.goal_pose.getX() - self.GOAL_OFFSET
      else:
        dist_error = self.goal_pose.getX()
        
      # determine correct speed based on error
      if(abs(dist_error) > self.DIST_ERROR_THRESHOLD):
        speed = self.DIST_GAIN * dist_error
        speed = Controller.getSign(speed) * max(min(abs(speed), self.MAX_SPEED), self.MIN_SPEED)
      else:
        speed = 0
      
      # determine the correct steering angle
      angle = self.desiredAngleToSteerAngle(self.ANGLE_GAIN * self.goal_pose.getY())
      angle = Controller.getSign(angle) * min(abs(angle), self.MAX_TURN)
      # reverse steering if we're going in reverse
      angle *= Controller.getSign(speed)
      # if we're too close to the goal already, reverse and correct
      if(speed == 0 and abs(angle) > self.ANGLE_ERROR_THRESHOLD):
        speed = -self.TURN_SPEED
        angle = -angle
        
      # publish the drive msg
      self.publishDrive(angle, speed)
      # publish drive msgs at a consistent rate
      self.drive_msg_rate.sleep()

if __name__ == "__main__":
  rospy.init_node('vehicle_controller')
  controller = Controller()
  controller.run()
