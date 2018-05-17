#!/usr/bin/env python2

import math
import rospy
from geometry_msgs.msg import PoseStamped
from point import *

import math

class Planner:
  # Import ROS parameters from the "params.yaml" file.
  WAYPOINTS_TOPIC = rospy.get_param("visual_servoing/line_follower/path_topic")
  GOAL_POSE_TOPIC = rospy.get_param("visaul_servoing/line_follwer/waypoint_topic")
	LOOKFW_DIST = rospy.get_param("visual_servoing/line_follower/look_ahead_distance")

  def __init__(self):
    # waypoints subscriber
    self.waypoints_sub = rospy.Subscriber(self.WAYPOINTS_TOPIC, Path, waypointsMsgCallback)
		# goal pose publisher
		self.goal_point_pub = rospy.Publisher(self.GOAL_POSE_TOPIC, PoseStamped, queue_size=10)

  def waypointsMsgCallback(self, msg):
    # collect the rect coords of the cone from the msg
    waypoints = [Point(pose.position.x, pose.position.y, pose.position.z) for pose in msg.poses]
		dist = [(wp.getVecMagnitude() - self.LOOKFW_DIST) for wp in waypoints]
		goal_pose_index = waypoints.index(min(dist))
		self.publishGoalPose(msg.poses(goal_pose_index))

  def publishGoalPose(self, goal_pose):
    # publish goal pose
		self.goal_point_pub.publish(goal_pose)

if __name__ == "__main__":
  rospy.init_node('trajectory_planner')
  planner = Planner()
  rospy.spin()
