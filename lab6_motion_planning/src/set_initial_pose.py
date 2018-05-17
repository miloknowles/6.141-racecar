#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

if __name__ == "__main__":
	rospy.init_node("initial_pose_publisher_script")
	pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
	print("Created publisher.")
	rospy.sleep(1.0)
	msg = PoseWithCovarianceStamped()
	msg.pose.pose.position.x = 71.86
	msg.pose.pose.position.y = 97.99
	pub.publish(msg)
	print("Published message.")
	rospy.spin()
