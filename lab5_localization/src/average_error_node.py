#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import time

from utils import CircularArray

class Averager:
	def __init__(self):
		self.average = 0
		self.sub = rospy.Subscriber('/ground_truth_delta', Float32, self.callback)
		self.buf = CircularArray(40 * 20)
		print('Initialized averager!')

	def callback(self, msg):
		self.buf.append(msg.data)
		self.average = self.buf.mean()
		print('Average value:', self.average)

if __name__=="__main__":
  rospy.init_node("averager")
  node = Averager()
  rospy.spin()
