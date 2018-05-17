#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan

class ScanProcessor:
    RAW_SCAN_TOPIC = rospy.get_param("/scan_processor/scan_topic")
    PROCESSED_SCAN_TOPIC = rospy.get_param("/scan_processor/scan_processed_topic")
    ANGLE_OFFSET = rospy.get_param("/scan_processor/laser_angle_offset")

    def __init__(self):
        self.scan_pub = rospy.Publisher(self.PROCESSED_SCAN_TOPIC, LaserScan, queue_size = 2)
        self.scan_sub = rospy.Subscriber(self.RAW_SCAN_TOPIC, LaserScan, self.callback, queue_size = 2)
        self.last_ranges = None

    def callback(self, data):
        ranges = np.array(data.ranges)
        
        if self.last_ranges is not None:
            invalid_ranges = np.isinf(ranges)
            ranges[invalid_ranges] = self.last_ranges[invalid_ranges]
            ranges[np.isinf(ranges)] = data.range_max
            self.last_ranges = None

            ranges = np.roll(ranges, int(self.ANGLE_OFFSET/data.angle_increment))

            data.ranges=ranges
            self.scan_pub.publish(data)
        else:
            self.last_ranges = ranges

if __name__ == "__main__":
    rospy.init_node('scan_processor')
    scan_processor = ScanProcessor()
    rospy.spin()
