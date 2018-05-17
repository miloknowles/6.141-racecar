#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
import time, os
from utils import LineTrajectory

class BuildTrajectory(object):
    """ Listens for points published by RViz and uses them to build a trajectory. Saves the output to the file system.
    """
    def __init__(self):
        self.save_path = os.path.join(rospy.get_param("~save_path"), time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj")
        self.trajectory = LineTrajectory("/built_trajectory")
        '''
        Insert appropriate subscribers/publishers here
        '''
        self.point_listener = rospy.Subscriber("/clicked_point", PointStamped, self.point_cb, queue_size=1)
        # save the built trajectory on shutdown
        rospy.on_shutdown(self.saveTrajectory)

    def point_cb(self, clicked_point):
        self.trajectory.addPoint(clicked_point.point)
        self.trajectory.publish_start_point()
        self.trajectory.publish_end_point()
        self.trajectory.publish_trajectory()

    def saveTrajectory(self):
        self.trajectory.save(self.save_path)

if __name__=="__main__":
    rospy.init_node("build_trajectory")
    pf = BuildTrajectory()
    rospy.spin()
