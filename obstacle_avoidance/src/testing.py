#!/usr/bin/env python

# NOTE: must make launch file that:
#   launches teleop
#   launches lab6/launch_all.launch
#   launches this node

import rospy
import time
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion, PointStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

CROSSROAD = {'begin': (71.8789,98.0842),
             'end': (78.8789,98.0842)}

WALL = {'begin': (67.9545593262, 89.8748474121),
        'end': (73.3967437744, 88.8627624512)}

PATH = {'begin' :(73.745300293, 98.2885742188),
        'end':(67.1408157349, 85.4581298828)}

class Tester(object):
    def __init__(self, iters=5):
        self.planning_start_time = 0
        self.planning_end_time = 0

        self.path_length = 0

        self.drive_start_time = 0
        self.drive_end_time = 0

        self.iters = iters
        self.cur_test = 1

        self.planning_times = []
        self.path_lengths = []
        self.drive_times = []

        self.method = "RRT*"
        self.last_odom = None
        self.test_collision = False
        self.stuck = False
        self.stuck_time = 0

        self.failures = 0
        self.successes = 0

        self.cur_drive = None
        self.place_tool_pub = rospy.Publisher('/headless_sim_rviz/place',
                                              String,
                                              queue_size=10)
        self.pose_pub = rospy.Publisher('/initialpose',
                                        PoseWithCovarianceStamped,
                                        queue_size=10)
        self.point_pub = rospy.Publisher('/clicked_point',
                                         PointStamped,
                                         queue_size=10)
        self.test_data_sub = rospy.Subscriber('/test_data', String, self.data_cb, queue_size=10)

        self.test_pub = rospy.Publisher('/test_data', String, queue_size=10)
        self.odom_sub = rospy.Subscriber('/vesc/odom', Odometry, self.odom_cb, queue_size=10)
        self.drive_sub = rospy.Subscriber('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, self.drive_sub)
        time.sleep(0.5)

        self.init_car()

    def init_car(self):
        rospy.loginfo("==== Running test {} of {} ====".format(self.cur_test, self.iters))
        # put the car in the right spot
        rospy.loginfo("    Replacing car...")
        self.place_tool(CROSSROAD['begin'], CROSSROAD['end'])
        # orient the localizer
        time.sleep(1)
        rospy.loginfo("    Initial pose for localize...")
        self.send_pose(CROSSROAD['begin'])
        # drop the wall
        time.sleep(1)
        rospy.loginfo("    Build the wall...")
        self.place_2_points(WALL['begin'], WALL['end'])
        time.sleep(1)
        # create the trajectory
        rospy.loginfo("    Drop the pins...")
        self.place_2_points(PATH['begin'], PATH['end'])

    def drive_sub(self, msg):
        self.cur_drive = msg.drive

    def data_cb(self, msg):
        test_output = msg.data
        if "COLLISION" in test_output:
            self.reset_and_next(False)

        if test_output == "PLANNING START":
            rospy.loginfo("   +Start planning")
            self.planning_start_time = rospy.get_time()

        if test_output == "PLANNING END":
            self.planning_end_time = rospy.get_time()
            rospy.loginfo("   +Finish planning [{:f}s]".format(self.planning_end_time-self.planning_start_time))

        if "PATH_LENGTH" in test_output:
            self.path_length = float(test_output[test_output.index(' '): -1])
            rospy.loginfo("   +Found path of length {:f}".format(self.path_length))

        if test_output == "PURSUIT START":
            self.drive_start_time = rospy.get_time()
            self.test_collision = True
            rospy.loginfo("   +Start driving")

        if test_output == "PURSUIT DONE":
            self.test_collision = False
            self.drive_end_time = rospy.get_time()
            rospy.loginfo("   +Finish driving [{:f}s]".format(self.drive_end_time-self.drive_start_time))
            self.reset_and_next(True)

    def odom_cb(self, data):
        if self.test_collision and (rospy.get_time()-self.drive_start_time) > 10.0:
            if self.last_odom is None:
                self.last_odom = data
            else:
                if data.pose.pose.position.x == self.last_odom.pose.pose.position.x:
                    if data.pose.pose.position.y == self.last_odom.pose.pose.position.y:
                        if self.cur_drive is not None and self.cur_drive.speed > 0:

                            if self.stuck:
                                if rospy.get_time() - self.stuck_time > 5:
                                    self.test_pub.publish("DEAD")
                                    self.reset_and_next(False)
                            else:
                                self.stuck = True
                                self.stuck_time = rospy.get_time()
                            return
                self.stuck = False


        self.last_odom = data


    def reset_and_next(self, success):
        self.test_collision = False
        self.stuck = False
        self.planning_times.append(self.planning_end_time - self.planning_start_time)
        self.path_lengths.append(self.path_length)

        if success:
            self.drive_times.append(self.drive_end_time - self.drive_start_time)
            rospy.loginfo("**** PASSED Test {} of {} ****".format(self.cur_test, self.iters))
            self.successes+=1
        else:
            rospy.loginfo("**** FAILED Test {} of {} ****".format(self.cur_test, self.iters))
            self.failures+=1
            rospy.loginfo(" - Collision with wall")

        rospy.loginfo("")

        self.planning_start_time = 0
        self.planning_end_time = 0
        self.path_length = 0
        self.drive_start_time = 0
        self.drive_end_time = 0

        if self.cur_test == self.iters:
            average_planning = sum(self.planning_times)/len(self.planning_times)
            average_drive = sum(self.drive_times)/len(self.drive_times)
            average_len = sum(self.path_lengths)/len(self.path_lengths)

            rospy.loginfo("")
            rospy.loginfo("")
            rospy.loginfo("==== TESTS COMPLETE ====".format(self.iters, self.method))
            rospy.loginfo("{} TRIALS OF {}:".format(self.iters, self.method))
            rospy.loginfo("    PASSED: {}".format(self.successes))
            rospy.loginfo("    FAILED: {}".format(self.failures))
            rospy.loginfo("")
            rospy.loginfo("==== STATS ====".format(self.iters, self.method))
            rospy.loginfo("    AVERAGE PLANNING TIME: {:f}s".format(average_planning))
            rospy.loginfo("    AVERAGE DRIVE TIME: {:f}s".format(average_drive))
            rospy.loginfo("    AVERAGE PATH LENGTH: {:f}s".format(average_len))
            rospy.loginfo("")
            rospy.loginfo("")

            self.planning_times = []
            self.path_lengths = []
            self.driving_times = []
            rospy.signal_shutdown('Tests complete')
            while True:
                pass

        self.cur_test += 1
        self.init_car()


    def place_tool(self, begin, end):
        self.place_tool_pub.publish("placeactivate")
        self.place_tool_pub.publish("placestart ({:.4f},{:.4f})".format(begin[0], begin[1]))
        self.place_tool_pub.publish("placeend ({:.4f},{:.4f})".format(end[0], end[1]))
        self.place_tool_pub.publish("placedeactivate")

    def place_2_points(self, begin, end):
        self.place_point(begin)
        self.place_point(end)


    def place_point(self, point):
        msg = PointStamped()
        msg.header = self.make_header()
        msg.point = Point(point[0], point[1], 0)
        self.point_pub.publish(msg)

    def send_pose(self, point):
        # send pose for initing the localization
        message = PoseWithCovarianceStamped()
        message.header = self.make_header()
        pose = PoseWithCovariance()
        pose.pose = Pose()
        pose.pose.position = Point()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0
        pose.pose.orientation = Quaternion()
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        message.pose = pose
        self.pose_pub.publish(message)

    def make_header(self):
        stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = 'map'
        return header

    def within(self, x, y, e=0.001):
        return x==y


if __name__=="__main__":
    rospy.init_node("tester_node")
    tester = Tester()
    rospy.spin()
