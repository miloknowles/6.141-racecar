#!/usr/bin/env python2
import math, random
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

# STARTUP COMMANDS
# roslaunch headless_simulator map_server.launch
# roslaunch racecar teleop.launch 
# roslaunch headless_simulator simulate.launch
# rviz (cfg/base.rviz)

def angle_to_index(angle, min_angle, max_angle, angle_increment):
    """ Converts an angle to a list index. Limits to valid indices. """
    angle = max(min_angle, angle)
    angle = min(angle, max_angle)
    return int((angle - min_angle) / angle_increment)

def index_to_angle(index, min_angle, max_angle, angle_increment):
    """ Converts an index to an angle. Limits to the min and max angle. """
    max_idx = int((max_angle - min_angle) / angle_increment)
    if index < 0:
        return min_angle
    elif index > max_idx:
        return max_angle
    else:
        return min_angle + index * angle_increment

def polar_to_xy(distance, theta):
    """
    Converts the polar coordinate given by (r, theta) into [x, y].
    """
    return [distance * math.cos(theta), distance * math.sin(theta)]

def linear_regression_coeff(x, y):
    """
    Returns the slope and y-intercept of the best fit line.
    """
    n = np.size(x)
    xbar, ybar = np.mean(x), np.mean(y)
    covariance = np.sum((x - xbar) * (y - ybar))
    variance = np.sum((x - xbar) ** 2)

    # Avoid zero division errors.
    if variance == 0:
        return float('inf'), 0
    else:
        slope = covariance / variance
        intercept = ybar - slope*xbar
        return slope, intercept

def fit_line(x1, y1, x2, y2):
    m = (y2-y1) / (x2-x1)
    b = y2 - m*x2
    return m, b

def point_distance(x, y, slope, offset):
    a, b, c = slope, -1, offset
    return abs(a*x + b*y + c) / math.sqrt(a**2 + b**2)

class WallFollower:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    # GAINS
    kP_error_to_rate = 1.7
    kP_rate_to_steer = 0.2
    kP_anticipate = 6

    def __init__(self):
        self.drive_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.scan_subscriber = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        self.marker_publisher = rospy.Publisher('/debug/wall', Marker, queue_size=10)
        self.debug_scan_publisher = rospy.Publisher('/debug/scan', LaserScan, queue_size=10)
        self.drive_seq = 0
        self.current_dist_estimate = 10
        self.x_lookahead = self.VELOCITY * 3

    def slice_laser_scan_front(self, scan_msg):
        min_angle, max_angle = -math.pi/10, math.pi/10
        min_idx = angle_to_index(min_angle, scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        max_idx = angle_to_index(max_angle, scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        return min_idx, max_idx, min_angle, max_angle

    def slice_laser_scan_lateral(self, scan_msg, dist_estimate=10):
        # Get only a portion of the laser scan, based on the side of the wall.
        center_angle = math.pi/4 if self.SIDE == 1 else -math.pi/4
        angle_delta = math.pi / 2 # The car "sees" this much angle sweep of the wall its trying to follow.

        if self.SIDE == 1: # left wall
            min_angle = math.pi / 2 - 0.8 * math.atan(self.x_lookahead / dist_estimate)
            max_angle = math.pi / 2
        else:
            min_angle = -math.pi / 2
            max_angle = -math.pi / 2 + 0.8 * math.atan(self.x_lookahead / dist_estimate)

        # print('Min angle:', min_angle, 'Max angle:', max_angle)
        min_idx = angle_to_index(min_angle, scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        max_idx = angle_to_index(max_angle, scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        return min_idx, max_idx, min_angle, max_angle

    def convert_scan_xy(self, scan_msg, min_idx=None, max_idx=None, min_angle=None, max_angle=None, publish=False):
        """
        Converts a scan message into an array of [x,y] pairs.
        """
        if min_idx == None: min_idx = 0
        if max_idx == None: len(scan_msg.ranges)
        if min_angle == None: min_angle = scan_msg.angle_min
        if max_angle == None: max_angle = scan_msg.angle_max

        # Slice the ranges and ignore points that are at the max range.
        ranges = np.array(scan_msg.ranges)[min_idx:max_idx]

        # Publish a debug scan msg.
        if publish: self.publish_filtered_scan(ranges, min_angle, max_angle, scan_msg.angle_increment)

        ranges = ranges[ranges < (scan_msg.range_max - 0.5)]
        angles = np.arange(len(ranges)) * scan_msg.angle_increment + min_angle
        xy_points = np.array([polar_to_xy(ranges[i], angles[i]) for i in range(len(ranges))])
        return xy_points

    def get_wall_estimate_regression(self, scan_msg, xy_points):
        """
        Given a scan message, returns the bearing (rad) and offset (m).
        Assumes a FLU body frame. Bearing angles are CCW around z-axis.
        """
        if len(xy_points) > 0:
            slope, intercept = linear_regression_coeff(xy_points[:,0], xy_points[:,1])

            # Get the bearing and offset of the wall.
            theta = math.atan(slope)
            distance = abs(intercept * math.cos(theta))
            # distance = point_distance(0, 0, theta, intercept)
            return theta, distance
        else:
            return 0, self.DESIRED_DISTANCE

    def get_wall_estimate_ransac(self, scan_msg, xy_points):
        # RANSAC parameters.
        n = 5
        ransac_iters = 50
        best_inliers = 0
        best_params = [0, 0]
        inlier_threshold = 0.02

        if len(xy_points) > n:
            # For each iteration, sample n points and fit a line to them.
            for r_iter in range(ransac_iters):
                idx1 = random.randint(0, len(xy_points)-1)
                idx2 = random.randint(0, len(xy_points)-1)

                # Ensure indices are unique.
                while idx1 == idx2:
                    idx1 = random.randint(0, len(xy_points)-1)
                    idx2 = random.randint(0, len(xy_points)-1)

                x1, y1, x2, y2 = xy_points[idx1][0], xy_points[idx1][1], xy_points[idx2][0], xy_points[idx2][1]
                slope, intercept = fit_line(x1, y1, x2, y2)

                num_inliers = 0
                for pt in xy_points:
                    dist = point_distance(pt[0], pt[1], slope, intercept)
                    if dist < inlier_threshold: num_inliers += 1

                if num_inliers > best_inliers:
                    best_inliers = num_inliers
                    best_params = [slope, intercept]

            m, b = best_params[0], best_params[1]
            theta = math.atan(m)
            distance = point_distance(0, 0, m, b)
            return theta, distance
        else:
            return 0, self.DESIRED_DISTANCE

    def publish_wall_estimate(self, theta, offset):
        """ Publishes the perceived location of the wall for debugging. """
        marker = Marker()
        marker.header.frame_id = "/base_link";
        marker.header.stamp = rospy.Time.now()
        marker.id = 0;
        marker.type = 0
        marker.action = 0
        marker.pose.position.x = 0
        marker.pose.position.y = offset * self.SIDE / math.cos(theta)
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = math.sin(theta / 2);
        marker.pose.orientation.w = math.cos(theta / 2);
        marker.scale.x = self.x_lookahead;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        self.marker_publisher.publish(marker)

    def publish_filtered_scan(self, ranges, min_angle, max_angle, angle_increment):
        """ Display the portion of the scan that the car is using. """
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.angle_min = min_angle
        msg.angle_max = max_angle
        msg.angle_increment = angle_increment
        msg.range_min = 0
        msg.range_max = 10
        msg.ranges = ranges
        self.debug_scan_publisher.publish(msg)

    def scan_callback(self, msg):
        """
        Called every time a new scan message is received.
        """
        # Estimate the side wall pose.
        side_bounds = self.slice_laser_scan_lateral(msg, dist_estimate=self.current_dist_estimate)
        xy_points_side = self.convert_scan_xy(msg, side_bounds[0], side_bounds[1], side_bounds[2], side_bounds[3], publish=True)
        bearing, offset = self.get_wall_estimate_ransac(msg, xy_points_side)

        # Publisher the wall as an arrow.
        self.publish_wall_estimate(bearing, offset)

        # Set the current distance estimate to get better scan slices.
        self.current_dist_estimate = max(offset, 0.2) # Limit from going to zero.

        # Check for a wall in front of the car.
        front_bounds = self.slice_laser_scan_front(msg)
        xy_points_front = self.convert_scan_xy(msg, front_bounds[0], front_bounds[1], front_bounds[2], front_bounds[3])

        # Calculate the desired theta rate.
        # kP_error_to_rate = 1.7

        # Distance term tries to make the car match the desired distance from the wall.
        distance_term = 0.3 * self.SIDE*math.tanh(offset-self.DESIRED_DISTANCE)

        # Bearing term tries to make the car drive parallel to the wall.
        bearing_term = 0.7 * math.sin(bearing)

        omega_desired = self.kP_error_to_rate * (bearing_term + distance_term)

        # If a wall is directly ahead, start to interpolate into a turn.
        if len(xy_points_front) > 10:
            front_wall_dist = np.mean(xy_points_front, axis=0)[0]

            anticipate_threshold = 2.0 * self.DESIRED_DISTANCE
            # kP_anticipate = 6.0

            if front_wall_dist < anticipate_threshold:
                front_wall_term = -1 * self.SIDE * self.kP_anticipate * (anticipate_threshold - (front_wall_dist-self.DESIRED_DISTANCE)) / anticipate_threshold
                omega_desired += front_wall_term

        # Calculate the necessary steering to achieve omega.
        # kP_rate_to_steer = 0.2
        steering_angle = self.kP_rate_to_steer * omega_desired

        # Publish the drive message.
        drive_msg = AckermannDriveStamped()
        drive_msg.header.seq = self.get_drive_seq()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.steering_angle_velocity = 0
        self.drive_publisher.publish(drive_msg)

    def get_drive_seq(self):
        self.drive_seq += 1
        return self.drive_seq

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
