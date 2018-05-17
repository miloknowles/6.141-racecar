#!/usr/bin/env python

import rospy
import numpy as np
import numpy.ma as ma
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import utils as Utils
import time

class FreeSpacePlanner:
    def __init__(self):
        print "HERE"
        # PARAMS
        self.FORWARD_ANGLE = normalize_angle(rospy.get_param("~forward_angle", math.pi))
        self.NEW_PT_ANGLE_THRESH = rospy.get_param("~new_pt_angle_thresh", math.pi/6.0)   # a new pt must beat the current one by this many radians
        self.NEW_PT_DIST_THRESH = rospy.get_param("~new_pt_dist_thresh", 0.5)  # a new pt must beat the current one by this many meters
        self.BLOCKED_TRAJ_THRESH = rospy.get_param("~blocked_traj_thresh", 2.0)
        self.STEP_SIZE = rospy.get_param("~step_size", 5)  # take every nth laser scan
        self.MAX_RANGE_METERS = rospy.get_param("~max_range_meters", 10.5)
        self.MIN_RANGE_METERS = rospy.get_param("~min_range_meters", 0.0)
        self.DEFLECTION_DIST = rospy.get_param("~deflection_dist", 0.4)
        self.COLLISION_RADIUS = rospy.get_param("~collision_radius", 0.45)
        self.MAX_TURN_ANGLE = rospy.get_param("~max_turn_angle", math.pi/3)

        # SUBSCRIBERS
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=1)

        self.inferred_pose_sub = rospy.Subscriber(rospy.get_param("~pose_topic", "/pf/viz/inferred_pose"), PoseStamped, self.inferred_poseCB, queue_size=1)
        print "INIT SUBSCRIBERS"


        # PUBLISHERS
        self.forward_dir_pub = rospy.Publisher(rospy.get_param("~forward_dir_topic", "/forward_dir"), PoseStamped, queue_size = 10)
        self.free_space_path_pub = rospy.Publisher(rospy.get_param("~free_space_path_topic", "/free_space_path"), MarkerArray, queue_size = 10)
        self.path_command_pub = rospy.Publisher(rospy.get_param("~trajectory_topic", '/waypoint_path'), PolygonStamped, queue_size = 10)
        print "INIT PUBLISHERS"


        # VARS
        ## laser vars
        self.lidar_initialized = False
        self.laser_angles = None  # array of laser angles
        self.real_laser_angles_initialized = False
        self.real_laser_angles_size = 0
        self.real_laser_angles = None  # angles in the fixed frame
        self.laser_ranges = None
        self.distances_forward = None
        self.laser_scan_min = None
        self.laser_scan_max = None
        self.laser_scan_inc = None
        self.closest_pt_index = None
        self.points_in_box = None
        self.self_centered_scan = None

        ## pose vars
        self.pose_initialized = None
        self.cur_pose = None
        self.cur_angle = None
        self.forward_dir_delta = None

        ## distnace vars
        self.distances_forward = None

        ## traj vars
        self.best_pt_valid = False
        self.best_pt = None
        self.best_pt_angle = 0
        self.best_pt_dist = 0  # ensure first found point is accepted

        self.deflected_pt = None

    def lidarCB(self, msg):
        if not isinstance(self.laser_angles, np.ndarray):
            print "...Received first LiDAR message"
            self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))[::self.STEP_SIZE]
            self.distances_forward = np.zeros_like(self.laser_angles)
            self.laser_scan_min = msg.angle_min
            self.laser_scan_max = msg.angle_max
            self.laser_scan_inc = msg.angle_increment
            self.lidar_initialized = True

        self.laser_ranges = np.array(msg.ranges[::self.STEP_SIZE])

        if self.pose_initialized:
            self.real_laser_angles = vec_normalize_angle(self.laser_angles + self.cur_angle - self.FORWARD_ANGLE)  # angles in fixed frame
            if self.best_pt is not None:
                self.self_centered_scan = self.laser_angles
                self.points_in_box = np.logical_and(np.absolute(np.multiply(self.laser_ranges, np.sin(self.self_centered_scan))) < self.COLLISION_RADIUS,
                                                    self.laser_ranges < 3)
                self.points_in_box = np.logical_and(self.points_in_box, np.absolute(self.laser_angles) < math.pi/3)
                if not np.any(self.points_in_box):
                    self.closest_pt_index = None

                else:
                    relevant_ranges = self.laser_ranges.copy()
                    relevant_ranges[np.logical_not(self.points_in_box)] = 99
                    #index = np.argsort(self.laser_ranges[self.points_in_box])[0]
                    index = np.argsort(relevant_ranges)[0]
                    self.closest_pt_index = index
                self.deflect_path()
            if not self.real_laser_angles_initialized:
                self.real_laser_angles_size = len(self.real_laser_angles)
                self.real_laser_angles_initialized = True
        #    self.distances_forward = self.laser_ranges * math.cos(self.laser_angles)

    def inferred_poseCB(self, msg):
        t0 = time.time()
        if self.pose_initialized is None:
            print "...Received first pose message"
            self.pose_initialized = True
        self.cur_pose = msg.pose
        self.cur_angle = Utils.quaternion_to_angle(msg.pose.orientation)
        self.forward_dir_delta = self.FORWARD_ANGLE - self.cur_angle

        if self.lidar_initialized and self.pose_initialized:
            if self.real_laser_angles is not None:
                self.find_best_angle()
        print('Loop time:', time.time()-t0)

    def deflect_path(self):
        # use the nearest colliding point to deflect the path accordingly
        if self.closest_pt_index is not None:
            dist = self.laser_ranges[self.closest_pt_index]
            angle = normalize_angle(self.real_laser_angles[self.closest_pt_index] + self.FORWARD_ANGLE)
            angle_rel_car_pose = self.laser_angles[self.closest_pt_index]

            print("angle", angle)
            print("best", self.best_pt_angle)

            deflection_dir = 1 if (sign(self.best_pt_angle) < sign(angle) or
                                    (sign(self.best_pt_angle) == sign(angle) and
                                     self.best_pt_angle - angle > 0)) else -1

            deflection_factor = deflection_dir * min((self.MAX_TURN_ANGLE)/(max(dist, 1.0)**2)* abs(abs(angle_rel_car_pose)-math.pi/4) / (math.pi/4), self.MAX_TURN_ANGLE)
            print("def factor", deflection_factor)
            new_angle = normalize_angle(self.best_pt_angle + deflection_factor)
            print("new angle", new_angle)
            self.deflected_pt = (self.cur_pose.position.x + self.best_pt_dist*math.cos(new_angle),
                                 self.cur_pose.position.y + self.best_pt_dist*math.sin(new_angle))
        else:
            self.deflected_pt = self.best_pt

        path = PolygonStamped()
        path.header = Utils.make_header("map")
        path.polygon = Polygon([Point(self.cur_pose.position.x, self.cur_pose.position.y,0),
                                Point(self.deflected_pt[0], self.deflected_pt[1],0)])

        self.path_command_pub.publish(path)

    def find_best_angle(self):
        abs_real_angles = np.absolute(self.real_laser_angles)
        relevant_ranges = self.laser_ranges.copy()
        relevant_ranges[abs_real_angles > math.pi/2] = 0.5
        abs_real_angles *= -1
        #self.laser_ranges[abs_real_angles > math.pi/2] = 0.5  # don't consider points behind car
        best_dist_index = np.lexsort((abs_real_angles, relevant_ranges))[-1]
        dist = self.laser_ranges[best_dist_index]
        angle = normalize_angle(self.real_laser_angles[best_dist_index]+self.FORWARD_ANGLE)

        # check the scan to the left and scan to the right
        # bias [angle proportional to distance] toward the scan with larger dist

        if self.best_pt_valid:
            # update the best_pt info (but not the actual best_pt) since we've moved closer to it
            self.best_pt_dist = np.hypot(self.cur_pose.position.x-self.best_pt[0], self.cur_pose.position.y - self.best_pt[1])
            self.best_pt_angle = normalize_angle(math.atan2(self.best_pt[1] - self.cur_pose.position.y , \
                                            self.best_pt[0] - self.cur_pose.position.x))
            if  self.best_pt_dist > 11:  # out of range
                print "condition 1"
                self.best_pt_valid = False
            else:
                lidar_min = self.cur_angle + self.laser_scan_min  # verified
                #index = int(np.hypot((self.best_pt_angle+4*math.pi)%(2*math.pi) ,(lidar_min+4*math.pi)%(2*math.pi)) / (self.STEP_SIZE * self.laser_scan_inc))
                positive_best_pt_angle = self.best_pt_angle if self.best_pt_angle > 0 else self.best_pt_angle + 2*math.pi
                positive_lidar_min = lidar_min if lidar_min > 0 else lidar_min + 2*math.pi
                index = int((positive_best_pt_angle - positive_lidar_min)/ (self.STEP_SIZE * self.laser_scan_inc))

                if index > (self.real_laser_angles_size -1) or index < 0:
                    self.best_pt_valid = False
                    print "condition 2"
                else:
                    thresh = self.BLOCKED_TRAJ_THRESH
                    self.best_pt_valid = (abs(self.laser_ranges[index-1] - self.best_pt_dist) < thresh or \
                                    abs(self.laser_ranges[index] - self.best_pt_dist) < thresh or \
                                    abs(self.laser_ranges[index+1] - self.best_pt_dist) < thresh)
                    if not self.best_pt_valid:
                        print "condition 3"

                    #ps = PoseStamped()
                    #ps.header = Utils.make_header("map")
                    #ps.pose.position.x = self.cur_pose.position.x
                    #ps.pose.position.y = self.cur_pose.position.y
                    #ps.pose.orientation = Utils.angle_to_quaternion(self.cur_angle + self.forward_dir_delta)
                    ##ps.pose.orientation = Utils.angle_to_quaternion(self.best_pt_angle if self.best_pt_angle > 0 else self.best_pt_angle)
                    #self.forward_dir_pub.publish(ps)

        #or abs(abs(self.best_pt_angle - self.FORWARD_ANGLE) - \
        #        abs(angle - self.FORWARD_ANGLE)) > self.NEW_PT_ANGLE_THRESH \

        if (not self.best_pt_valid or abs(self.best_pt_dist - dist) > self.NEW_PT_DIST_THRESH):
            print "condition 3"
            self.best_pt_dist = dist
            self.best_pt_angle = angle
            self.best_pt = (self.cur_pose.position.x + dist*math.cos(angle), \
                    self.cur_pose.position.y + dist*math.sin(angle))
            self.best_pt_valid = True

        #left_scan = self.real_laser_angles[best_dist_index-1] if best_dist_index > 0 else 0
        #right_scan = self.real_laser_angles[best_dist_index+1] if best_dist_index < self.real_laser_angles_size else 0
        #best_dist_angle = best_dist_angle + (-1 if left_scan < right_scan else 1) * 0.1*((math.pi/2)/((max(min(left_scan, right_scan), 0))+1))

        if self.best_pt is not None:
            traj = Marker()
            traj.header = Utils.make_header("map")
            traj.ns = "my_namespace"
            traj.id = 0
            traj.type = 5  #LINELIST
            traj.action = 0  # ADD
            traj.pose.position.x = 0
            traj.pose.position.y = 0
            traj.pose.position.z = 0
            traj.pose.orientation.x = 0.0
            traj.pose.orientation.y = 0.0
            traj.pose.orientation.z = 0.0
            traj.pose.orientation.w = 0.0
            traj.scale.x = 0.1
            traj.scale.y = 0.1
            traj.scale.z = 0.1
            traj.color.a = 1.0
            traj.color.r = 1.0
            traj.color.g = 0.0
            traj.color.b = 0.0
            traj.points = [Point(self.cur_pose.position.x,
                                 self.cur_pose.position.y,
                                 0),
                            Point(self.best_pt[0],
                                  self.best_pt[1],
                                  0)]

            collide = Marker()
            collide.header = Utils.make_header("map")
            collide.ns = "my_namespace2"
            collide.id = 0
            collide.type = 8  #POINT CLOUD
            collide.action = 0  # ADD
            collide.pose.position.x = 0
            collide.pose.position.y = 0
            collide.pose.position.z = 0
            collide.pose.orientation.x = 0.0
            collide.pose.orientation.y = 0.0
            collide.pose.orientation.z = 0.0
            collide.pose.orientation.w = 0.0
            collide.scale.x = 0.2
            collide.scale.y = 0.2
            collide.scale.z = 0.2
            collide.color.a = 1.0
            collide.color.r = 1.0
            collide.color.g = 1.0
            collide.color.b = 0.0
            #points = [Point(x[0], x[1], 0) for x in self.pol2cart(self.cur_pose.position.x, self.cur_pose.position.y, self.FORWARD_ANGLE, self.laser_ranges[self.points_in_box], self.real_laser_angles[self.points_in_box])]
            #print self.points_in_box
            #print self.closest_pt_index
            if self.closest_pt_index is not None:
                points = [Point(x[0], x[1], 0) for x in self.pol2cart(self.cur_pose.position.x, self.cur_pose.position.y, self.FORWARD_ANGLE, self.laser_ranges[self.closest_pt_index], self.real_laser_angles[self.closest_pt_index])]
            else:
                points = []
            collide.points = points

            deflected = Marker()
            deflected.header = Utils.make_header("map")
            deflected.ns = "my_namespace3"
            deflected.id = 0
            deflected.type = 5  #LINELIST
            deflected.action = 0  # ADD
            deflected.pose.position.x = 0
            deflected.pose.position.y = 0
            deflected.pose.position.z = 0
            deflected.pose.orientation.x = 0.0
            deflected.pose.orientation.y = 0.0
            deflected.pose.orientation.z = 0.0
            deflected.pose.orientation.w = 0.0
            deflected.scale.x = 0.1
            deflected.scale.y = 0.1
            deflected.scale.z = 0.1
            deflected.color.a = 1.0
            deflected.color.r = 0.5
            deflected.color.g = 1.0
            deflected.color.b = 0.75
            if self.deflected_pt is not None:
                deflected.points = [Point(self.cur_pose.position.x,
                                     self.cur_pose.position.y,
                                     0),
                                Point(self.deflected_pt[0],
                                      self.deflected_pt[1],
                                      0)]
            markers = MarkerArray([collide, traj, deflected])
            self.free_space_path_pub.publish(markers)


    def pol2cart(self, x0, y0, phi0, rho, phi):
        """
        Converts polar laser scans into cartesian coordinates.
        (x0, y0, phi0): The origin of the scan (car pose).
        rho: array of ranges.
        phi: array of angles.
        resolution: the dimension of a map grid cell in meters.
        """
        close_enough = (rho < self.MAX_RANGE_METERS) & (rho > self.MIN_RANGE_METERS)
        forward_only = np.absolute(phi) < math.pi/2.0
        mask = np.logical_and(close_enough, forward_only)
        rho = rho[close_enough]
        phi = phi[close_enough]
        x = rho*np.cos(phi+phi0)+x0
        y = rho*np.sin(phi+phi0)+y0
        return np.stack((x, y), axis=-1)

def normalize_angle(angle):
    #return angle
    return Utils.quaternion_to_angle(Utils.angle_to_quaternion(angle))

def sign(x):
    return -1 if x < 0 else 1

vec_normalize_angle = np.vectorize(normalize_angle)

if __name__=="__main__":
    rospy.init_node("free_space_controller")
    fsp = FreeSpacePlanner()
    rospy.spin()






