#!/usr/bin/env python

import rospy
import numpy as np
import utils
import threading, time, collections, heapq, itertools, math, os
from itertools import count
from utils import Circle, Path, SearchNode, SearchNodeTree, TreeNode

from geometry_msgs.msg import PoseStamped, PolygonStamped, PoseArray, PointStamped, Point32
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, String
from nav_msgs.srv import GetMap

import PlanningLibC as plc
from rrt_star import RRTStarAlgorithm
from fmt_star import FMTStarAlgorithm
from anytree import AnyNode

import cv2
import time

import matplotlib.pyplot as plt

class PathPlanner(object):
    """ Abstract class for path planning """
    def __init__(self):

        # Wait on a ROS Occupancy Grid message to set up the map.
        self.occupancy_grid_msg = self.get_occupancy_grid_msg()
        self.occupancy_grid = plc.OccupancyGrid(self.occupancy_grid_msg)
        self.print_map_stats()

        self.clicked_point_sub = rospy.Subscriber(
            "/clicked_point", PointStamped, self.clicked_pose, queue_size=10
        )

        self.graph_pub = rospy.Publisher('/visualization/graph', Marker, queue_size=10)
        self.wall_pub = rospy.Publisher('/visualization/wall', Marker, queue_size=10)

        self.map_dilated_pub = rospy.Publisher('/map_dilated/', OccupancyGrid, queue_size=5)

        self.line_trajectory = utils.LineTrajectory('visualization')

        # Initially, there is no start or goal.
        self.start = None
        self.goal = None

        # Record the wall
        self.wall_start = None
        self.wall_end = None
        self.wall = False

        # NB temporary
        self.previous_point = None
        self.curr_point = None

        self.dilated = False
        self.test_pub = rospy.Publisher('/test_data', String)

        rospy.loginfo('Initialized path planner node!')

    def print_map_stats(self):
        w = self.occupancy_grid.getMapWidth()
        h = self.occupancy_grid.getMapHeight()
        r = self.occupancy_grid.getMapResolution()
        wm = self.occupancy_grid.getMapWidthMeters()
        hm = self.occupancy_grid.getMapHeightMeters()
        origin = self.occupancy_grid.getOriginPoint()
        rospy.loginfo('MAP INFO: \nwidth=%d cells (%f m) \nheight=%d cells (%f m) \nres=%f m/cell' % (w, wm, h, hm, r))
        rospy.loginfo('MAP ORIGIN: x=%f y=%f' % (origin.x, origin.y))

    def clicked_pose(self, msg):
        print('Clicked pose called.')
        if not self.wall:
            print('Setting wall')
            if self.wall_start is None:
                self.wall_start = (msg.point.x, msg.point.y)
                print('Point 1: ', self.wall_start)
            else:
                self.wall_end = (msg.point.x, msg.point.y)
                self.wall = True
                print('Point 2: ', self.wall_end)
                dist = ((self.wall_start[0]-self.wall_end[0])**2 + \
                        (self.wall_start[1]-self.wall_end[1])**2) ** 0.5
                dir_vec = ((self.wall_end[0]-self.wall_start[0])/dist,
                           (self.wall_end[1]-self.wall_start[1])/dist)
                step = 1.0/(dist*10.0)
                i = 0
                points = []

                while i <= dist:
                    x = self.wall_start[0] + dir_vec[0]*i
                    y = self.wall_start[1] + dir_vec[1]*i
                    points.append((x,y))
                    self.occupancy_grid.setPointValue(plc.Point2f(x, y), 126)
                    i += step

                marker = Marker()
                marker.header = utils.make_header("/map")
                marker.ns = "visualization/wall"
                marker.id = 10
                marker.type = 4 # line strip
                marker.action = 0
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.1
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                for p in points:
                    pt = Point32()
                    pt.x = p[0]
                    pt.y = p[1]
                    pt.z = -0.1
                    marker.points.append(pt)
                self.wall_pub.publish(marker)
                print('Wall set')
                print('Setting Goals')
        else:
            self.previous_point = self.curr_point
            self.curr_point = msg.point
            print(self.previous_point)
            print(self.curr_point)
            if self.previous_point is not None:
                self.compute_plan(self.previous_point, self.curr_point)

    def get_occupancy_grid_msg(self):
        """
        Attempts to retrieve the map from specified map service.
        Returns: a ROS Occupancy Grid message.
        """
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.loginfo("Getting map from service: %s", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        return map_msg

    def set_start_and_goal_points(self, start, goal):
        """
        Sets the start and goal pose.
        start: (plc.Point2f) the xy coords of start.
        goal: (plc.Point2f) the xy coords of goal.
        """
        self.start = start
        self.goal = goal

    def plan_request_callback(self, msg):
        if len(msg.poses) != 2:
            rospy.loginfo('Warning: received a plan request that did not contain two poses.')

            # Set the start and goal poses.
            self.set_start_and_goal_points(plc.Point2f(msg.poses[0].position.x, msg.poses[0].position.y),
                                                                 plc.Point2f(msg.poses[1].position.x, msg.poses[1].position.y))

        self.compute_plan(self.start, self.goal)

    def compute_plan(self, start, goal):
        """ This should be implemented in the child class. """
        raise NotImplementedError

    def publish_graph(self, line_list, duration=0.0, should_publish=True):
        """
        line_list: A list of points, where each pair should be connected by an edge.
        i.e 0->1, 2->3, etc.
        """
        marker = Marker()
        marker.header = utils.make_header("/map")
        marker.ns = 'visualization' + "/trajectory"
        marker.id = 100
        marker.type = 5 # Line list.
        marker.lifetime = rospy.Duration.from_sec(duration)

        if should_publish:
            marker.action = 0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            for p in line_list:
                pt = Point32(p.x, p.y, 0)
                marker.points.append(pt)
        else:
            marker.action = 2

        self.graph_pub.publish(marker)

def make_flamegraph(filterx=None):
    import flamegraph
    perf_log_path = os.path.join(os.path.dirname(__file__), "../tmp/perf2.log")
    flamegraph.start_profile_thread(fd=open(perf_log_path, "w"),
                                    filter=filterx,
                                    interval=0.001)

class RRTStarPathPlanner(PathPlanner):
    def __init__(self):
        # Call super class init.
        super(RRTStarPathPlanner, self).__init__()

    def compute_plan(self, start, goal):
        # Set the start and goal poses now that we have them.

        if not self.dilated:
            # self.occupancy_grid.dilateOccupancyGrid(0.2) # Dilate the map by 20cm before planning.
            height = self.occupancy_grid_msg.info.height
            width = self.occupancy_grid_msg.info.width
            resolution = self.occupancy_grid_msg.info.resolution
            kernel_width = int(0.5 / resolution)
            occupancy_array = np.array(self.occupancy_grid.getOccupancyGridMsg().data).reshape(height, width).astype(bool).astype(float)
            kernel = np.ones((kernel_width, kernel_width))
            new_occupancy_array = cv2.dilate(occupancy_array, kernel) #np.ones(kernel_width, kernel_width))
            self.occupancy_grid_msg.data = new_occupancy_array.flatten().tolist()
            self.occupancy_grid.setOccupancyGrid(self.occupancy_grid_msg)
            self.map_dilated_pub.publish(self.occupancy_grid.getOccupancyGridMsg())
            self.dilated = True

        planner = RRTStarAlgorithm(self.occupancy_grid, plc.Point2f(start.x, start.y), plc.Point2f(goal.x, goal.y),
                                   publish_graph_cb=self.publish_graph)
        self.test_pub.publish("PLANNING START")
        t0 = time.time()
        # Set rewire to True for RRT*, False for RRT.
        path = planner.run(rewire = True, verbose=True, graph = True)
        self.test_pub.publish("PLANNING DONE")
        self.test_pub.publish('Runtime: %f' % (time.time() - t0))
        dist = 0
        prev_pt = None
        for pt in path:
            self.line_trajectory.addPoint(pt)
            if prev_pt is not None:
                dist += ((pt.x-prev_pt.x)**2+(pt.y-prev_pt.y)**2)**0.5
            prev_pt = pt
        self.test_pub.publish("PATH_LENGTH: %f" % dist)
        self.line_trajectory.publish_viz()
        self.line_trajectory.publish_waypoints()
        return path

class FMTStarPathPlanner(PathPlanner):
    def __init__(self):
        super(FMTStarPathPlanner, self).__init__()

    def compute_plan(self, start, goal):
        if not self.dilated:
            # self.occupancy_grid.dilateOccupancyGrid(0.2) # Dilate the map by 20cm before planning.
            height = self.occupancy_grid_msg.info.height
            width = self.occupancy_grid_msg.info.width
            resolution = self.occupancy_grid_msg.info.resolution
            kernel_width = int(0.5 / resolution)
            occupancy_array = np.array(self.occupancy_grid.getOccupancyGridMsg().data).reshape(height, width).astype(bool).astype(float)
            kernel = np.ones((kernel_width, kernel_width))
            new_occupancy_array = cv2.dilate(occupancy_array, kernel) #np.ones(kernel_width, kernel_width))
            self.occupancy_grid_msg.data = new_occupancy_array.flatten().tolist()
            self.occupancy_grid.setOccupancyGrid(self.occupancy_grid_msg)
            self.map_dilated_pub.publish(self.occupancy_grid.getOccupancyGridMsg())
            self.dilated = True

        planner = FMTStarAlgorithm(start, goal, self.occupancy_grid)
        path = planner.runAlgorithm(publish_graph_cb=self.publish_graph)
        for point in path:
            self.line_trajectory.addPoint(point)
        self.line_trajectory.publish_viz()
        line_list = []
        for node in planner.paths_tree.values():
            if node.is_root:
                continue
            parent = node.parent
            line_list.append(planner.free_samples[node.point_index])
            line_list.append(planner.free_samples[parent.point_index])
        self.publish_graph(line_list)
        #print("start", planner.start_pt)
        #print("goal", planner.goal_pt)
        #print("path", [(elem.x,elem.y) for elem in path])
        self.line_trajectory.publish_waypoints()
        return path

if __name__=="__main__":
    rospy.init_node("trajectory_search")
    #pf = RRTStarPathPlanner()
    pf = RRTStarPathPlanner()
    rospy.spin()
