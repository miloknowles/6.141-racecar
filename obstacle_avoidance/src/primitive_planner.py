#!/usr/bin/env python

import math, time
import utils
import numpy as np
from anytree import AnyNode
from visualization_msgs.msg import Marker
from geometry_msgs.msg import (
    PoseWithCovarianceStamped, Point32, PoseStamped,
    PolygonStamped, PointStamped, Point)
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid as OGMSG
import rospy
import dubins
import heapq
from occupancy_grid import OccupancyGrid

import PlanningLibC as plc


class Turn_Primitive:
    # Turn primitive static params.
    TIME_STEP = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/time_step")
    TOTAL_TIME = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/total_time")
    MAX_VELOCITY = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/max_velocity")
    WHEELBASE_LENGTH = utils.getParamOrFail("/obstacle_avoidance/pure_pursuit/wheelbase")

    def __init__(self, angle, total_time=None):
        if total_time is not None:
            self.TOTAL_TIME = total_time
        self.steering_angle = angle
        abs_angle = math.fabs(angle)
        slip_speed = max(1.3, 5.86 - 34.54 * abs_angle + 107.42 * (abs_angle**2) - 115.37 * (abs_angle**3))
        self.velocity = min(self.MAX_VELOCITY, slip_speed)
        self.num_steps_per_primitive = int(round(self.TOTAL_TIME / self.TIME_STEP))

    def doPointsCollide(self, poses, occupancy_grid):
        # path_plc = plc.Point2fList()
        # for x, y, theta in poses:
        #     path_plc.append(plc.Point2f(x, y))

        # return occupancy_grid.pathOccupiedVectPoint2f(path_plc)
        return occupancy_grid.pathOccupied(poses)

    def _apply_once(self, start_pose):
        x, y, theta = start_pose
        dx = self.velocity * self.TIME_STEP * math.cos(theta)
        dy = self.velocity * self.TIME_STEP * math.sin(theta)
        dtheta = (self.velocity * self.TIME_STEP / self.WHEELBASE_LENGTH) * math.tan(self.steering_angle) # ignore sideslip
        new_theta = theta + dtheta
        new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi

        return x + dx, y + dy, new_theta

    def apply(self, start_pose, occupancy_grid=None):
        poses = [start_pose]
        for _ in range(self.num_steps_per_primitive):
            start_pose = self._apply_once(start_pose)
            poses.append(start_pose)
            if start_pose is None:
                return None, None
        if self.doPointsCollide(poses, occupancy_grid):
            return None, None
        return start_pose, poses


class Primitive_Planner:
    # Primitive_Planner static params.
    MAX_TURN = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/max_turn_to_divide")
    NUM_TURN_DIVISIONS = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/num_turn_divisions")
    TURNING_RADIUS = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/turning_radius")
    SAMPLING_RATE = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/sampling_rate")
    MAX_TURN_POSSIBLE = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/max_turn_possible")
    MAX_TURN_TOTAL_TIME = utils.getParamOrFail("/obstacle_avoidance/primitive_planner/max_turn_total_time")

    def __init__(self):

        # Build list of turn primitives.
        assert self.NUM_TURN_DIVISIONS % 2 == 1
        turn_angles = np.linspace(-self.MAX_TURN, self.MAX_TURN, self.NUM_TURN_DIVISIONS)
        np.testing.assert_almost_equal(turn_angles[self.NUM_TURN_DIVISIONS / 2], 0.0)
        turn_angles = sorted(turn_angles, key=lambda x: math.fabs(x))
        rospy.loginfo("Making turn angles: {}".format(turn_angles))
        self.primitives = [Turn_Primitive(angle) for angle in turn_angles]
        self.primitives.extend([Turn_Primitive(a * self.MAX_TURN_POSSIBLE, total_time=self.MAX_TURN_TOTAL_TIME) for a in [1, -1]])

        # Setup visualization publishers - latch them for easier debugging.
        self.display_tree = []
        self.display_path = []
        graph_viz_topic = utils.getParamOrFail("/obstacle_avoidance/graph_viz_topic")
        path_viz_topic = utils.getParamOrFail("/obstacle_avoidance/path_viz_topic")
        goal_line_viz_topic = utils.getParamOrFail("/obstacle_avoidance/goal_line_viz_topic")
        self.graph_pub = rospy.Publisher(graph_viz_topic, Marker, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher(path_viz_topic, Marker, queue_size=10, latch=True)
        self.goal_line_pub = rospy.Publisher(goal_line_viz_topic, Marker, queue_size=3, latch=True)

        # Data structure and publisher for the waypoints (PolygonStamped).
        self.line_trajectory = utils.LineTrajectory(viz_namespace='visualization')
        waypoint_topic = utils.getParamOrFail("/obstacle_avoidance/trajectory_topic")
        self.waypoint_pub = rospy.Publisher(waypoint_topic, PolygonStamped, queue_size=10, latch=True)

        # Listen on /clicked_point for goal points.
        # This will execute a plan from the current pose to specified goal.
        self.clicked_point_sub = rospy.Subscriber(
            "/clicked_point", PointStamped, self.clicked_point, queue_size=10
        )

        # Stores a list of points for collision checking.
        # self.trajectory_plc = plc.Point2fList()
        self.trajectory_list = []
        self.current_trajectory_cost = None

        # Listen on /initialpose for start and goal poses.
        self.clicked_pose_sub = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=10
        )

        self.pose_update_sub = rospy.Subscriber(
            utils.getParamOrFail("obstacle_avoidance/localization_topic"), PoseStamped, self.localization_cb, queue_size=1
        )

        # Setup occupancy grid.
        # self.local_grid_sub = rospy.Subscriber(
        #     utils.getParamOrFail("obstacle_avoidance/local_map_topic"), OGMSG, self.local_map_cb, queue_size=5
        # )

        self.occupancy_grid_sub = rospy.Subscriber(
            utils.getParamOrFail("obstacle_avoidance/dilated_map_topic"), OGMSG, self.occupancy_grid_cb, queue_size=1
        )

        self.occupancy_grid_msg = self.get_static_map() # Blocks until static map received.
        # self.occupancy_grid = plc.OccupancyGrid(self.occupancy_grid_msg)
        self.occupancy_grid = OccupancyGrid(self.occupancy_grid_msg)

        self.occupancy_grid.dilateOccupancyGrid(0.2, True)

        # Store state: (x, y, theta).
        self.previous_pose = None
        self.current_pose = None
        self.goal_point = None # Set by the clicked point callback.

        rospy.loginfo("Initialized Primitive Planner node!")

    def localization_cb(self, msg):
        """
        Called every time a new pose estimate is received.
        """
        self.current_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            utils.quaternion_to_angle(msg.pose.orientation)
        )

    def update_path(self, path_pt_1, path_pt_2):
        """
        Send a new path of waypoints to the controller (as PolygonStamped), and
        publish a visualization of the path.
        """
        rospy.loginfo('Updating path!')

        # Reset the trajectories for publishing and collision checking.
        self.line_trajectory.clear()
        # self.trajectory_plc = plc.Point2fList()
        self.trajectory_list

        for node in path_pt_1:
            for x, y, theta in node.poses:
                pt = plc.Point2f(x,y)
                self.line_trajectory.addPoint(pt)
                # self.trajectory_plc.append(pt)
                self.trajectory_list.append((x, y))
        for x, y, theta in path_pt_2:
            pt = plc.Point2f(x,y)
            self.line_trajectory.addPoint(pt)
            # self.trajectory_plc.append(pt)
            self.trajectory_list.append((x, y))

        # Send new path to controller.
        self.waypoint_pub.publish(self.line_trajectory.toPolygon())

        # Visualize the controller's new path.
        color = [0.2, 0.3, 1.0]
        marker = Marker()
        marker.header = utils.make_header("/map")
        marker.id = 106
        marker.type = 5 # Line list.
        marker.lifetime = rospy.Duration(0) # Stay forever.
        marker.action = 0 # Add/update.
        marker.scale.x = 0.1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.5
        for ii in range(len(self.line_trajectory.points)-1):
            marker.points.append(Point32(self.line_trajectory.points[ii][0], self.line_trajectory.points[ii][1], 0.0))
            marker.points.append(Point32(self.line_trajectory.points[ii+1][0], self.line_trajectory.points[ii+1][1], 0.0))
        self.path_pub.publish(marker)

    def occupancy_grid_cb(self, msg, debug=False): #local_map_cb(self, msg):
        """
        Called every time a new local map is received. Updates the occupancy grid and replans
        if the current path results in a collision.
        """
        # # Need a pose to transform local map into world map frame.
        # if (self.current_pose is not None):
        #     self.occupancy_grid.dynamicUpdate(msg, self.current_pose[0], self.current_pose[1], self.current_pose[2])
        # time.sleep(0.2)

        self.occupancy_grid_msg = msg
        # self.occupancy_grid = plc.OccupancyGrid(self.occupancy_grid_msg)
        self.occupancy_grid.setGridData(self.occupancy_grid_msg.data)

        if debug:
            pub = rospy.Publisher('/debugdebug', OGMSG, queue_size=5)
            pub.publish(self.occupancy_grid.getOccupancyGridMsg())

    def main(self):
        """
        Replans at a given frequency.
        """
        target_rate = 20
        target_time = 1.0 / target_rate
        starttime = time.time()

        while not rospy.is_shutdown():
            # Replan at every map update (after goal set) to see if a better path can be found.
            if (self.goal_point is not None):
                starttime = time.time()
                success, path_pt_1, path_pt_2, path_cost = self.plan_path(self.current_pose, self.goal_point, visualize=True)
                rospy.loginfo('Planned for %f sec' % (time.time() - starttime))
                rospy.loginfo('Success: %d', success)

            # Publish the new path if (1) none has been found yet, (2) it improves cost
            # or (3) the current path causes a collision. Don't plan unless a goal point has
            # been clicked.
            if (self.goal_point is not None) and success:
                shouldUpdatePath = (
                    (self.current_trajectory_cost is None) or
                    ((self.current_trajectory_cost - path_cost) > 1.0) or
                    # self.occupancy_grid.pathOccupiedVectPoint2f(self.trajectory_plc)
                    self.occupancy_grid.pathOccupied(self.trajectory_list)
                )

                isNone = (self.current_trajectory_cost is None)
                pathDiff = float('-inf') if isNone else (self.current_trajectory_cost - path_cost)
                isShorter = (pathDiff > 5.0)
                # isOccupied = self.occupancy_grid.pathOccupiedVectPoint2f(self.trajectory_plc)
                isOccupied = self.occupancy_grid.pathOccupied(self.trajectory_list)

                print("isNone:", isNone, "pathDiff:", pathDiff, "isShorter:", isShorter, "isOccupied:", isOccupied)

                if shouldUpdatePath:
                    rospy.loginfo('Sending a new path to the controller!')
                    self.current_trajectory_cost = path_cost
                    self.update_path(path_pt_1, path_pt_2)

            sleeptime = target_time - (time.time()-starttime)
            time.sleep(0 if sleeptime < 0 else sleeptime) # Sleep long enough to hit target rate.

    def get_static_map(self):
        """
        Blocks until a map is received on the ~static_map service.
        Returns: a ROS Occupancy Grid message.
        """
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.loginfo("Getting map from service: %s.", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        return map_msg

    def clicked_pose(self, msg):
        self.previous_pose = self.current_pose
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            utils.quaternion_to_angle(msg.pose.pose.orientation)
        )
        rospy.loginfo('Current pose: %f %f %f' % self.current_pose)

    def clicked_point(self, msg):
        """
        Callback for /clicked_point from RViz. This will update the goal to be
        the (x,y) location of the click.
        """
        self.goal_point = (msg.point.x, msg.point.y, 0.0)
        self.current_trajectory_cost = None # Mark that no trajectory has been found yet!

        goalVect = np.array([self.goal_point[0]-self.current_pose[0], self.goal_point[1]-self.current_pose[1]])
        goalVect = goalVect / utils.norm(goalVect[0], goalVect[1]) # Unit vector.
        normal = (-1*goalVect[1], goalVect[0])

        scale = 100
        endpoints = [
            (self.goal_point[0]+scale*normal[0], self.goal_point[1]+scale*normal[1]),
            (self.goal_point[0]-scale*normal[0], self.goal_point[1]-scale*normal[1])
        ]
        marker = Marker()
        marker.header = utils.make_header("/map")
        marker.id = 104
        marker.type = 5 # Line List.
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.1
        marker.action = 0 # Add/update.
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.points = [Point(pt[0], pt[1], 0) for pt in endpoints]
        self.goal_line_pub.publish(marker)

        rospy.loginfo('Updated goal point: %f %f %f' % self.goal_point)

    def update_occupancy_grid(self, grid):
        self.occupancy_grid = grid

    def plan_path(self, start_pose, goal_x, visualize=True):
        start_x, start_y, start_theta = start_pose

        root = AnyNode(
            start_pose=None,
            poses=[],
            end_pose=start_pose,
            cost=0.,
            discovered=False
        )
        success, (tree_leaf, dubins_path), cost = self.BFS(root, self.goal_point, visualize=visualize)

        if success:
            return success, tree_leaf.path, dubins_path, cost
        else:
            print 'Planning failed'
            return False, None, None, None

    def generate_and_check_dubins_curve(self, start, goal):
        # shortest_path returns a tuple of sampled poses and cumulative distances
        path, dists = dubins.shortest_path(start, goal, self.TURNING_RADIUS).sample_many(self.SAMPLING_RATE)
        total_dist = dists[-1]
        # path_plc = plc.Point2fList()
        # for x, y, theta in path:
        #     path_plc.append(plc.Point2f(x, y))
        # path_is_occupied = self.occupancy_grid.pathOccupiedVectPoint2f(path_plc)
        path_is_occupied = self.occupancy_grid.pathOccupied(path)

        if path_is_occupied:
            return None, None

        return path, total_dist

    def generate_children(self, node, visualize):
        children = []
        for i, primitive in enumerate(self.primitives):
            new_pose, new_poses = primitive.apply(node.end_pose, self.occupancy_grid)

            if new_pose is not None: # If this connection does not collide.
                children.append(
                    AnyNode(
                        start_pose=node.end_pose,
                        poses=new_poses,
                        end_pose=new_pose,
                        primitive=primitive,
                        primitive_num=i,
                        cost=node.cost+primitive.TOTAL_TIME,
                        discovered=False,
                        parent=node
                    )
                )
                if visualize:
                    for p1, p2 in zip(new_poses, new_poses[1:]):
                        self.display_tree.extend([p1, p2])

        # TODO maybe choose children ordering according to a heuristic
        if visualize:
            self.visualize()
        return children

    def goal_line_check(self, test_pose, start_pt, goal_pt):
        """
        Tests whether the test pose is past the goal line, which is defined as the
        line passing through goal_pt that is perpendicular to the vector from start
        to goal.
        """
        goalVect = (goal_pt[0]-start_pt[0], goal_pt[1]-start_pt[1])
        testVect = (test_pose[0]-start_pt[0], test_pose[1]-start_pt[1])
        progVect = (testVect[0]-goalVect[0], testVect[1]-goalVect[1])
        progress = progVect[0]*goalVect[0] + progVect[1]*goalVect[1]
        return (progress > 0 or utils.distance(test_pose[0:2], goal_pt) < 0.1)

    def BFS(self, root, goal_pt, visualize=False, with_dubins=False):
        self.display_tree = []

        # Store nodes in a min heap.
        heap = [(root.end_pose[0], root)]

        starttime = time.time()
        timeout = 1.0

        while len(heap) != 0 and (time.time() - starttime) < timeout:

            # Gets the node with LOWEST cost.
            _, node = heapq.heappop(heap)
            if not node.discovered: # This check is probably unnecessary.

                # If feasible path exists from this point onwards, returns goal = (goal_x, node.end_pose[1], 0.).
                if with_dubins:
                    dubins_path, path_length = self.generate_and_check_dubins_curve(node.end_pose, goal_pt)
                    if dubins_path is not None:
                        if visualize:
                            for p1, p2 in zip(dubins_path, dubins_path[1:]):
                                self.display_tree.extend([p1, p2])
                            self.visualize() # Show the tree.
                        return True, (node, dubins_path), path_length + node.cost

                # Alternatively, if the pose has passed the goal line, return success.
                if self.goal_line_check(node.end_pose, self.current_pose, goal_pt):
                    return True, (node, []), node.cost

                node.discovered = True
                for child in self.generate_children(node, visualize=visualize):
                    dist_to_goal = utils.distance(child.end_pose, goal_pt) # Cost = euclidean distance to goal.
                    heapq.heappush(heap, (dist_to_goal, child))

        if (time.time() - starttime) >= timeout:
            print 'Timed out'
            _, node = heapq.heappop(heap)
            return True, (node, []), node.cost


        # Ran out nodes to try.
        print 'Ran out of nodes'
        return False, (None, None), None

    def DFS(self, root, goal):
        # self.visualize()
        print "DFS level"
        root.discovered = True
        # if feasible path exists from this point onwards, returns
        dubins_path, path_lengths = self.generate_and_check_dubins_curve(root.end_pose, goal)
        if dubins_path is not None:
            print "Path to goal found"
            for p1, p2 in zip(dubins_path, dubins_path[1:]):
                self.display_tree.extend([p1, p2])
            self.visualize()
            return True, (root, dubins_path)

        for child in self.generate_children(root):
            if not child.discovered:
                done, answer = self.DFS(child, goal)
                if done:
                    return answer

        return False, (None, None)

    def visualize(self, duration=0.0, should_publish=True):
        """
        Display the tree.
        """
        red, green = (1., 0., 0.), (0., 1., 0.)
        for (pub, linelist, color) in [(self.graph_pub, self.display_tree, red)]:
                                       # (self.path_pub, self.display_path, green)]:
            marker = Marker()
            marker.header = utils.make_header("/map")
            marker.ns = 'visualization' + "/trajectory"
            marker.id = 101
            marker.type = 5 # Line list.
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = color[0] #1.0
                marker.color.g = color[1] #0.0
                marker.color.b = color[2] #0.0
                marker.color.a = 0.5

                for p in linelist:
                    pt = Point32(p[0], p[1], 0)
                    marker.points.append(pt)
            else:
                marker.action = 2

            pub.publish(marker)

if __name__=="__main__":
    rospy.init_node("primitive_planner")
    pp = Primitive_Planner()
    pp.main()
    rospy.spin()
