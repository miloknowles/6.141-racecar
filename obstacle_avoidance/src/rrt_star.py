# RRT Star implementation.
import os, sys, math, datetime, json, random, time
from utils import *
import PlanningLibC as plc

class RRTStarAlgorithm(object):
  def __init__(self, occupancy_grid, start_state, goal_state, publish_graph_cb=None):
    """
    Initializes a data structure for running RRT*.

    occupancy_grid: a plc.OccupancyGrid object.
    start_state: a plc.Point2f
    goal_state: a plc.Point2f
    """
    self.tree = plc.MotionPlanningTree(occupancy_grid)
    assert self.tree.occupancyGridSet(), 'Error: og not set'

    # The start node has a parent index of -1 to show that it is root.
    self.tree.addNode(start_state, -1, 0)

    self.publish_graph_cb = publish_graph_cb

    self.start_state = start_state
    self.goal_state = goal_state

    # Store the occupancy grid wrapper.
    self.occupancy_grid = occupancy_grid

    # Limit expansion distance to this.
    self.expand_distance = 10 * start_state.distance(goal_state)
    rospy.loginfo('Start state: %s' % str(start_state))
    rospy.loginfo('Goal state:  %s' % str(goal_state))
    rospy.loginfo('RRT* Maximum Expand Distance: %f', self.expand_distance)

    self.max_iters = 100000 # Maximum RRT iterations.
    self.iteration = 0 # Stores the current iteration.

    self.goal_sample_rate = 0.1

    self.added_goal_state = False # Termination condition.

  def run(self, rewire=True, verbose=False, graph=False):
    """
    Runs RRT* to find a path between start and goal.
    """
    startTime = time.time()

    self.iteration = 0
    while not self.stop_condition:
      self.iteration += 1
      if verbose: print('Iteration:', self.iteration)

      # Sample a random position from the configuration space.
      q_rand = self.sample_occupancy_grid()

      # Find the closest neighbor index to q_rand in the graph.
      q_near_index = self.tree.nearestNeighborIndex(q_rand)
      q_near = self.tree.getNode(q_near_index)

      # Returns a new configuration, limited to some exploration distance.
      # Will also check for obstacle collisions.
      q_new = self.extend(q_rand, q_near_index)

      # If the new configuration is invalid (i.e due to collision, skip this iteration).
      if q_new == None:
        if verbose: print('Invalid configuration, skipping iteration.')
        continue

      # Add a node. Set its parent to the nearest node in tree. Set distance to 
      # the cumulative distance so far.
      transition_dist = q_new.distance(q_near)
      self.tree.addNode(q_new, q_near_index, self.tree.getCost(q_near_index)+transition_dist)

      # Do rewiring after adding node.
      if rewire:
        r = 100.0 * self.expand_distance * math.sqrt((math.log(self.tree.numNodes()) / self.tree.numNodes())) ** 2
        num_rewired = self.tree.rewireThroughNode(self.tree.numNodes()-1, r)

      # Check if goal state was just added.
      if q_new.distance(self.goal_state) < 0.1:
        self.added_goal_state = True

      # Publish the graph to ROS as a marker.
      if self.publish_graph_cb is not None and graph:
        line_list = []
        for i in range(self.tree.numNodes()):
          line_list.append(self.tree.getNode(i))

          # Connect to parent if not root node.
          if (self.tree.getParent(i) != -1):
            line_list.append(self.tree.getNode(self.tree.getParent(i)))
          else:
            line_list.append(self.tree.getNode(i))

        self.publish_graph_cb([], should_publish=False) # Clear the previous graph.
        self.publish_graph_cb(line_list, should_publish=True) # Show new graph.

    # Get the final path, setting the node we start tracing from to the last one.
    path = self.get_final_path(self.tree.numNodes()-1)
    print('Found path in: %f sec.' % (time.time() - startTime))
    return path

  def sample_occupancy_grid(self):
    if random.random() < self.goal_sample_rate:
      return self.goal_state
    else:
      return self.occupancy_grid.sampleOccupancyGrid()

  def get_final_path(self, end_idx):
    current = end_idx
    path = []

    while self.tree.getParent(current) != -1:
      path.append(self.tree.getNode(current))
      current = self.tree.getParent(current)
    path.append(self.tree.getNode(current))

    path.reverse() # Order from start to finish.
    return path

  def extend(self, q_rand, q_near_idx):
    """
    Returns a q_new after limiting exploration distance along the line from
    q_near to q_rand. Also checks for obstacle collisions along this line. If
    there is a collision, returns None.
    q_rand: (Point2f)
    q_near: (node index) the nearest neighbor of q_rand.
    """
    # Limit distance of expansion.
    q_near = self.tree.getNode(q_near_idx)

    dist = q_rand.distance(q_near)
    dist = min(dist, self.expand_distance)

    q_new = None
    if q_rand.x == q_near.x:
      y = (q_near.y - dist) if (q_rand.y <= q_near.y) else (q_near.y + dist)
      q_new = (q_rand.x, y)
    else:
      theta = math.atan2(q_rand.y - q_near.y, q_rand.x - q_near.x)
      q_new = plc.Point2f(q_near.x + dist * math.cos(theta), q_near.y + dist * math.sin(theta))

    # Ensure there are no collisions when going to new configuration.
    if self.occupancy_grid.pathOccupied(q_near, q_new):
      return None

    return q_new

  @property
  def stop_condition(self):
    """
    Evaluates to (bool) whether the RRT stop condition has been met.
    """
    return (self.iteration >= self.max_iters or self.added_goal_state)

  def rewire(self, new_idx, near_idx):
    num_nodes = self.tree.numNodes()
    q_new = self.tree.getNode(new_idx)

    # For each nearby node, see if it's less expensive to get there
    # from the node that was just added.
    for i in near_idx:
      q_near = self.tree.getNode(i)
      cost_diff = q_new.distance(q_near)
      new_cost = self.tree.getCost(new_idx) + cost_diff

      if self.tree.getCost(i) > new_cost:
        if self.occupancy_grid.pathOccupied(q_near, q_new) == False: # Ensure rewiring avoids collisions.
          self.tree.setParent(i, new_idx)
          self.tree.setCost(i, new_cost)
