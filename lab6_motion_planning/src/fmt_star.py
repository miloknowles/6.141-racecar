"""
Implements Fast Marching Tree (FMT*) algorithm
"""
import math
from anytree import AnyNode
import PlanningLibC as plc
import time
import numpy as np

class FMTStarAlgorithm(object):

    def __init__(self, start_pt, goal_pt, occupancy_grid):
        self.free_samples = []
        self.free_samples_np = np.array([])
        self.start_pt = start_pt
        self.goal_pt = goal_pt
        self.og = occupancy_grid
        self.map_res = self.og.getMapResolution()
        self.num_samples = 0
        self.line_list = []

        self.dim = 2
        num_free_cells = self.og.getFreeGridCount()
        area_of_free_space = num_free_cells * self.map_res * self.map_res
        area_of_unit_circle = math.pi
        self.gamma_min = 2 * ((1./self.dim)**(1./self.dim)) * (area_of_free_space / area_of_unit_circle) ** (1./self.dim)

    def sampleFromFreeSpace(self, num_samples=2000):
        """ Generate num_samples samples from the unoccupied area. Returns a list. """
        self.num_samples = num_samples

        self.gamma = 10 * self.gamma_min * (math.log(self.num_samples) / self.num_samples) ** (1./self.dim)


        samples = []
        while len(samples) < num_samples:
            pt = self.og.sampleOccupancyGrid()
            if self.og.getPointValue(pt) == 0:
                samples.append(pt)
        return samples

    def getNeighborIndices(self, orig_index, allowed_indices):
        """ Get the indices of neighbors of orig_index that are allowed """
        # Two samples are considered neighbours if their Euclidean distance is smaller than:
        #   gamma * (log(n) / n) ^ (1/d)
        # where: n is number of samples generated
        #      : gamma > 2 ((1/d)^(1/d))*(mu(X_free)/zeta_d)^(1/d) is a tuning parameter
        #      : zeta_d is the volume of the unit ball in d-dimensional space
        #      : mu(X) is the d-dimensional Lebesgue measure of X. a.k.a length, area, volume, etc.

        allowed_indices = np.array(list(allowed_indices))
        #print(allowed_indices)
        #print(orig_index)
        diff_from_orig = self.free_samples_np[allowed_indices] - self.free_samples_np[orig_index]
        dist_from_orig = np.linalg.norm(diff_from_orig, axis=1)
        return allowed_indices[dist_from_orig < self.gamma]

        # neighbor_indices = []
        # for index in allowed_indices:
        #     dist = self.costBetweenPoints(orig_index, index)
        #     if dist < self.gamma:
        #         neighbor_indices.append(index)
        # return neighbor_indices

    def costBetweenPoints(self, start_index, end_index):
        """ Computes Euclidean distance between points """
        start_pt = self.free_samples[start_index]
        end_pt = self.free_samples[end_index]
        return math.sqrt((end_pt.x - start_pt.x)**2 + (end_pt.y - start_pt.y)**2)

    def isCollision(self, start_index, end_index):
        """ Returns whether or not there is a collision along the line segment """
        return self.og.pathOccupied(
            self.free_samples[start_index],
            self.free_samples[end_index]
        )

    def runAlgorithm(self, publish_graph_cb=None):
        """ Runs FMT* algorithm. Returns tuple of path (itself a tuple of points) and the overall cost """
        # Generate free_sample_set comprising of start_pt, goal_pt and n other samples from the free space
        self.free_samples.append(plc.Point2f(self.start_pt.x, self.start_pt.y))
        self.free_samples.append(plc.Point2f(self.goal_pt.x, self.goal_pt.y))
        self.free_samples.extend(self.sampleFromFreeSpace())
        self.free_samples_np = np.zeros((len(self.free_samples), 2), dtype=np.float64)
        for i, sample in enumerate(self.free_samples):
            self.free_samples_np[i] = [sample.x, sample.y]
        start_index = 0
        goal_index = 1

        # Initialise open_set with start_pt and unvisited_set with all other samples (incl goal_pt) and closed_set with empty
        open_set = {start_index}
        unvisited_set = set(range(len(self.free_samples)))
        unvisited_set.remove(start_index)
        closed_set = set()

        # Initialise paths_tree with root node start_pt
        self.paths_tree = {start_index: AnyNode(point_index=start_index, cost=0)}

        while True:
            print(len(open_set), len(closed_set), len(unvisited_set))

            start = time.time()
            neighbor_time = 0

            # If open_set is empty, report failure
            if not open_set:
                assert False

            # Find lowest-cost node (z) in open_set
            lowest_cost_node_index = min(open_set, key=lambda x: self.paths_tree[x].cost)

            # If z is goal, return unique path to z and report success
            if lowest_cost_node_index == goal_index:
                path = self.paths_tree[goal_index].path
                cost = path[-1].cost
                path = list(self.free_samples[node.point_index] for node in path) # Convert to points
                return path #, cost

            # For each of z's neighbours (x) in unvisited_set:
            start_neighbor = time.time()
            unvisited_neighbors_of_lowest_cost_indices = self.getNeighborIndices(lowest_cost_node_index, unvisited_set)
            end_neighbor = time.time()

            neighbor_time += end_neighbor - start_neighbor

            nodes_successfully_connected_this_round = []
            for unvisited_neighbor_index in unvisited_neighbors_of_lowest_cost_indices:

                # Find neighbor nodes (y) of x in open_set
                start_neighbor = time.time()
                open_neighbors_indices = self.getNeighborIndices(unvisited_neighbor_index, open_set)
                end_neighbor = time.time()

                neighbor_time += end_neighbor - start_neighbor

                # Find locally-optimal one-set connection to x from among nodes y
                locally_optimal_connection_start = min(open_neighbors_indices, key=lambda x: self.costBetweenPoints(unvisited_neighbor_index, x))

                # If that connection is collision-free, add edge to path_tree
                if not self.isCollision(locally_optimal_connection_start, unvisited_neighbor_index):
                    cost_up_to_now = self.paths_tree[locally_optimal_connection_start].cost
                    additional_cost = self.costBetweenPoints(locally_optimal_connection_start, unvisited_neighbor_index)
                    full_cost = cost_up_to_now + additional_cost
                    self.paths_tree[unvisited_neighbor_index] = AnyNode(
                        point_index=unvisited_neighbor_index,
                        cost=full_cost,
                        parent=self.paths_tree[locally_optimal_connection_start]
                    )
                    if publish_graph_cb is not None:
                        #publish_graph_cb(
                        self.line_list.extend(
                            [self.free_samples[unvisited_neighbor_index], self.free_samples[locally_optimal_connection_start]]
                        )
                        #)
                        publish_graph_cb(self.line_list)
                    nodes_successfully_connected_this_round.append(unvisited_neighbor_index)

            # Remove successfully connected nodes x from unvisited_set, moving them to open_set
            for node_index in nodes_successfully_connected_this_round:
                unvisited_set.remove(node_index)
                open_set.add(node_index)

                # HACK: If z is goal, return unique path to z and report success
                if node_index == goal_index:
                    path = self.paths_tree[goal_index].path
                    cost = path[-1].cost
                    path = list(self.free_samples[node.point_index] for node in path) # Convert to points
                    return path #, cost

            # Remove z from open_set and move to closed_set
            open_set.remove(lowest_cost_node_index)
            closed_set.add(lowest_cost_node_index)

            end = time.time()
            print("=====")
            print("Total time:", end - start)
            print("Neighbor:", neighbor_time)
            print("=====")
