#pragma once

#include <vector>
#include <unordered_map>
#include <algorithm>

#include "point2.hpp"
#include "occupancy_grid.hpp"
#include "pure_pursuit_controller.hpp"

namespace plc {

/**
 * @brief Representation of nodes in the CL-RRT tree.
 * Each node represents a state of the robot, along with
 * the reference input that got the robot there.
 */
struct TreeNode {
  // Constructor with ID.
  TreeNode(const Pose3& s, const PurePursuitInput& i, int id, float c = 0) :
    state(s), input(i), id(id), cost(c) {}

  // Constructor without ID.
  TreeNode(const Pose3& s, const PurePursuitInput& i, float c = 0) :
    state(s), input(i), cost(c) {}

  // Default constructor.
  TreeNode() = delete;

  void setID(int i) { id = i; }

  Pose3 state;
  PurePursuitInput input;
  float cost;
  int id;
};

/**
 * @brief Typedef used for python wrapper.
 */
typedef std::vector<int> IntegerList;

template <typename State>
class MotionPlanningTree {
 public:
 	MotionPlanningTree<State>() = default;
 	MotionPlanningTree<State>(const OccupancyGrid& og) : occupancyGrid_(og) {}

 	/**
 	 * @brief Add a node to the tree.
 	 * @param[in] pt The location of the new node.
 	 * @param[in] parent The index of the parent node for this node.
 	 * @param[in] cost The cost to assign to this new node.
 	 */
  void addNode(const State& pt, int parent = -1, float cost = 0.0f) {
  	nodes_.push_back(pt);
  	parents_.push_back(parent);
  	costs_.push_back(cost);
  }

 	/**
 	 * @brief Returns the number of nodes in tree.
 	 */
 	int numNodes() const { return nodes_.size(); }

 	/**
 	 * @brief Return the nearest node to a point.
   * Uses nearestNeighborIndex as a helper function.
 	 */
  State nearestNeighbor(const State& pt) {
    int idx = nearestNeighborIndex(pt);
    return (idx == -1) ? State() : nodes_[idx];
  }

 	/**
 	 * @brief Return the index of the nearest node to a point.
 	 */
  int nearestNeighborIndex(const State& pt) {
  	if (nodes_.size() > 0) {
  		int best = 0;
  		float bestDist = pt.distance(nodes_[0]);
  		for (int i = 0; i < nodes_.size(); i++) {
  			if (pt.distance(nodes_[i]) < bestDist) {
  				best = i;
  				bestDist = pt.distance(nodes_[i]);
  			}
  		}
  		return best;
  	} else {
  		return -1;
  	}
  }

 	/**
 	 * @brief Return the indices of all points within a radius of pt.
 	 */
 	IntegerList getNearbyIndices(const State& pt, float radius) {
 		IntegerList nearby;
 		for (int ii = 0; ii < nodes_.size(); ii++) {
 			if (nodes_[ii].distance(pt) < radius) {
 				nearby.push_back(ii);
 			}
 		}
 		return nearby;
 	}

 	/**
 	 * @brief Get a node by its index.
 	 */
 	State getNode(int idx) const { return nodes_[idx]; }

  /**
   * @brief Get current list of nodes in tree.
   */
  const std::vector<State>& getNodes() const { return nodes_; }

 	/**
 	 * @brief Get the parent index.
 	 */
 	int getParent(int idx) const { return parents_[idx]; }

 	/**
 	 * @brief Get the cost of a node by its index.
 	 */
 	float getCost(int idx) const { return costs_[idx]; }

 	/**
 	 * @brief Set the parent index of node with index nodeIdx to parentIdx.
 	 */
 	bool setParent(int nodeIdx, int parentIdx) {
 		if (nodeIdx < nodes_.size() && parentIdx < nodes_.size()) {
 			parents_[nodeIdx] = parentIdx;
 			return true;
 		} else {
 			return false;
 		}
 	}

 	bool setCost(int nodeIdx, float cost) {
 		if (nodeIdx < costs_.size()) {
 			costs_[nodeIdx] = cost;
 			return true;
 		} else {
 			return false;
 		}
 	}

  void clear() {
    nodes_.clear();
    parents_.clear();
    costs_.clear();
  }

  /**
   * @brief Reconstructs a path from endIdx to the root of the tree.
   * If there is a cycle, or the path doesn't lead to root, this will
   * return false.
   * Note: endIdx = -1 means use the most recently added node.
   */
  bool reconstructPath(std::vector<TreeNode>* path, int endIdx = -1) const {
    int idx = (endIdx == -1) ? numNodes()-1 : endIdx;
    while (path->size() < nodes_.size() && getParent(idx) != -1) {
      path->push_back(nodes_[idx]);
      idx = parents_[idx];
    }
    path->push_back(nodes_[idx]); // Push back the start node.
    std::reverse(path->begin(), path->end());
    return (parents_[idx] == -1);
  }

  /**
   * @brief Performs RRT* rewiring.
   * Note: this function requires an occupancy grid to be set.
   */
  int rewireThroughNode(int nodeIdx, float radius) {
  	int ctr = 0;

  	// If the occupancy grid is not set, don't do anything.
  	if (!occupancyGridSet()) {
  		return 0;
  	}
  	State node = getNode(nodeIdx);
  	float nodeCost = getCost(nodeIdx);
  	std::vector<int> nearbyIdx = getNearbyIndices(node, radius);

  	for (int ii = 0; ii < nearbyIdx.size(); ii++) {
  		State nearby = getNode(nearbyIdx[ii]);

  		float costUsingNode = nodeCost + nearby.distance(node);

  		if (getCost(nearbyIdx[ii]) > costUsingNode) {
  			if (!occupancyGrid_.pathOccupied(nearby, node)) {
  				setParent(nearbyIdx[ii], nodeIdx);
  				setCost(nearbyIdx[ii], costUsingNode);
  				ctr++;
  			}
  		}
  	}
  	return ctr;
  }

 	bool occupancyGridSet() const {
    return (occupancyGrid_.getMapWidth() > 0 && occupancyGrid_.getMapHeight() > 0);
  }

 private:
 	std::vector<int> parents_{};
 	std::vector<State> nodes_{};
 	std::vector<float> costs_{};

 	OccupancyGrid occupancyGrid_{};
};

inline std::vector<Point2f> graphToLineList(const MotionPlanningTree<TreeNode>& tree) {
  std::vector<Point2f> list;

  for (int i = 0; i < tree.numNodes(); i++) {
    TreeNode node = tree.getNode(i);
    list.push_back(node.state.position);

    if (tree.getParent(i) != -1) {
      list.push_back(tree.getNode(tree.getParent(i)).state.position); // Connect to parent.
    } else {
      list.push_back(node.state.position); // Connect to self.
    }
  }
  return list;
}

} // namespace plc
