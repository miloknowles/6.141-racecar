#pragma once

#include <vector>
#include <unordered_map>

#include "point2.hpp"
#include "occupancy_grid.hpp"

namespace plc {

typedef std::vector<int> IntegerList;

class MotionPlanningTree {
 public:
 	MotionPlanningTree();

 	MotionPlanningTree(const OccupancyGrid& og) : occupancyGrid_(og) {
 	}
 	
 	/**
 	 * @brief Add a node to the tree.
 	 * @param[in] pt The location of the new node.
 	 * @param[in] parent The index of the parent node for this node.
 	 * @param[in] cost The cost to assign to this new node.
 	 */
 	void addNode(const Point2f& pt, int parent = -1, float cost = 0);

 	/**
 	 * @brief Returns the number of nodes in tree.
 	 */
 	int numNodes() const { return nodes_.size(); }

 	/**
 	 * @brief Return the nearest node to a point.
 	 */
 	Point2f nearestNeighbor(const Point2f& pt);

 	/**
 	 * @brief Return the index of the nearest node to a point.
 	 */
 	int nearestNeighborIndex(const Point2f& pt);

 	/**
 	 * @brief Return the indices of all points within a radius of pt.
 	 */
 	IntegerList getNearbyIndices(const Point2f& pt, float radius) {
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
 	Point2f getNode(int idx) { return nodes_[idx]; }

 	/**
 	 * @brief Get the parent index.
 	 */
 	int getParent(int idx) { return parents_[idx]; }

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

 	int rewireThroughNode(int nodeIdx, float radius);

 	bool occupancyGridSet() { return (occupancyGrid_.getMapWidth() > 0 && occupancyGrid_.getMapHeight() > 0); }

 private:
 	std::vector<int> parents_{};
 	std::vector<Point2f> nodes_{};
 	std::vector<float> costs_{};

 	OccupancyGrid occupancyGrid_{};
};

} // namespace plc
