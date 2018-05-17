#include "motion_planning_tree.hpp"

namespace plc {

	MotionPlanningTree::MotionPlanningTree() {
	}

	void MotionPlanningTree::addNode(const Point2f& pt, int parent, float cost) {
		nodes_.push_back(pt);
		parents_.push_back(parent);
		costs_.push_back(cost);
	}

	Point2f MotionPlanningTree::nearestNeighbor(const Point2f& pt) {
		if (nodes_.size() > 0) {
			int best = 0;
			float bestDist = pt.distance(nodes_[0]);
			for (int i = 0; i < nodes_.size(); i++) {
				if (pt.distance(nodes_[i]) < bestDist) {
					best = i;
					bestDist = pt.distance(nodes_[i]);
				}
			}
			return nodes_[best];
		} else {
			return Point2f(0, 0);
		}
	}

	int MotionPlanningTree::nearestNeighborIndex(const Point2f& pt) {
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

	int MotionPlanningTree::rewireThroughNode(int nodeIdx, float radius) {
		int ctr = 0;
		
		// If the occupancy grid is not set, don't do anything.
		if (!occupancyGridSet()) {
			return 0;
		}
		Point2f node = getNode(nodeIdx);
		float nodeCost = getCost(nodeIdx);
		std::vector<int> nearbyIdx = getNearbyIndices(node, radius);

		for (int ii = 0; ii < nearbyIdx.size(); ii++) {
			Point2f nearby = getNode(nearbyIdx[ii]);

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

} // namespace plc
