#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>

#include "occupancy_grid.hpp"
#include "point2.hpp"
#include "motion_planning_tree.hpp"

using namespace plc;

// Function to compare the elements of two vectors.
template <typename T>
void EXPECT_EQ_VECT(std::vector<T> v1, std::vector<T> v2) {
	EXPECT_EQ(v1.size(), v2.size());

	for (int ii = 0; ii < v1.size(); ii++) {
		EXPECT_EQ(v1[ii], v2[ii]);
	}
}

TEST(MPTTest, testConstructor) {
	MotionPlanningTree<Point2f> mpt;
	int numNodes = mpt.numNodes();
	EXPECT_EQ(numNodes, 0);
}

TEST(MPTTest, testAddNode) {
	MotionPlanningTree<Point2f> mpt;
	mpt.addNode(Point2f(0, 0), -1, 10);
	EXPECT_EQ(mpt.numNodes(), 1);
	EXPECT_EQ(mpt.getNode(0), Point2f(0, 0));
	EXPECT_EQ(mpt.getParent(0), -1);
	EXPECT_EQ(mpt.getCost(0), 10);

	mpt.addNode(Point2f(1, 1), 0, 20);
	EXPECT_EQ(mpt.numNodes(), 2);
	EXPECT_EQ(mpt.getNode(1), Point2f(1, 1));
	EXPECT_EQ(mpt.getParent(1), 0);
	EXPECT_EQ(mpt.getCost(1), 20);

	EXPECT_EQ(mpt.getNode(mpt.getParent(1)), Point2f(0, 0));
}

TEST(MPTTest, testNearestNeighbor) {
	MotionPlanningTree<Point2f> mpt;
	mpt.addNode(Point2f(0, 0));
	mpt.addNode(Point2f(-11.1, -13.4));

	int nearIdx = mpt.nearestNeighborIndex(Point2f(1, 1));
	Point2f nearNode = mpt.nearestNeighbor(Point2f(-12.1, -13.1));

	EXPECT_EQ(nearIdx, 0);
	EXPECT_EQ(nearNode, Point2f(-11.1, -13.4));
}

TEST(MPTTest, testNearestNeighborEmpty) {
	MotionPlanningTree<Point2f> mpt;
	int nearIdx = mpt.nearestNeighborIndex(Point2f(1, 1));
	Point2f nearNode = mpt.nearestNeighbor(Point2f(1, 1));

	EXPECT_EQ(nearNode, Point2f(0, 0));
	EXPECT_EQ(nearIdx, -1);
}

TEST(MPTTest, testGetNearbyIndices) {
	MotionPlanningTree<Point2f> mpt;
	mpt.addNode(Point2f(0, 0));
	mpt.addNode(Point2f(1, 1));
	mpt.addNode(Point2f(3, 3));

	std::vector<int> nearbyIdx = mpt.getNearbyIndices(Point2f(0, 0), 1.5);
	std::vector<int> expected = {0, 1};
	EXPECT_EQ_VECT(nearbyIdx, expected);
}
