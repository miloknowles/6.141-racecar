#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>

#include "occupancy_grid.hpp"
#include "point2.hpp"
#include "motion_planning_tree.hpp"

using namespace plc;

OccupancyGrid setupOccupancyGrid() {
	nav_msgs::OccupancyGrid grid;
	grid.info.width = 100;
	grid.info.height = 200;
	grid.info.resolution = 0.1; // 0.1 meters per grid cell.
	OccupancyGrid ogw(grid);
	return ogw;
}

// Function to compare the elements of two vectors.
template <typename T>
void EXPECT_EQ_VECT(std::vector<T> v1, std::vector<T> v2) {
	EXPECT_EQ(v1.size(), v2.size());

	for (int ii = 0; ii < v1.size(); ii++) {
		EXPECT_EQ(v1[ii], v2[ii]);
	}
}

TEST(Point2Test, testEqualityOperators) {
	Point2f p1(11.0, 11.0);
	Point2f p2(11.0, 11.0);

	EXPECT_TRUE(p1 == p2);
	EXPECT_FALSE(p1 != p2);

	Point2f p3(11.1, 11.1);
	EXPECT_FALSE(p1 == p3);
	EXPECT_TRUE(p1 != p3);
}

// Test the that originalOccupied_ cells vector is set properly.
TEST(OccupancyGridTest, testOriginalOccupied) {

	// Empty map should have none occupied.
	OccupancyGrid ogw = setupOccupancyGrid();
	std::vector<Point2i> orig = ogw.getOriginalOccupied();
	EXPECT_EQ(orig.size(), 0);

	// Occupy a few cells.
	nav_msgs::OccupancyGrid grid;
	grid.info.width = 100;
	grid.info.height = 200;
	grid.info.resolution = 0.1; // 0.1 meters per grid cell.

	for (int i = 0; i < 100*200; i++) {
		grid.data.push_back(0);
	}
	grid.data[0] = 1;
	grid.data[100] = 1;
	grid.data[200] = 1;
	OccupancyGrid ogw1(grid);

	std::vector<Point2i> orig1 = ogw1.getOriginalOccupied();
	std::vector<Point2i> expected = {Point2i(0, 0), Point2i(0, 1), Point2i(0, 2)};
	EXPECT_EQ(orig1.size(), 3);
	EXPECT_EQ_VECT(orig1, expected);
}

TEST(OccupancyGridTest, testPointToGrid) {
	OccupancyGrid ogw = setupOccupancyGrid();

	// Expect (0, 0) in meters to be grid (0, 0) also.
	Point2f pt1(0, 0);
	Point2i pt1g = ogw.pointToGrid(pt1);
	EXPECT_EQ(pt1g.x, 0);
	EXPECT_EQ(pt1g.y, 0);

	// Expect (3, 4) in meters to be grid (30, 40).
	Point2f pt2(3.01, 4.01);
	Point2i pt2g = ogw.pointToGrid(pt2);
	EXPECT_EQ(pt2g.x, 30);
	EXPECT_EQ(pt2g.y, 40);
}

TEST(OccupancyGridTest, testGridToPoint) {
	OccupancyGrid ogw = setupOccupancyGrid();

	Point2i pt1g(0, 0);
	Point2f pt1 = ogw.gridToPoint(pt1g);
	EXPECT_EQ(pt1.x, 0);
	EXPECT_EQ(pt1.y, 0);

	Point2i pt2g(30, 40);
	Point2f pt2 = ogw.gridToPoint(pt2g);
	EXPECT_EQ(pt2.x, 3);
	EXPECT_EQ(pt2.y, 4);
}

TEST(OccupancyGridTest, testGetBresenhamPointsHorizontal) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(0, 0);
	Point2f pt2(1, 0);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	// Check the size and coordinates of all points.
	EXPECT_EQ(gridPts.size(), 10);
	std::vector<Point2i> expected;
	for (int i = 0; i < 10; i++) {
		expected.push_back(Point2i(i, 0));
	}
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsVertical) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(0, 0);
	Point2f pt2(0, 1);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	// Check the size and coordinates of all points.
	EXPECT_EQ(gridPts.size(), 10);
	std::vector<Point2i> expected;
	for (int i = 0; i < 10; i++) {
		expected.push_back(Point2i(0, i));
	}
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsPosYPosX) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(1.05, 1.05);
	Point2f pt2(1.15, 1.45);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	EXPECT_EQ(gridPts.size(), 5);
	std::vector<Point2i> expected = {
		Point2i(10, 10),
		Point2i(10, 11),
		Point2i(10, 12),
		Point2i(11, 13),
		Point2i(11, 14),
	};
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsNegYNegX) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(1.15, 1.45);
	Point2f pt2(1.05, 1.05);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	EXPECT_EQ(gridPts.size(), 5);
	std::vector<Point2i> expected = {
		Point2i(10, 10),
		Point2i(10, 11),
		Point2i(10, 12),
		Point2i(11, 13),
		Point2i(11, 14),
	};
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsNegYPosX) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(1.05, 1.45);
	Point2f pt2(1.15, 1.05);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	EXPECT_EQ(gridPts.size(), 5);
	std::vector<Point2i> expected = {
		Point2i(11, 10),
		Point2i(11, 11),
		Point2i(11, 12),
		Point2i(10, 13),
		Point2i(10, 14),
	};
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsPosYNegX) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(1.15, 1.05);
	Point2f pt2(1.05, 1.45);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	EXPECT_EQ(gridPts.size(), 5);
	std::vector<Point2i> expected = {
		Point2i(11, 10),
		Point2i(11, 11),
		Point2i(11, 12),
		Point2i(10, 13),
		Point2i(10, 14),
	};
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsPosXNegY) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(1.05, 1.15);
	Point2f pt2(1.45, 1.05);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	EXPECT_EQ(gridPts.size(), 5);
	std::vector<Point2i> expected = {
		Point2i(10, 11),
		Point2i(11, 11),
		Point2i(12, 11),
		Point2i(13, 10),
		Point2i(14, 10),
	};
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsNegXNegY) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(1.45, 1.15);
	Point2f pt2(1.05, 1.05);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	EXPECT_EQ(gridPts.size(), 5);
	std::vector<Point2i> expected = {
		Point2i(10, 10),
		Point2i(11, 10),
		Point2i(12, 10),
		Point2i(13, 11),
		Point2i(14, 11),
	};
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsNegXPosY) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(1.45, 1.05);
	Point2f pt2(1.05, 1.15);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	EXPECT_EQ(gridPts.size(), 5);
	std::vector<Point2i> expected = {
		Point2i(10, 11),
		Point2i(11, 11),
		Point2i(12, 11),
		Point2i(13, 10),
		Point2i(14, 10),
	};
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testGetBresenhamPointsPosXPosY) {
	OccupancyGrid ogw = setupOccupancyGrid();

	std::vector<Point2i> gridPts;
	Point2f pt1(0, 0);
	Point2f pt2(1, 1);
	ogw.getBresenhamPoints(pt1, pt2, &gridPts);
	printPointVect(gridPts);

	// Check the size and coordinates of all points.
	EXPECT_EQ(gridPts.size(), 10);
	std::vector<Point2i> expected;
	for (int i = 0; i < 10; i++) {
		expected.push_back(Point2i(i, i));
	}
	EXPECT_EQ_VECT(gridPts, expected);
}

TEST(OccupancyGridTest, testSampleOccupancyGrid) {
	OccupancyGrid ogw = setupOccupancyGrid();

	int numSamples = 100;
	for (int i = 0; i < numSamples; i++) {
		Point2f pt = ogw.sampleOccupancyGrid();
		EXPECT_TRUE(pt.x <= 10);
		EXPECT_TRUE(pt.x >= 0);

		EXPECT_TRUE(pt.y <= 20);
		EXPECT_TRUE(pt.y >= 0);
	}
}

TEST(OccupancyGridTest, testGetters) {
	OccupancyGrid ogw = setupOccupancyGrid();
	EXPECT_EQ(ogw.getMapWidth(), 100);
	EXPECT_EQ(ogw.getMapHeight(), 200);
	EXPECT_EQ(ogw.getMapResolution(), 0.1f);
	EXPECT_EQ(ogw.getMapWidthMeters(), 10.0f);
	EXPECT_EQ(ogw.getMapHeightMeters(), 20.0f);
}

TEST(OccupancyGridTest, testSetGetValue) {
	OccupancyGrid ogw = setupOccupancyGrid();
	bool result = ogw.setGridValue(Point2i(1, 1), 127);
	EXPECT_TRUE(result);

	int val = ogw.getGridValue(Point2i(1, 1));
	EXPECT_EQ(val, 127);
}

TEST(OccupancyGridTest, testPathOccupied) {
	OccupancyGrid ogw = setupOccupancyGrid();
	bool result = ogw.setGridValue(Point2i(1, 1), 127);
	std::cout << result << std::endl;
	Point2f start(0, 0);
	Point2f end(0.2, 0.2);
	EXPECT_TRUE(ogw.pathOccupied(start, end));
}

TEST(OccupancyGridTest, testDilateOccupancyGrid) {
	OccupancyGrid ogw = setupOccupancyGrid();
	ogw.setGridValue(Point2i(0, 0), 100);
	ogw.dilateOccupancyGrid(0.2); // Dilate with 0.2 meter radius.

	// Note: all cells that are partially within the dilation radius will get
	// occupied. That's why (1, 2) and (2, 1) are included below.
	std::vector<Point2i> expected = {
		Point2i(0, 0),
		Point2i(0, 1),
		Point2i(0, 2),
		Point2i(1, 0),
		Point2i(2, 0),
		Point2i(1, 1),
		Point2i(1, 2),
		Point2i(2, 1)
	};

	for (const Point2i& pt : expected) {
		int val = ogw.getGridValue(pt);
		EXPECT_EQ(ogw.getGridValue(pt), 100);
	}
}

TEST(MPTTest, testConstructor) {
	MotionPlanningTree mpt;
	int numNodes = mpt.numNodes();
	EXPECT_EQ(numNodes, 0);
}

TEST(MPTTest, testAddNode) {
	MotionPlanningTree mpt;
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
	MotionPlanningTree mpt;
	mpt.addNode(Point2f(0, 0));
	mpt.addNode(Point2f(-11.1, -13.4));

	int nearIdx = mpt.nearestNeighborIndex(Point2f(1, 1));
	Point2f nearNode = mpt.nearestNeighbor(Point2f(-12.1, -13.1));

	EXPECT_EQ(nearIdx, 0);
	EXPECT_EQ(nearNode, Point2f(-11.1, -13.4));
}

TEST(MPTTest, testNearestNeighborEmpty) {
	MotionPlanningTree mpt;
	int nearIdx = mpt.nearestNeighborIndex(Point2f(1, 1));
	Point2f nearNode = mpt.nearestNeighbor(Point2f(1, 1));

	EXPECT_EQ(nearNode, Point2f(0, 0));
	EXPECT_EQ(nearIdx, -1);
}

TEST(MPTTest, testGetNearbyIndices) {
	MotionPlanningTree mpt;
	mpt.addNode(Point2f(0, 0));
	mpt.addNode(Point2f(1, 1));
	mpt.addNode(Point2f(3, 3));

	std::vector<int> nearbyIdx = mpt.getNearbyIndices(Point2f(0, 0), 1.5);
	std::vector<int> expected = {0, 1};
	EXPECT_EQ_VECT(nearbyIdx, expected);
}

// Run all the tests that were declared with TEST().
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
