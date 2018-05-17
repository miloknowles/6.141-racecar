#include <gtest/gtest.h>

#include "utils.hpp"
#include "point2.hpp"

using namespace plc;

TEST(UtilsTest, testCircleLineIntersection) {
	Point2f p1(0, 0);
	Point2f p2(1, 0);

	std::vector<Point2f> pts1 = utils::circleLineIntersection(p1, p2, Point2f(0, 0), 1);
	EXPECT_EQ(pts1.size(), 2);
	EXPECT_EQ(pts1[1], Point2f(-1, 0));
	EXPECT_EQ(pts1[0], Point2f(1, 0));

	std::vector<Point2f> pts2 = utils::circleLineIntersection(p1, p2, Point2f(0, 20), 1);
	EXPECT_EQ(pts2.size(), 0);

	std::vector<Point2f> pts3 = utils::circleLineIntersection(p1, p2, Point2f(0, 1), 1);
	EXPECT_EQ(pts3.size(), 1);
	EXPECT_EQ(pts3[0], Point2f(0, 0));
}

TEST(UtilsTest, testClosestPtOnSegment) {
	Point2f p1(0, 0);
	Point2f p2(1, 0);

	// Beyond p2.
	Point2f p3(2, 1);
	Point2f closest1 = utils::closestPtOnSegment(p1, p2, p3);
	EXPECT_EQ(1, closest1.x);
	EXPECT_EQ(0, closest1.y);

	// Before p1.
	Point2f p4(-1, -1);
	Point2f closest2 = utils::closestPtOnSegment(p1, p2, p4);
	EXPECT_EQ(0, closest2.x);
	EXPECT_EQ(0, closest2.y);

	// In the middle.
	Point2f p5(0.5, 0.5);
	Point2f closest3 = utils::closestPtOnSegment(p1, p2, p5);
	EXPECT_NEAR(0.5, closest3.x, 1e-3);
	EXPECT_NEAR(0.0, closest3.y, 1e-3);
}
