#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>

#include "point2.hpp"
#include "pose3.hpp"
#include "pure_pursuit_controller.hpp"

using namespace plc;

/**
 * @brief Test that all 3 cases of circeLineSegmentIntersection work.
 */
TEST(PPTest, testCircleLineSegmentIntersection) {
	Point2f pt1(0, 0);
	Point2f pt2(0, 10);

	// Case 1: no intersection, should choose second point.
	Point2f c1(2, 0);
	float r1 = 1;

	Point2f inter1;
	bool res1 = circleLineSegmentIntersection(pt1, pt2, c1, r1, &inter1);
	EXPECT_EQ(pt2.x, inter1.x);
	EXPECT_EQ(pt2.y, inter1.y);
	EXPECT_FALSE(res1);

	// Case 2: single intersection, choose that point.
	float r2 = 2;
	Point2f inter2;
	bool res2 = circleLineSegmentIntersection(pt1, pt2, c1, r2, &inter2);
	EXPECT_EQ(0, inter2.x);
	EXPECT_EQ(0, inter2.y);
	EXPECT_TRUE(res2);

	// Case 3: double intersection, choose one w/ more progress.
	Point2f c2(0, 3);
	Point2f inter3;
	bool res3 = circleLineSegmentIntersection(pt1, pt2, c2, r1, &inter3);
	EXPECT_EQ(0, inter3.x);
	EXPECT_EQ(4, inter3.y);
	EXPECT_TRUE(res3);
}

/**
 * @brief Test that chooseRefInput works with a 2 point path.
 */
TEST(PPTest, testChooseRefInput) {
	PurePursuitController::Params params;
	params.minLookaheadDist = 1.0;
	params.maxLookaheadDist = 4.0;
	params.maxSpeed = 4.0;

	PurePursuitController controller(params);

	// Facing in +y direction, offset right, max speed.
	// Therefore, lookahead distance should be 4.0m.
	Pose3 initialPose1(2, 0, 1.5, 4.0);
	std::vector<Pose3> path1 = {Pose3(0, 0, 0, 2.5), Pose3(0, 10, 0, 2.5)};
	std::pair<int, Point2f> rPair1 = controller.chooseRefInput(initialPose1, path1, 0);

	EXPECT_EQ(0, rPair1.first);
	EXPECT_NEAR(0, rPair1.second.x, 1e-3);
	EXPECT_NEAR(std::sqrt(12), rPair1.second.y, 1e-3);
}

TEST(PPTest, testControlOutputStraight) {
	PurePursuitController controller;

	Pose3 pose(Point2f(0, 0), 0, 1.0);
	PurePursuitInput ref(Point2f(1, 0), 1.0);
	ControlOutput control = controller.getControlOutput(pose, ref);

	EXPECT_EQ(control.steerAngle, 0);
	EXPECT_EQ(control.speed, 1.0);
}

TEST(PPTest, testControlOutputSign) {
	PurePursuitController controller;

	Pose3 pose(Point2f(0, 0), 0, 1.0);
	PurePursuitInput ref1(Point2f(1, 1), 1.0);
	PurePursuitInput ref2(Point2f(1, -1), 1.0);

	ControlOutput c1 = controller.getControlOutput(pose, ref1);
	ControlOutput c2 = controller.getControlOutput(pose, ref2);

	// printf("Steer1: %f\n", c1.steerAngle);
	// printf("Steer2: %f\n", c2.steerAngle);

	EXPECT_GT(c1.steerAngle, 0);
	EXPECT_EQ(c1.speed, 1.0);

	EXPECT_LT(c2.steerAngle, 0);
	EXPECT_EQ(c2.speed, 1.0);
}

TEST(PPTest, testControlOutputSignBehind) {
	PurePursuitController controller;

	Pose3 pose(Point2f(0, 0), 0, 1.0);
	PurePursuitInput ref1(Point2f(-1, 1), 1.0);
	PurePursuitInput ref2(Point2f(-1, -1), 1.0);

	ControlOutput c1 = controller.getControlOutput(pose, ref1);
	ControlOutput c2 = controller.getControlOutput(pose, ref2);

	// printf("Steer1: %f\n", c1.steerAngle);
	// printf("Steer2: %f\n", c2.steerAngle);

	EXPECT_GT(c1.steerAngle, 0);
	EXPECT_EQ(c1.speed, 1.0);

	EXPECT_LT(c2.steerAngle, 0);
	EXPECT_EQ(c2.speed, 1.0);
}

TEST(PPTest, testSetRefSpeed) {
	PurePursuitController controller;

	Pose3 pose(Point2f(0, 0), 0, 1.0);
	PurePursuitInput ref(Point2f(1, 1), 2.0);

	ControlOutput control = controller.getControlOutput(pose, ref);
	EXPECT_EQ(control.speed, 2.0);
}

TEST(PPTest, testSimForward) {
	PurePursuitController controller;

	// Go straight and increase speed.
	Pose3 p1(Point2f(0, 0), 0, 1.0);
	ControlOutput c1(0, 2.0);
	Pose3 p2 = controller.simulateForward(p1, c1, 0.1);

	EXPECT_GT(p2.speed, 1.0); // Speed should increase.
	EXPECT_EQ(p2.position.y, 0);
	EXPECT_GT(p2.position.x, 0);
	EXPECT_EQ(p2.theta, 0);

	// Make a left turn.
	ControlOutput c2(0.5, 2.0);
	Pose3 p3 = controller.simulateForward(p2, c2, 0.1);

	EXPECT_GT(p3.position.x, p2.position.x);
	EXPECT_GT(p3.theta, p2.theta);

	// Make sure that y increases after the left turn.
	p3 = controller.simulateForward(p3, c2, 0.1);
	EXPECT_GT(p3.position.y, p2.position.y);

	// Make a right turn.
	ControlOutput c3(-0.5, 2.0);
	Pose3 p4 = controller.simulateForward(p3, c3, 0.1);

	EXPECT_LT(p4.theta, p3.theta);

	while (p4.theta > -0.1) {
		// Make sure that y decreases after left turn.
		p4 = controller.simulateForward(p4, c3, 0.1);
	}
	p4 = controller.simulateForward(p4, c3, 10.0);
	EXPECT_LT(p4.position.y, p3.position.y);
}

TEST(PPTest, testSpeedConverge) {
	PurePursuitController::Params params;
	params.maxAccel = 1.0;
	params.maxDecel = -1.0;
	PurePursuitController controller(params);

	// Go straight and increase speed.
	Pose3 pose(Point2f(0, 0), 0, 0);
	ControlOutput c1(0, 2.0);

	float dt = 0.5;
	int steps = (int) 2.0 / dt;
	for (int i = 0; i < steps; i++) {
		pose = controller.simulateForward(pose, c1, dt);
	}
	
	EXPECT_EQ(pose.speed, 2.0);

	ControlOutput c2(0, 0);
	for (int i = 0; i < steps; i++) {
		pose = controller.simulateForward(pose, c2, dt);
	}

	EXPECT_EQ(pose.speed, 0.0);
}
