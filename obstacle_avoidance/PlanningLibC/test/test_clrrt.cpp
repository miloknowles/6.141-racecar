#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>

#include "point2.hpp"
#include "pose3.hpp"
#include "pure_pursuit_controller.hpp"
#include "clrrt.hpp"

using namespace plc;

TEST(CLRRTTest, testRecedingHorizonGoalTest) {
	Pose3 currentPose(Point2f(0, 0), 0, 0);
	Pose3 goalPose(Point2f(10, 10), 0, 0);

	Pose3 failPose(Point2f(5, 10), 0, 0);
	Pose3 succPose(Point2f(11, 10), 0, 0);

	bool res1 = recedingHorizonGoalTest(currentPose, goalPose, failPose, 10);
	bool res2 = recedingHorizonGoalTest(currentPose, goalPose, succPose, 10);

	EXPECT_FALSE(res1);
	EXPECT_TRUE(res2);
}

// /**
//  * @brief The forward simulation should stabilize along a straight line.
//  */
// TEST(CLRRTTest, testSimulateForward) {
// 	CLRRT rrt;

// 	float desiredSpeed = 1.0;
// 	Pose3 start(Point2f(0, 0), 0, 1.0);
// 	PurePursuitInput startRef(Point2f(0, 0), desiredSpeed);
// 	PurePursuitInput goalRef(Point2f(1.0, 1.0), desiredSpeed);

// 	std::vector<Pose3> path;
// 	float dt = 0.1;
// 	bool success = rrt.simulateForward(start, startRef, goalRef, dt, &path);
// 	EXPECT_TRUE(success);

// 	Pose3 pose = path.back();
// 	EXPECT_NEAR(pose.position.x, 1.0, 0.5 * desiredSpeed / dt);
// 	EXPECT_NEAR(pose.position.y, 1.0, 0.5 * desiredSpeed / dt);
// }

// /**
//  * @brief Stabilize from the right of the reference line.
//  */
// TEST(CLRRTTest, testRightOfRef) {
// 	CLRRT rrt;
	
// 	float desiredSpeed = 2.0;
// 	Pose3 start(Point2f(0, -1), 0, 0); // Start to right of ref line.
// 	PurePursuitInput startRef(Point2f(0, 0), 1.0);
// 	PurePursuitInput goalRef(Point2f(1.0, 0), desiredSpeed);

// 	std::vector<Pose3> path;
// 	float dt = 0.1;
// 	bool success = rrt.simulateForward(start, startRef, goalRef, dt, &path);
// 	EXPECT_TRUE(success);

// 	Pose3 pose = path.back();
// 	EXPECT_NEAR(pose.position.x, 1.0, 0.5 * desiredSpeed / dt);
// 	EXPECT_NEAR(pose.position.y, 0, 0.5 * desiredSpeed / dt);
// }

// /**
//  * @brief Stabilize from the left of the reference line.
//  */
// TEST(CLRRTTest, testLeftOfRef) {
// 	CLRRT rrt;
	
// 	Pose3 start(Point2f(0, 1.0), 0, 0); // Start to left of ref line.
// 	PurePursuitInput startRef(Point2f(0, 0), 1.0);
// 	PurePursuitInput goalRef(Point2f(1.0, 0), 1.0);

// 	std::vector<Pose3> path;
// 	float dt = 0.1;
// 	bool success = rrt.simulateForward(start, startRef, goalRef, dt, &path);
// 	EXPECT_TRUE(success);
// }
