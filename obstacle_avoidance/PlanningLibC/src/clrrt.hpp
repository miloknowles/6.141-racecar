#pragma once

#include <vector>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>

#include "ros/ros.h"

#include "point2.hpp"
#include "pose3.hpp"
#include "utils.hpp"
#include "motion_planning_tree.hpp"
#include "pure_pursuit_controller.hpp"

namespace plc {

/**
 * @brief The heuristic by which nodes are chosen when connecting
 * a sample to the tree. It favors parents that would give the new
 * node the minimum cost-so-far.
 */
float forwardProgressHeuristic(const Pose3& nodePose, const Pose3& samplePose, float costSoFar);

/**
 * @brief Sample reference points from within a cone in front of the car (polar).
 * @param[in] pose The current pose of robot.
 * @param[in] meanR, sigmaR Params of normal distribution in radial direction.
 * @param[in] meanT, sigmaT Params of normal distribution in theta.
 * @param[in] rng A random number generator.
 */
Pose3 forwardProgressSampling(const Pose3& pose, float meanR, float sigmaR,
                              float meanT, float sigmaT, std::default_random_engine& rng);

/**
 * @brief Samples the goal with some probability, otherwise
 * does forward progress sampling.
 */
Pose3 biasedForwardProgressSampling(const Pose3& goalPose, float goalSampleRate,
                                    const Pose3& pose, float meanR, float sigmaR,
                                    float meanT, float sigmaT, std::default_random_engine& rng);


/**
 * @brief Returns (bool) whether or not the goal has been reached.
 * In this case, the goal is simply getting past an imaginary goal line,
 * which is perpendicular to the vector from start to goal, and passes
 * through the goal pose.
 */
bool recedingHorizonGoalTest(const Pose3& currentPose, const Pose3& goalPose,
                             const Pose3 newPose, float horizonDistance);

/**
 * @brief Makes a simple sphere marker for a pose.
 */
visualization_msgs::Marker getPointMarker(const Pose3& pose);

class CLRRT {
 public:
  /**
   * @brief Parameters used when running the CL-RRT algorithm.
   */
  struct Params {
    Params() {}
    int maxIters = 100;
    float goalSampleRate = 0.1;
    bool verbose = false;
    bool publishGraph = true;
    float goalPoseTolerance = 0.1;
    float horizonDistance = 5.0;
    float meanR = 20.0;
    float sigmaR = 10.0;
    float meanT = 0.0;
    float sigmaT = 0.785;
    float dt = 0.1;
    float targetSpeed = 2.5;
  };

  CLRRT() = default; // Disable default constructor.

  /**
   * @brief Initialize CLRRT with custom Pure Pursuit controller params (recommended).
   * Note that the CLRRT params are passed into the run method, not during init.
   */
  CLRRT(const PurePursuitController::Params& ppParams) : controller_(ppParams) {
    ROS_INFO("Initialized CLRRT with PurePursuit params.");
  }

  /**
   * @brief Runs CL-RRT until goal or max iterations reached.
   * @param[in] currentPose The current robot pose in world frame.
   * @param[in] goalPose The goal pose in world frame.
   * @param[in] graph_pub Pass in a publisher for the graph.
   * @param[in] samples_pub Pass in a publisher for the samples.
   * has reached the goal. This allows custom termination behavior.
   * @param[in] map The occupancy grid (should be recent).
   * @param[in] params See above.
   * @param[out] path A vector of poses that get filled in by CLRRT.
   */
  bool run(
    const Pose3& currentPose,
    const Pose3& goalPose,
    const ros::Publisher& graph_pub,
    const ros::Publisher& samples_pub,
    const ros::Publisher& simulate_pub,
    const OccupancyGrid& map,
    const Params& params,
    std::vector<Pose3>* path,
    std::vector<Pose3>* simPath
  );

  /**
   * @brief Simulates a Pure Pursuit controller's path.
   */
  bool simulateForward(const Pose3& startPose, const std::vector<Pose3>& refPath,
                       const float targetSpeed, const int goalIdx, float dt,
                       std::vector<Pose3>* path);

  /**
   * @brief Called at every iteration when the graph is updated.
   */
  void graphUpdatedCallback(const ros::Publisher& pub);

 private:
  MotionPlanningTree<TreeNode> tree_{};

  // Random number generator used for sampling.
  std::default_random_engine rng_{};

  // Controller used for forward simulation.
  PurePursuitController controller_{};
};

} // namespace plc
