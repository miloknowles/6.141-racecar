#pragma once

#include <cmath>

#include "point2.hpp"
#include "pose3.hpp"
#include "utils.hpp"

namespace plc {

/**
 * @brief Finds the point of intersection between a finite line
 * segment and a circle.
 * Case 1: (no intersection) Return the *second* line segment endpoint.
 * Case 2: (single) Return the single intersection point.
 * Case 3: (two) Return the intersection point closest to the 2nd
 * endpoint. Line segments should "point" in the direction of progress.
 */
bool circleLineSegmentIntersection(const Point2f& p1, const Point2f& p2,
                                  const Point2f& c, const float r,
                                  Point2f* returnedPt);

/**
 * @brief The reference input to a Pure Pursuit controller, which
 * includes a reference point in the world frame, and a target speed.
 */
struct PurePursuitInput {
  PurePursuitInput(const Point2f& p, float s) : position(p), speed(s) {}
  PurePursuitInput() = delete;

  Point2f position;
  float speed;
};

/**
 * @brief Represents a desired control action (for a VESC-like controller).
 */
struct ControlOutput {
  ControlOutput(float theta, float speed) : steerAngle(theta), speed(speed) {}
  ControlOutput() = delete;

  float steerAngle = 0;
  float speed = 0;
};

/**
 * @brief A controller that uses the Pure Pursuit steering law to
 * target a reference point in its current frame.
 */
class PurePursuitController {
 public:
  /**
   * @brief Parameters for the Pure Pursuit controller.
   * Details in http://acl.mit.edu/papers/KuwataGNC08.pdf (page 16).
   */
  struct Params {
    Params() {}
    float maxTurnAngle = 0.5; // radians
    float maxSlewRate = 0.3294; // radians/sec
    float wheelbaseLength = 0.33; // meters
    float maxAccel = 4.0; // m/s^2
    float maxDecel = -4.0; // m/s^2
    float maxSpeed = 4.5; // m/s
    float minLookaheadDist = 1.0;
    float maxLookaheadDist = 4.0;
  };

  /**
   * @brief Constructor with custom params.
   */
  PurePursuitController(const Params& params) : params_(params) {
    std::cout << "Initialized PurePursuitController with params." << std::endl;
  }
  PurePursuitController() = default;

  /**
   * @brief Returns a control action given by the instantaneous
   * pure pursuit steering law.
   * @param[in] currentPose The pose of the robot in world frame.
   * @param[in] refInput A reference point and speed.
   */
  ControlOutput getControlOutput(const Pose3& currentPose,
                                 const PurePursuitInput& refInput);

  /**
   * @brief Uses a PI controller to determine a drive action
   * given the desired control action and current pose.
   */


  /**
   * @brief Determine speed-dependent lookahead distance.
   */
  float calcLookaheadDist(float speed) {
    float dist = (speed / params_.maxSpeed) * params_.maxLookaheadDist;
    dist = std::min(std::max(dist, params_.minLookaheadDist), params_.maxLookaheadDist);
    return dist;
  }

  /**
   * @brief Find the best reference input given the current pose and
   * a path of waypoints.
   * (Case 1) For all line segments that intersect the current lookahead circle,
   * choose the one with HIGHEST index.
   * (Case 2) If no line segments intersect, choose the reference point with
   * currentPathIdx.
   * Returns: (index, refPt)
   *   (Case 1) The endpoint of best line segment.
   *   (Case 2) currentPathIdx
   */
  std::pair<int, Point2f> chooseRefInput(const Pose3& currentPose,
                                         const std::vector<Pose3>& path,
                                         const int currentPathIdx);

  /**
   * @brief Uses the model in http://acl.mit.edu/papers/KuwataGNC08.pdf
   * to forward simulate the dynamics of the vehicle over one timestep.
   */
  Pose3 simulateForward(const Pose3& pose, const ControlOutput& control, float dt = 0.1);

  /**
   * @brief Update the params to allow for dynamic reconfiguration.
   */
  void updateParams(const Params& params) {
    params_ = params;
  }

 private:
   Params params_{};
};

} // namespace plc
