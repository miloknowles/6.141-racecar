#include "pure_pursuit_controller.hpp"

namespace plc {

ControlOutput PurePursuitController::
	getControlOutput(const Pose3& currentPose, const PurePursuitInput& refInput) {
  // Determine steer angle.
  Point2f desVector = refInput.position - currentPose.position;
  float distToGoal = utils::norm(desVector.x, desVector.y);
  float angleToGoal = std::atan2(desVector.y, desVector.x);
  float desAngle = angleToGoal - currentPose.theta;

  float turn = std::atan2(
    params_.wheelbaseLength*std::sin(desAngle),
    0.5*distToGoal + 0.5*params_.wheelbaseLength*std::cos(desAngle));

  // Just command the desired speed for now.
  return ControlOutput(turn, refInput.speed);
}

// TODO: add sideslip terms
// TODO: add non-instant steering
Pose3 PurePursuitController::
	simulateForward(const Pose3& pose, const ControlOutput& control, float dt) {
  Pose3 nextPose;

  // First, compute time derivates.
  float xdot = pose.speed * std::cos(pose.theta);
  float ydot = pose.speed * std::sin(pose.theta);
  float thetadot = (pose.speed / params_.wheelbaseLength) * std::tan(control.steerAngle);
  float vdot = (control.speed < pose.speed) ? params_.maxDecel : params_.maxAccel;
  vdot = (vdot / fabs(vdot)) * std::min(fabs(control.speed - pose.speed) / dt, fabs(vdot));

  // Update the pose by a single dt.
  nextPose.position.x = pose.position.x + dt*xdot;
  nextPose.position.y = pose.position.y + dt*ydot;
  nextPose.theta = utils::normalizeAngle(pose.theta + dt*thetadot);
  nextPose.speed = std::max(std::min(pose.speed + dt*vdot, params_.maxSpeed), 0.0f);
  
  return nextPose;
}

std::pair<int, Point2f> PurePursuitController::
  chooseRefInput(const Pose3& currentPose, const std::vector<Pose3>& path,
                 const int currentPathIdx) {

  // Determine lookahead distance based on current speed.
  float lookaheadDist = calcLookaheadDist(currentPose.speed);
  // std::cout << "Lookahead: " << lookaheadDist << std::endl;

  // For each line segment (after currentPathidx), see if the lookahead circle intersects.
  int bestIdx = currentPathIdx;
  Point2f latestValid = path[currentPathIdx].position;

  for (int ii = currentPathIdx; ii < path.size()-1; ii++) {
    Point2f intersect; // Filled in by helper.
    bool valid = circleLineSegmentIntersection(
      path[ii].position, path[ii+1].position,
      currentPose.position, lookaheadDist, &intersect);

    // Lookahead circle has an intersection ON the segment.
    if (valid) {
      bestIdx = ii;
      latestValid = intersect;
    }
  }
  return std::pair<int, Point2f>(bestIdx, latestValid);
}

bool circleLineSegmentIntersection(const Point2f& p1, const Point2f& p2,
                                   const Point2f& c, const float r,
                                   Point2f* returnedPt) {
  // Find the closest point on the line segment.
  std::vector<Point2f> lookaheadPts = utils::circleLineIntersection(p1, p2, c, r);

  // Check that points are actually on line segment.
  std::vector<Point2f> ptsOnSegment;
  for (const Point2f& pt : lookaheadPts) {
    if ((pt.x <= p1.x && pt.x >= p2.x) || (pt.x >= p1.x && pt.x <= p2.x)) {
      ptsOnSegment.push_back(pt);
    }
  }
  // If no points, return the second line segment endpoint.
  if (ptsOnSegment.size() == 0) {
    (*returnedPt) = p2;
    return false;
    // return (p1.distance(c) <= p2.distance(c)) ? p1 : p2;

  // If a single intersection point, return that.
  } else if (ptsOnSegment.size() == 1) {
    (*returnedPt) = ptsOnSegment[0];
    return true;

  // If two intersection points, choose the one closest to the 2nd endpoint.
  // For paths in order, this should be the forward direction.
  } else {
    (*returnedPt) = (ptsOnSegment[0].distance(p2) <= ptsOnSegment[1].distance(p2)) ?
      ptsOnSegment[0] : ptsOnSegment[1];
    return true;
  }
}
  
}
