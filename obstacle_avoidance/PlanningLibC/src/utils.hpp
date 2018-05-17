#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <chrono>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point32.h"

#include "pose3.hpp"
#include "point2.hpp"

namespace plc {

namespace utils {

float normalizeAngle(float angle);

/**
 * @brief Magnitude of a 2d vector.
 */
template <typename T>
float norm(T x, T y) {
  return std::sqrt(pow(x, 2) + pow(y, 2));
}

template <typename T>
float norm(const Point2<T>& p) {
  return norm(p.x, p.y);
}

/**
 * @brief Returns the dot product of two vector-like objects.
 */
template <typename T>
float dot(T v1, T v2) {
	return (v1.x*v2.x + v1.y*v2.y);
}

float pathLength(const std::vector<Pose3>& path);

float vectorAngle(float x1, float y1, float x2, float y2);

template <typename T>
int sign(const T& v) {
  return (v < 0) ? -1 : 1;
}

// Equations from: http://mathworld.wolfram.com/Circle-LineIntersection.html
std::vector<Point2f> circleLineIntersection(const Point2f p1, const Point2f p2, const Point2f c, float r);

inline void printPose3(const Pose3& pose) {
	printf("Pose3: x=%f y=%f theta=%f speed=%f \n", pose.position.x, pose.position.y, pose.theta, pose.speed);
}

/**
 * @brief Find a parameter or fail.
 */
template <typename T>
void getParamOrFail(const ros::NodeHandle& nh, const std::string& name, T* val) {
  if (!nh.getParam(name, *val)) {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(name, true).c_str());
    exit(1);
  }
  return;
}

visualization_msgs::Marker makeGraphMarker(const std::vector<Point2f> lineList,
                                           float duration, bool show);

/**
 * @brief Return the closest point to point p on segment (s1, s2).
 */
Point2f closestPtOnSegment(const Point2f& s1, const Point2f& s2, const Point2f& p);

/**
 * @brief Elapsed time.
 */
using Clocktime = std::chrono::steady_clock::time_point;
inline void printElapsed(const Clocktime& begin, const Clocktime& end) {
  std::cout << "Time difference (ms) = " << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << std::endl;
}

} // namespace utils

} // namespace plc
