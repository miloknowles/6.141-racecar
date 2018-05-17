#include "utils.hpp"

namespace plc {

namespace utils {

/**
 * @brief Normalize an angle between -pi, pi.
 */
float normalizeAngle(float angle) {
	return angle - 2*3.14159*std::floor((angle + 3.14159) / (2*3.14159));
}

/**
 * @brief Path length of a sequence of poses.
 */
float pathLength(const std::vector<Pose3>& path) {
  if (path.size() < 2) {
    return 0.0;
  }
  float total = 0;
  for (int ii = 0; ii < path.size()-1; ii++) {
    total += path[ii].distance(path[ii+1]);
  }
  return total;
}

/**
 * @brief Return the angle between two vectors.
 */
float vectorAngle(float x1, float y1, float x2, float y2) {
	return std::acos((x1*x2 + y1*y2 / (norm(x1, y1) * norm(x2, y2))));
}

/**
 * @brief Finds the points of intersection between a line and circle.
 * Can return 0, 1, or 2 points.
 * Equations from: http://mathworld.wolfram.com/Circle-LineIntersection.html
 * Note: line is shifted so that circle is at the origin, then shifted back.
 */
std::vector<Point2f> circleLineIntersection(const Point2f p1l, const Point2f p2l, const Point2f c, float r) {
  Point2f p1 = p1l - c;
  Point2f p2 = p2l - c;

  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  float dr = utils::norm(dx, dy);
  float D = p1.x * p2.y - p2.x * p1.y;
  float discrim = pow(r, 2) * pow(dr, 2) - pow(D, 2);
  std::vector<Point2f> pts;

  // No intersection.
  if (discrim < 0) {
    return pts;
  // Tangent to circle.
  } else if (discrim == 0) {
    pts.push_back(Point2f(D*dy / pow(dr, 2), -D*dx / pow(dr, 2)) + c);
  // Double intersection.
  } else {
    Point2f plus((D*dy + sign(dy)*dx*std::sqrt(discrim)) / pow(dr, 2),
                 (-1*D*dx + abs(dy)*std::sqrt(discrim)) / pow(dr, 2));
    Point2f minus((D*dy - sign(dy)*dx*std::sqrt(discrim)) / pow(dr, 2),
                 (-1*D*dx - abs(dy)*std::sqrt(discrim)) / pow(dr, 2));
    pts.push_back(plus + c);
    pts.push_back(minus + c);
  }
  return pts;
}

visualization_msgs::Marker makeGraphMarker(const std::vector<Point2f> lineList,
                                           float duration, bool show) {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.ns = "visualization/graph";
  m.id = 101;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.lifetime = ros::Duration(duration);

  if (show) {
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.r = 1.0;
    m.color.a = 0.5;

    for (const Point2f& pt : lineList) {
      geometry_msgs::Point p;
      p.x = pt.x, p.y = pt.y;
      m.points.push_back(geometry_msgs::Point(p));
    }
  } else {
    m.action = visualization_msgs::Marker::DELETE; // Hide.
  }
  return m;
}

Point2f closestPtOnSegment(const Point2f& s1, const Point2f& s2, const Point2f& p) {
  float dotprod = dot(p - s1, s2 - s1);
  if (dotprod <= 0) {
    return s1;
  } else if (dotprod >= norm(s2 - s1)) {
    return s2;
  } else {
    return (s1 + (s2 - s1) * dotprod);
  }
}

} // namespace utils

} // namespace plc
