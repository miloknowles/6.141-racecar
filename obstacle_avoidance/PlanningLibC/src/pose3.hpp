#pragma once

#include "point2.hpp"

namespace plc {

struct Pose3 {
  Pose3(float x, float y, float theta, float speed = 0) :
    position(x, y), theta(theta), speed(speed) {}

  Pose3(const Point2f& pos, float theta, float speed = 0) :
    position(pos), theta(theta), speed(speed) {}

  Pose3() = default;

  Point2f position{};
  float theta = 0;
  float speed = 0;

  // Overload useful operators.
  bool operator==(const Pose3& other) const {
    return (this->position == other.position && this->theta == other.theta);
  }
  bool operator!=(const Pose3& other) const {
    return !(*this == other);
  }
  Pose3 operator+(const Pose3& other) const {
    return Pose3(this->position + other.position, this->theta + other.theta);
  }
  Pose3 operator-(const Pose3& other) const {
    return Pose3(this->position - other.position, this->theta + other.theta);
  }
  float distance(const Pose3& other) const {
    return ((this->position).distance(other.position));
  }
};

}
