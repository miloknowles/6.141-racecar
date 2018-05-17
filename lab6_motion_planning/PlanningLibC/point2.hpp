#pragma once

#include <cmath>
#include <iostream>
#include <vector>

namespace plc {

// Prints out a vector of Point2 objects.
template <typename T>
void printPointVect(const std::vector<T>& v) {
	for (auto vi : v) {
		std::cout << "(" << vi.x << ", " << vi.y << ") ";
	}
	std::cout << std::endl;
}

// Prints out a single Point2 object.
template <typename T>
void printPt(T pt) {
	std::cout << "Pt: x=" << pt.x << " y=" << pt.y << std::endl;
}

template <typename T>
struct Point2 {
	Point2(T xin, T yin) : x(xin), y(yin) {};
	T x;
	T y;

  // Overload useful operators.
  bool operator==(const Point2 &other) const {
    return (this->x == other.x && this->y == other.y);
  }
  bool operator!=(const Point2 &other) const {
    return !(*this == other);
  }
  Point2 operator+(const Point2 &other) const {
    return Point2(this->x + other.x, this->y + other.y);
  }
  Point2 operator-(const Point2 &other) const {
    return Point2(this->x - other.x, this->y - other.y);
  }
  float distance(const Point2 &other) const {
    return std::sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
  }
};

// Typedefs for floating and integer points.
using Point2f = Point2<float>;
using Point2i = Point2<int>;

} // namespace plc
