#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <unordered_set>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"

#include "point2.hpp"
#include "pose3.hpp"
#include "utils.hpp"

namespace plc {

typedef std::vector<Point2f> Point2fList;

class OccupancyGrid {
 public:
 	/**
 	 * @brief Constructs the occupancy grid wrapper with a map.
 	 */
 	OccupancyGrid(nav_msgs::OccupancyGrid grid) {
 		setOccupancyGrid(grid);
    // int argc;
    // char** argv;
    // ros::init(argc, argv, "clrrt_node");
    // grid_sub_ = nh_.subscribe(grid_topic_, 5, &OccupancyGrid::mapUpdateCallback, this);
    // ros::spin();
  }

  // OccupancyGrid(OccupancyGrid const &) = delete;
  // void operator=(OccupancyGrid const &x) = delete;

  OccupancyGrid() = default;

 	/**
 	 * @brief Returns whether the straight line path between two
 	 * continuous points in the map frame is occupied (has an obstacle).
 	 */
  bool pathOccupied(const Point2f& p1, const Point2f& p2) const;

  /**
   * @brief Returns whether sequence of waypoints has a collision.
   */
  bool pathOccupiedVectPoint2f(const std::vector<Point2f>& path) const;
  bool pathOccupiedVectPose3(const std::vector<Pose3>& path) const;

 	/**
 	 * @brief Returns a randomly sampled Point2f from inside the map.
   * Note: the point is in the map frame.
 	 */
  inline Point2f sampleOccupancyGrid() const {
    float randFloatx = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float randFloaty = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    return Point2f(randFloatx * mapWidthMeters_ + origin_.position.x,
                   randFloaty * mapHeightMeters_+ origin_.position.y);
  }

  /**
   * @brief Keeps sampling until a point is unoccupied, or maximum retries is hit.
   */
  Point2f sampleOccupancyGridValid() const;

  void updateGridData(const std::vector<int>& gridData);

  void mapUpdateCallback(const nav_msgs::OccupancyGrid& msg) {
    std::cout << "map update callback" << std::endl;
    dynamicUpdate(msg, Pose3(Point2f(0, 0), 0));
  }

  /**
   * @brief Dilates the OccupancyGrid by filling in adjacent cells of occupied ones.
   */
  void dilateOccupancyGrid(float radius, bool dilateStatic = false);

  /**
   * @brief Update the stored occupancy grid.
   */
  void setOccupancyGrid(const nav_msgs::OccupancyGrid& grid);

  /**
   * @brief Update the grid data with a dynamic occupancy grid.
   * The dynamic occupancy grid can be in a different frame,
   * have a different resolution, or a different size.
   * @param[in] msg The dynamic occupancy grid message.
   * @param[in] tfDynInMap The transform between the dynamic frame and the map frame.
   * For dynamic map in the robot's frame, this would simply be the robot pose.
   */
  void dynamicUpdate(const nav_msgs::OccupancyGrid& msg, const Pose3& tfDynInMap);

  /**
   * @brief Returns a vector of grid points along the line from grid
   * Point1 to Point2.
   */
  void getBresenhamPoints(const Point2f& p1, const Point2f& p2, std::vector<Point2i>* gridPts) const;

  /**
   * @brief Set the value of a grid cell in the occupancy grid.
   * Returns (bool) whether the set was successful.
   */
  inline bool setGridValue(const Point2i& gridPt, int value) {
    if (value > 127 || value < -1) {
      return false;
    }
    int idx = gridPt.x + gridPt.y * mapWidth_;
    if (idx >= gridData_.size()) {
      return false;
    } else {

      // If cell is not occupied and about to be.
      if (getGridValue(gridPt) <= 0 && value > 0) {
        currentOccupied_.push_back(gridPt);
      } else if (getGridValue(gridPt) > 0 && value <= 0) {
        // If occupied and about to be free.
        auto ref = std::find(currentOccupied_.begin(), currentOccupied_.end(), gridPt);
        if (ref != currentOccupied_.end()) {
          currentOccupied_.erase(ref);
        }
      }
      gridData_[idx] = static_cast<int8_t>(value);
      return true;
    }
  }

  inline bool setGridValueStatic(const Point2i& gridPt, int value) {
    if (value > 127 || value < -1) {
      return false;
    }
    int idx = gridPt.x + gridPt.y * mapWidth_;
    if (idx >= staticGridData_.size()) {
      return false;
    } else {
      staticGridData_[idx] = static_cast<int8_t>(value);
      return true;
    }
  }

  /**
   * @brief Sets a grid value while preserving the static map.
   * This allows dynamic updates to make any cells occupied, but
   * ensures that all occupied cells from the static map stay.
   */
  inline bool setGridValueDynamic(const Point2i gridPt, int value) {
    int idx = gridPt.x + gridPt.y * mapWidth_;
    if (idx >= gridData_.size() || value > 127 || value < -1) {
      return false;
    } else {
      int val = std::max(value, static_cast<int>(staticGridData_[gridPt.x + gridPt.y * mapWidth_]));
      gridData_[idx] = static_cast<int8_t>(val);
      return true;
    }
  }

  /**
   * @brief Set the value of the cell corresponding to a continuous
   * point in the map frame.
   * Returns (bool) whether the set was successful.
   */
  inline bool setPointValue(const Point2f& pt, int value) {
    Point2i gridPt = pointToGrid(pt);
    return setGridValue(gridPt, value);
  }

 	/**
 	 * @brief Return the occupancy grid value at gridPt.
 	 * Returns the value if in range, else returns -1.
 	 */
 	inline int getGridValue(const Point2i& gridPt) const {
    int idx = gridPt.x + gridPt.y * mapWidth_;
    if (idx < gridData_.size()) {
      return static_cast<int>(gridData_[idx]);
    } else {
      return -1;
    }
  }

  /**
   * @brief Return the occupancy grid value at a continuous point in the map frame.
   */
  inline int getPointValue(const Point2f& pt) const {
    Point2i gridPt = pointToGrid(pt);
    return getGridValue(gridPt);
  }

 	/**
 	 * @brief Converts a point in the map frame to the corresponding grid point.
 	 */
 	inline Point2i pointToGrid(const Point2f& pt) const {
    int xMapPx = static_cast<int>((pt.x - origin_.position.x) / mapRes_);
    int yMapPx = static_cast<int>((pt.y - origin_.position.y) / mapRes_);
    return Point2i(xMapPx, yMapPx);
  }

 	/**
 	 * @brief Converts a grid point to the corresponding continuous point
   * in the map frame.
 	 */
 	inline Point2f gridToPoint(const Point2i& pt) const {
    float xWorld = mapRes_ * (float)pt.x + (float)origin_.position.x;
    float yWorld = mapRes_ * (float)pt.y + (float)origin_.position.y;
    return Point2f(xWorld, yWorld);
  }

  /**
   * @brief Getter functions for map metadata.
   */
  int getMapWidth() const { return mapWidth_; }
  int getMapHeight() const { return mapHeight_; }
  float getMapResolution() const { return mapRes_; }
  float getMapWidthMeters() const { return mapWidthMeters_; }
  float getMapHeightMeters() const { return mapHeightMeters_; }

  /**
   * @brief Get the number of unoccupied cells in the occupancy grid.
   */
  int getFreeGridCount() const {
    int count = 0;
    for (int i = 0; i < gridData_.size(); i++) {
      if (gridData_[i] == 0) {
        count++;
      }
    }
    return count;
  }

  /**
   * @brief Get the number of occupied cells in the occupancy grid.
   */
  int getOccupiedGridCount() const {
    return gridData_.size() - getFreeGridCount();
  }

  /**
   * @brief Get a vector of grid points that were occupied in the original map.
   * Note: this shouldn't change after dilation.
   */
  std::vector<Point2i> getOriginalOccupied() const { return originalOccupied_; }

  /**
   * @brief Get the (x, y) location of grid cell (0, 0) in the continuous map frame.
   * Note: this is originally set by whatever service is creating the map.
   */
  Point2f getOriginPoint() const { return Point2f(origin_.position.x, origin_.position.y); }

  /**
   * @brief Returns a ROS OccupancyGrid msg from the current map.
   */
  nav_msgs::OccupancyGrid getOccupancyGridMsg();

  /**
   * @brief Rewire the tree using a certain node.
   */
  void rewireThroughNode(int nodeIdx);

 private:
  /**
   * @brief Recompute the list of currently occupied cells.
   * Try not to call this very often - it's inefficient.
   */
  void updateCurrentOccupied();

 	std_msgs::Header mapHeader_{};
  nav_msgs::MapMetaData mapInfo_{};
  std::vector<int8_t> gridData_{};
  std::vector<int8_t> staticGridData_{};

  int mapWidth_ = 0; // cells in the x direction (cols).
  int mapHeight_ = 0; // cells in the y direction (rows).
  float mapWidthMeters_ = 0;
  float mapHeightMeters_ = 0;
  float mapRes_ = 0; // meters per cell.

  bool mapDilated_ = false;
  std::vector<Point2i> originalOccupied_{}; // occupied cells in the original map.
  std::vector<Point2i> currentOccupied_{};

  // The world frame pose of the cell (0, 0).
  geometry_msgs::Pose origin_{};

  // Max retries for sampleOccupancyGridValid.
  int maxSampleRetries_ = 100;
};

} // namespace plc
