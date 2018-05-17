#include "occupancy_grid.hpp"

namespace plc {

void OccupancyGrid::updateCurrentOccupied() {
	currentOccupied_.clear();
  for (int ii = 0; ii < mapWidth_; ii++) {
  	for (int jj = 0; jj < mapHeight_; jj++) {
  		if (getGridValue(Point2i(ii, jj))) {
  			currentOccupied_.push_back(Point2i(ii, jj));
  		}
  	}
  }
}

void OccupancyGrid::dynamicUpdate(const nav_msgs::OccupancyGrid& msg,
																	const Pose3& tfDynInMap) {
	const int dynMapWidth = msg.info.width;
	const int dynMapHeight = msg.info.height;
	const float dynMapRes = msg.info.resolution;

	// Fill in neighborhood around a dynamic cell in case resolutions are different.
	const int cellRadius = static_cast<int>(dynMapRes / mapInfo_.resolution) / 2;

	const float dynOriginX = msg.info.origin.position.x;
	const float dynOriginY = msg.info.origin.position.y;

	for (int gi = 0; gi < msg.data.size(); gi++) {
		// Get the position of the grid cell in dynamic map frame (probably robot).
		// Also add half of the resolution to get the center of the cell!
		float xDyn = dynMapRes * static_cast<float>(gi % dynMapWidth) + dynOriginX + 0.5*dynMapRes;
		float yDyn = dynMapRes * static_cast<float>(gi / dynMapWidth) + dynOriginY + 0.5*dynMapRes;

		// Transform into the map frame.
		float xMap = tfDynInMap.position.x + xDyn*std::cos(tfDynInMap.theta) - yDyn*std::sin(tfDynInMap.theta);
		float yMap = tfDynInMap.position.y + yDyn*std::cos(tfDynInMap.theta) + xDyn*std::sin(tfDynInMap.theta);

		Point2i gridPtMap = pointToGrid(Point2f(xMap, yMap));
		for (int ii = -cellRadius; ii <= cellRadius; ii++) {
			for (int jj = -cellRadius; jj <= cellRadius; jj++) {
				Point2i cell = Point2i(ii, jj) + gridPtMap;
				setGridValueDynamic(Point2i(ii, jj) + gridPtMap, static_cast<int>(msg.data[gi]));
			}
		}
	}
}

void OccupancyGrid::updateGridData(const std::vector<int>& gridData) {
	gridData_.clear();
	for (int ii = 0; ii < gridData.size(); ii++) {
		gridData_.push_back(static_cast<int8_t>(gridData[ii]));
	}
	return;
}

void OccupancyGrid::setOccupancyGrid(const nav_msgs::OccupancyGrid& grid) {
	gridData_ = grid.data;
	staticGridData_ = grid.data; // Save a static copy of the map.

	mapInfo_ = grid.info;
	mapHeader_ = grid.header;

	// Precompute useful values.
	mapWidth_ = (int)mapInfo_.width;
	mapHeight_ = (int)mapInfo_.height;
	mapRes_ = (float)mapInfo_.resolution;
	mapWidthMeters_ = (float)mapWidth_ * mapRes_;
	mapHeightMeters_ = (float)mapHeight_ * mapRes_;

	// Pad with zeros so that the data is the proper length.
	// TODO: can you fill in one shot?
	while (gridData_.size() < (mapWidth_ * mapHeight_)) {
		gridData_.push_back(0);
		staticGridData_.push_back(0);
	}

	// Get all occupied cells.
  for (int ii = 0; ii < mapWidth_; ii++) {
  	for (int jj = 0; jj < mapHeight_; jj++) {
  		if (getGridValue(Point2i(ii, jj))) {
  			originalOccupied_.push_back(Point2i(ii, jj));
  			currentOccupied_.push_back(Point2i(ii, jj));
  		}
  	}
  }

	origin_ = mapInfo_.origin;
}

void OccupancyGrid::
	getBresenhamPoints(const Point2f& p1, const Point2f& p2, std::vector<Point2i>* gridPts) const {

	Point2i p1g = pointToGrid(p1);
	Point2i p2g = pointToGrid(p2);

	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;

	float dErr = dy / dx;
	float err = 0.0;

	// Case 1: vertical line, get all points in y-range.
	if (dx == 0) {
		int minY = std::min(p1g.y, p2g.y);
		int maxY = std::max(p1g.y, p2g.y);
		for (int y = minY; y <= maxY; y++) {
			gridPts->push_back(Point2i(p1g.x, y));
		}
		return;
	}

	// Case 2: slope between -1 and 1, increment along x.
	if (dErr >= -1 && dErr <= 1) {
		// Reorder points so that x is increasing.
		if (p2g.x < p1g.x) {
			Point2i tmp = p1g;
			p1g = p2g;
			p2g = tmp;
		}

		int y = p1g.y;
		for (int x = p1g.x; x <= p2g.x; x++) {
			gridPts->push_back(Point2i(x, y));

			err += dErr;
			while (std::abs(err) > 0.5) {
				// Increment or decrement y, and decrease the abs of err.
				if (dErr >= 0) {
					y += 1;
					err -= 1.0;
				} else {
					y -= 1;
					err += 1.0;
				}
			}
		}
		return;
	}

	// Case 3: slope between 1 and vertical, increment along y.
	else if (dErr > 1.0 || dErr < -1.0) {

		// Reorder points so that y is increasing.
		if (p2g.y < p1g.y) {
			Point2i tmp = p1g;
			p1g = p2g;
			p2g = tmp;
		}
		// dErr should now be dx / dy.
		dErr = dx / dy;
		int x = p1g.x;

		for (int y = p1g.y; y <= p2g.y; y++) {
			gridPts->push_back(Point2i(x, y));

			err += dErr;
			while (std::abs(err) > 0.5) {
				// Increment or decrement x, and decrease the abs of err.
				if (dErr >= 0.0) {
					x += 1;
					err -= 1.0;
				} else {
					x -= 1;
					err += 1.0;
				}
			}
		}
		return;
	}
}

bool OccupancyGrid::pathOccupied(const Point2f& p1, const Point2f& p2) const {
	// gridPts gets filled in with all of the grid coordinates on the
	// straight line between p1 and p2.
	std::vector<Point2i> gridPts;
	getBresenhamPoints(p1, p2, &gridPts);

	// For each grid point, check if occupied.
	// Grid values range from 0 to 255, and are -1 if the cell is unknown.
	for (auto pt : gridPts) {
		if (getGridValue(pt) != 0) {
			return true;
		}
	}

	// If no occupied grid points are on the line, return false.
	return false;
}

bool OccupancyGrid::pathOccupiedVectPoint2f(const std::vector<Point2f>& path) const {
	if (path.size() < 2) {
		return false;
	}
	for (int ii = 0; ii < path.size()-1; ii++) {
		if (pathOccupied(path[ii], path[ii+1])) {
			return true;
		}
	}
	return false;
}

bool OccupancyGrid::pathOccupiedVectPose3(const std::vector<Pose3>& path) const {
	if (path.size() < 2) {
		return false;
	}
	for (int ii = 0; ii < path.size()-1; ii++) {
		if (pathOccupied(path[ii].position, path[ii+1].position)) {
			return true;
		}
	}
	return false;
}

void OccupancyGrid::dilateOccupancyGrid(float radius, bool dilateStatic) {
  if (mapDilated_) {
    std::cout << "Warning: map was already dilated. This will dilate even more." << std::endl;
  }
 	printf("Dilating map with radius=%f \n", radius);
  int radiusCells = (int) std::ceil(radius / mapRes_); // Round up.

  // TODO: why was const ref getting bad values?
  for (Point2i pt: currentOccupied_) {
  	const int minX = std::max(0, pt.x - radiusCells);
  	const int maxX = std::min(pt.x + radiusCells, mapWidth_-1);
  	const int minY = std::max(0, pt.y - radiusCells);
  	const int maxY = std::min(pt.y + radiusCells, mapHeight_-1);

  	// Generate candidate cells in a square.
  	for (int xi = minX; xi <= maxX; xi++) {
  		for (int yi = minY; yi <= maxY; yi++) {
  			// Discard cells outside of the desired circle.
  			float xdiff = static_cast<float>(pt.x - xi);
  			float ydiff = static_cast<float>(pt.y - yi);
  			float dist = utils::norm(xdiff, ydiff);

  			if (std::floor(dist) <= (float)radiusCells) {
  				if (dilateStatic) {
  					setGridValueStatic(Point2i(xi, yi), 100);
  				} else {
  					setGridValue(Point2i(xi, yi), 100);
  				}
  			}
  		}
  	}
  }
  updateCurrentOccupied(); // Ensure currentOccupied_ reflects new state.
  mapDilated_ = true;
}

nav_msgs::OccupancyGrid OccupancyGrid::getOccupancyGridMsg() {
	nav_msgs::OccupancyGrid msg;
	msg.header = mapHeader_;
	msg.info = mapInfo_;
	msg.data = gridData_;
	return msg;
}

Point2f OccupancyGrid::sampleOccupancyGridValid() const {
	int ctr = 0;
	Point2f sample = sampleOccupancyGrid();
	while (getPointValue(sample) > 0 && ctr < maxSampleRetries_) {
		sample = sampleOccupancyGrid();
		ctr++;
	}
	return ctr == maxSampleRetries_ ? Point2f(0, 0) : sample;
}

} // namespace plc
