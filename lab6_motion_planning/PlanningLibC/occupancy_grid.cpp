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

void OccupancyGrid::setOccupancyGrid(const nav_msgs::OccupancyGrid& grid) {
	gridData_ = grid.data;

	mapInfo_ = grid.info;
	mapHeader_ = grid.header;

	// Precompute useful values.
	mapWidth_ = (int)mapInfo_.width;
	mapHeight_ = (int)mapInfo_.height;
	mapRes_ = (float)mapInfo_.resolution;
	mapWidthMeters_ = (float)mapWidth_ * mapRes_;
	mapHeightMeters_ = (float)mapHeight_ * mapRes_;

	// Pad with zeros so that the data is the proper length.
	while (gridData_.size() < (mapWidth_ * mapHeight_)) {
		gridData_.push_back(0);
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
		if (getGridValue(pt) > 0) {
			return true;
		}
	}

	// If no occupied grid points are on the line, return false.
	return false;
}

void OccupancyGrid::dilateOccupancyGrid(float radius) {
  if (mapDilated_) {
    std::cout << "Warning: map was already dilated. This will dilate even more." << std::endl;
  }
  int radiusCells = (int) std::ceil(radius / mapRes_); // Round up.

  for (const Point2i& pt: currentOccupied_) {
  	int minX = std::max(0, pt.x - radiusCells);
  	int maxX = std::min(pt.x + radiusCells, mapWidth_-1);
  	int minY = std::max(0, pt.y - radiusCells);
  	int maxY = std::min(pt.y + radiusCells, mapHeight_-1);

  	// Generate candidate cells in a square.
  	for (int xi = minX; xi <= maxX; xi++) {
  		for (int yi = minY; yi <= maxY; yi++) {
  			// Discard cells outside of the desired circle.
  			if (std::floor(std::sqrt(std::pow(pt.x - xi, 2) + std::pow(pt.y - yi, 2))) <= radiusCells) {
  				setGridValue(Point2i(xi, yi), 100);
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
