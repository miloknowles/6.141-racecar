import math
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg

class OccupancyGrid:

    def __init__(self, msg): #gridData, resolution, width, height, origin):
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        origin = msg.info.origin.position
        self.origin = origin.x, origin.y

        self.info = msg.info

        self.dilated = False
        self.setGridData(msg.data) # Provide as a 2d numpy array (y first dimension)

        assert self.gridData.shape == (self.height, self.width)

    def setGridData(self, gridData):
        self.gridData = np.array(gridData).reshape((self.height, self.width))
        return self.gridData

    def pointToGrid(self, pt):
        xPx = (pt[0] - self.origin[0]) / self.resolution
        yPx = (pt[1] - self.origin[1]) / self.resolution

        xPx = int(round(xPx))
        yPx = int(round(yPx))

        return (xPx, yPx)

    def getOccupancyGridMsg(self):
        msg = OccupancyGridMsg()
        msg.info = self.info
        msg.data = self.gridData.flatten()
        return msg

    def getBresenhamPoints(self, pt1, pt2):
        # bresenham_pts = []

        p1g = self.pointToGrid(pt1)
        p2g = self.pointToGrid(pt2)

        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]

        err = 0.0

        # Case 1: vertical line
        if dx == 0:
            miny = min(p1g[1], p2g[1])
            maxy = max(p1g[1], p2g[1])
            for i in range(miny, maxy+1):
                # bresenham_pts.append((p1g[0], i))
                yield (p1g[0], i)
            # return bresenham_pts
            return

        derr = math.fabs(dy * 1. / dx)

        # Case 2: slope between -1 and 1, increment along x
        if (derr >= -1 and derr <= 1):
            if p2g[0] < p1g[0]: # Reorder so x is increasing
                p1g, p2g = p2g, p1g

            y = p1g[1]
            for x in range(p1g[0], p2g[0]+1):
                # bresenham_pts.append((x, y))
                yield (x, y)
                err += derr
                while math.fabs(err) > 0.5:
                    if derr >= 0:
                        y += 1
                        err -= 1.0
                    else:
                        y -= 1
                        err += 1.0
            # return bresenham_pts
            return

        # Case 3: slope between 1 and vertical, increment along y.
        if (derr > 1 or derr < -1):
            if p2g[1] < p1g[1]: # Reorder so y is increasing
                p1g, p2g = p2g, p1g
            derr = dx / dy

            x = p1g[0]
            for y in range(p1g[1], p2g[1]+1):
                # bresenham_pts.append((x, y))
                yield (x, y)
                err += derr
                while math.fabs(err) > 0.5:
                    if derr >= 0.0:
                        x += 1
                        err -= 1.0
                    else:
                        x -= 1
                        err += 1.0
                # return bresenham_pts
                return

        assert False

    def lineOccupied(self, pt1, pt2):
        bresenham_pts = self.getBresenhamPoints(pt1, pt2)
        for x, y in bresenham_pts:
            if (x < self.width and y < self.height) and self.gridData[y][x] != 0:
                return True
        return False

    def pathOccupied(self, path):
        if len(path) < 2:
            return False
        for i, j in zip(path, path[1:]):
            if self.lineOccupied(i, j):
                return True
        return False

    def dilateOccupancyGrid(self, radius, _=None):
        if self.dilated:
            print("Warning: map was already dilated. This will dilate even more.")
        print("Dilating map with radius {}")
        radiusPx = int(math.ceil(2 * radius / self.resolution))

        self.gridData = cv2.dilate(self.gridData.astype(float), np.ones((radiusPx, radiusPx)))
        self.dilated = True
