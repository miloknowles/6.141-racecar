## High Speed Obstacle Avoidance
Team 11's Final Challenge for 2018 RSS

### Setup

Catkin command line tools
```bash
catkin build obstacle_avoidance # Build the package.
catkin clean # Prepare for a fresh build.
catkin config --blacklist lab6 direct_drive # Don't build these packages (conflicting PlanningLibC installations)
```

#### Running Primitive Obstacle Avoidance
See ```cfg/params.yaml``` in ```obstacle_avoidance/``` and in ```lab6/``` for all params.
```bash
teleop # Don't need a roscore if you run this first.
roslaunch obstacle_avoidance primitive_avoidance.launch # Map server and localization.
roslaunch obstacle_avoidance primitive_planner.launch # The planner itself.
roslaunch lab6 follow_trajectory.launch # Launches Jacob's pure pursuit controller.
```
#### Running FreeSpace Obstacle Avoidance
The params are inside of ```src/free_space_planner.py```. 
```bash
teleop
rosrun obstacle_avoidance free_space_planner.py
roslaunch obstacle base_server.launch # Provides the base map to particle filter.
roslaunch obstacle localize.launch
```

#### Race
See ```lab6/cfg/params.yaml``` - make sure min and max speed are correct.
```bash
teleop
roslaunch ta_lab5 map_server.launch # Make sure this is serving the basement map.
roslaunch ta_lab5 localize.launch
roslaunch lab6 follow_trajectory.launch
roslaunch lab6 load_trajectory.launch # Run this to send waypoints to controller.
```

### PlanningLibC

An important primitive is the ```Point2``` object, which is templated into ```Point2f``` and ```Point2i```.
- ```Point2f``` objects are used for poses in continuous world space.
- ```Point2i``` objects are used for represented discrete coordinates in an occupancy grid.

```python
from PlanningLibC import *

# Create a point.
pt = Point2f(13.1, 1.7)
print(pt.x, pt.y)

# Calculate distance to another point.
pt2 = Point2f(14.7, 0.1)
dist = pt.distance(pt2) # Order doesn't matter for distance.

# Point comparison is supported.
Point2f(0.1, 0) == Point2f(0.1, 0) # True
Point2i(0, 0) != Point2i(0, 1) # True
```

The main class in PlanningLibC is the ```OccupancyGrid```. It's a wrapper around a ROS ```OccupancyGrid``` message that allows obstacle checking and some other useful operations. In general, operations can be done in grid coordinates or continuous coordinates, and conversions between the two are handled internally.

Frame convention:
- Point2f (x, y) coordinates are in the MAP frame.
- Grid cell row indices increase as y increases.
- Grid cell column indices increases as x increases.
- Grid cell (0, 0) is not necessarily at (0.0, 0.0) in the map frame, so PlanningLibC handles this translation internally.

See ```test/test_PlanningLibC.py``` for more API usage.

#### Creating an OccupancyGrid object
```python
import PlanningLibC as plc
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg # To avoid ambiguity.

# Create an example OccupancyGrid msg.
og = OccupancyGridMsg()
og.info.width = 100
og.info.height = 200
og.info.resolution = 0.1
og.data = [0 for _ in range(og.info.width * og.info.height)]

# Create an OccupancyGrid object (wrapper).
ogw = plc.OccupancyGrid(self.og)
```

#### Getting map metadata
```python
w = ogw.getMapWidth() # 100
h = ogw.getMapHeight() # 200
res = ogw.getMapResolution() # 0.1
w_m = ogw.getMapWidthMeters() # 10
h_m = ogw.getMapHeightMeters() # 20
```

#### Get/set OccupancyGrid cell data
*Note that cells have values between [-128, 127] (ROS stores an int8_t for each cell).*
```python
# Set the value of grid cell (0, 0) to 127.
ogw.setGridValue(plc.Point2i(0, 0), 127)

# Get the value of grid cell (0, 0).
val = ogw.getGridValue(plc.Point2i(0, 0)) # 127

# Set the value of grid cell corresponding to (7.3, 2.1) in the world frame to 126.
ogw.setPointValue(plc.Point2f(7.3, 2.1), 126)

# Get the value of grid cell corresponding to (7.3, 2.1) in the world frame.
val = ogw.getPointValue(plc.Point2f(7.3, 2.1)) # 126
```

#### Collision checking
PlanningLibC can check for collisions along the straight line between two points (used in RRT and others). Internally, Bresenham's Line Algorithm is used to figure out which grid cells to check for occupancy.
```python
ogw.pathOccupied(plc.Point2f(1.0, 2.0), plc.Point2f(1.7, 0.3))
```

#### Sampling from the map
```python
randPt = ogw.sampleOccupancyGrid()
```

#### Dilating the map
The function below takes in a radius of dilation, and internally modifies the map to be dilated. For a radius of 0.2m, for example, all cells within a 0.2m radius of an occupied cell become occupied.
```python
ogw.dilateOccupancyGrid(0.2) # Dilate the map by 20cm.
```

#### Get a ROS OccupancyGrid message
It may be useful to get a ROS msg representing the current map state.
```python
msg = ogw.getOccupancyGridMsg()
```
