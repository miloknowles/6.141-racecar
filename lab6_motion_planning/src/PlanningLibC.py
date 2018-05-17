# IMPORT THIS FILE INSTEAD OF THE PlanningLibC MODULE INSTALLED BY CATKIN.

# This imports the PlanningLibC wrapper built by catkin into a shared object.
# For me, this shared object was installed to: racecar_ws/devel/lib/python2.7/dist-packages/lab6
from lab6 import PlanningLibC as plc
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg

from StringIO import StringIO
import rospy

# We need to create a final wrapper class in Python to deal with serialization
# of ROS messages.
class OccupancyGrid(plc.OccupancyGridWrapper):
  def __init__(self, msg):
    """
    Initialize the OccupancyGrid from an OccupancyGrid message.
    """
    super(OccupancyGrid, self).__init__(self._to_cpp(msg))

  def _to_cpp(self, msg):
    """
    Return a serialized string from a ROS message instance.
    """
    buf = StringIO()
    msg.serialize(buf)
    return buf.getvalue()

  def _from_cpp(self, str_msg, cls):
    """
    Return a ROS message from a serialized string.
    
    str_msg: (str) serialized message
    cls: (ROS message class) e.g. sensor_msgs.msg.LaserScan
    """
    msg = cls()
    return msg.deserialize(str_msg)

  def setOccupancyGrid(self, msg):
    super(OccupancyGrid, self).setOccupancyGridString(self._to_cpp(msg))

  def getOccupancyGridMsg(self):
    """
    Returns a ROS OccupancyGrid message representing the current map.
    """
    return self._from_cpp(self.getOccupancyGridMsgString(), OccupancyGridMsg)

# These classes are defined in occupancy_grid.hpp.
# We simply define them here so they are in the same namespace
# as the OccupancyGrid class above.
class Point2f(plc.Point2f):
  def __repr__(self):
    return 'Point2f: x=%f y=%f' % (self.x, self.y)

class Point2i(plc.Point2i):
  def __repr__(self):
    return 'Point2i: x=%d y=%d' % (self.x, self.y)

class MotionPlanningTree(plc.MotionPlanningTree):
  pass

class IntegerList(plc.IntegerList):
  pass
