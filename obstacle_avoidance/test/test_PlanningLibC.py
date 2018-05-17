import sys, os
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
import unittest

sys.path.append('../src/')
import PlanningLibC as plc

class IntegerListTest(unittest.TestCase):
  def test_01(self):
    il = plc.IntegerList()

    for i in range(10):
      il.append(i)

    self.assertEqual(len(il), 10)
    self.assertEqual(il[0], 0)

    il.extend([18, 19, 20])
    self.assertEqual(len(il), 13)

    il2 = il[1:4]
    self.assertEqual(len(il2), 3)

class Point2Test(unittest.TestCase):
  def test_point2f(self):
    """
    Test the Point2f struct.
    """
    pt = plc.Point2f(1.1, 2.2)
    self.assertAlmostEqual(pt.x, 1.1)
    self.assertAlmostEqual(pt.y, 2.2)
    self.assertTrue(pt == plc.Point2f(1.1, 2.2))

  def test_point2i(self):
    """
    Test the Point2i struct.
    """
    pt = plc.Point2i(1, 2)
    self.assertEqual(pt.x, 1)
    self.assertEqual(pt.y, 2)
    self.assertTrue(pt == plc.Point2i(1, 2))

class MotionPlanningTreeTest(unittest.TestCase):
  def setUp(self):
    self.mpt = plc.MotionPlanningTree()

  def test_constructor(self):
    self.assertEqual(self.mpt.numNodes(), 0)

  def test_constructor_og(self):
    og = OccupancyGridMsg()
    og.info.width = 100
    og.info.height = 200
    og.info.resolution = 0.1
    og.data = [0 for _ in range(og.info.width * og.info.height)]
    ogw = plc.OccupancyGrid(og)
    mpt = plc.MotionPlanningTree(ogw)
    self.assertTrue(mpt.occupancyGridSet())

  def test_add_node(self):
    self.mpt.addNode(plc.Point2f(1, 1), -1, 10)
    self.assertEqual(self.mpt.numNodes(), 1)
    self.assertEqual(self.mpt.getNode(0), plc.Point2f(1, 1))
    self.assertEqual(self.mpt.getParent(0), -1)
    self.assertEqual(self.mpt.getCost(0), 10)

  def test_nearest_neighbor(self):
    self.mpt.addNode(plc.Point2f(1, 1), -1, 10)
    self.mpt.addNode(plc.Point2f(10, 11), 0, 20)
    nearest = self.mpt.nearestNeighbor(plc.Point2f(1.1, 1.1))
    nearestIdx = self.mpt.nearestNeighborIndex(plc.Point2f(10.1, 11.3))

    self.assertEqual(nearest, plc.Point2f(1, 1))
    self.assertEqual(nearestIdx, 1)

    self.mpt.addNode(plc.Point2f(-1, -3), 0, 30)
    nearestIdx = self.mpt.nearestNeighborIndex(plc.Point2f(-4, -5))
    self.assertEqual(nearestIdx, 2)

  def test_nearby_indices(self):
    self.mpt.addNode(plc.Point2f(1, 1), -1, 10)
    self.mpt.addNode(plc.Point2f(10, 11), 0, 20)
    indices = self.mpt.getNearbyIndices(plc.Point2f(0, 0), 100)
    self.assertEqual(len(indices), 2)

class OccupancyGridWrapperTest(unittest.TestCase):
  def setUp(self):
    og = OccupancyGridMsg()
    og.info.width = 100
    og.info.height = 200
    og.info.resolution = 0.1
    og.data = [0 for _ in range(og.info.width * og.info.height)]
    self.og = og

  def test_constructor(self):
    ogw = plc.OccupancyGrid(self.og)
    self.assertEqual(ogw.getMapWidth(), 100)
    self.assertEqual(ogw.getMapHeight(), 200)
    self.assertAlmostEqual(ogw.getMapResolution(), 0.1)

  def test_set_occupancy_grid(self):
    ogw = plc.OccupancyGrid(self.og)
    ogw.setOccupancyGrid(self.og)
    self.assertEqual(ogw.getMapWidth(), 100)
    self.assertEqual(ogw.getMapHeight(), 200)
    self.assertAlmostEqual(ogw.getMapResolution(), 0.1)

  def test_sample_occupancy_grid(self):
    ogw = plc.OccupancyGrid(self.og)

    for i in range(100):
      pt = ogw.sampleOccupancyGrid()
      self.assertTrue(pt.x <= 100 and pt.x >= 0)
      self.assertTrue(pt.y <= 200 and pt.y >= 0)

  def test_set_get_grid_value(self):
    ogw = plc.OccupancyGrid(self.og)

    ogw.setGridValue(plc.Point2i(0, 0), 127)
    self.assertTrue(ogw.getGridValue(plc.Point2i(0, 0)), 127)

    ogw.setPointValue(plc.Point2f(0, 0), 126)
    self.assertTrue(ogw.getPointValue(plc.Point2f(0.0, 0.0)), 126)

  def test_path_occupied(self):
    ogw = plc.OccupancyGrid(self.og)
    ogw.setPointValue(plc.Point2f(1, 1), 127)
    self.assertTrue(ogw.pathOccupied(plc.Point2f(0, 0), plc.Point2f(2, 2)))

  def test_path_occupied_vect(self):
    ogw = plc.OccupancyGrid(self.og)
    ogw.setPointValue(plc.Point2f(1, 1), 127)
    path = plc.Point2fList()
    for pt in [plc.Point2f(0, 0), plc.Point2f(1.1, 1.1), plc.Point2f(1.5, 1.5)]:
        path.append(pt)
    self.assertTrue(ogw.pathOccupiedVectPoint2f(path))

  def test_dilate_occupancy_grid(self):
    ogw = plc.OccupancyGrid(self.og)
    ogw.setGridValue(plc.Point2i(0, 0), 100)
    ogw.dilateOccupancyGrid(0.2, False) # Dilate with a radius of 0.2 on the non-static map.

    expected = [
      plc.Point2i(0, 0),
      plc.Point2i(0, 1),
      plc.Point2i(0, 2),
      plc.Point2i(1, 0),
      plc.Point2i(2, 0),
      plc.Point2i(1, 1),
      plc.Point2i(1, 2),
      plc.Point2i(2, 1)
    ]
    for pt in expected:
      self.assertEqual(ogw.getGridValue(pt), 100)

  def test_get_occupancy_grid_msg(self):
    ogw = plc.OccupancyGrid(self.og)
    ogw.setGridValue(plc.Point2i(0, 0), 100)

    # Make sure that the returned msg reflects the changed cell value.
    msg = ogw.getOccupancyGridMsg()
    self.assertEqual(msg.data[0], 100)

  def test_dynamic_update(self):
    ogw = plc.OccupancyGrid(self.og)

    dynMap = OccupancyGridMsg()
    dynMap.info.width = 10
    dynMap.info.height = 10
    dynMap.info.resolution = 0.1
    dynMap.info.origin.position.x = 0.0
    dynMap.info.origin.position.y = 0.0
    dynMap.data = [0 for _ in range(100)]
    ogw.dynamicUpdate(dynMap, 0.0, 0.0, 0.0)

    # Make sure empty dynamic update keeps map empty.
    for ii in range(100):
      for jj in range(200):
        # print(ii, jj)
        self.assertEqual(ogw.getGridValue(plc.Point2i(ii, jj)), 0)

    points = [
      plc.Point2i(10, 20),
      plc.Point2i(19, 29),
      plc.Point2i(19, 20),
      plc.Point2i(10, 29),
      plc.Point2i(15, 25)
    ]
    dynMap.data = [100 for _ in range(100)]
    ogw.dynamicUpdate(dynMap, 1.0, 2.0, 0)

    for pt in points:
      self.assertEqual(ogw.getGridValue(pt), 100)

if __name__ == '__main__':
  unittest.main()
