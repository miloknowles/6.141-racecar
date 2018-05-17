# Unit tests for the Particle Filter.
# Make sure you run this from the directory of this file!
# Also make sure to run roscore and roslaunch lab5_localization full_simulation.launch

import unittest
import rospy
import numpy as np
import yaml, os, math
from particle_filter import *


def setup_params():
  """
  Set params manually since these tests aren't launched through ROS.
  """
  rospy.set_param('~max_particles', 2000)
  rospy.set_param('~max_viz_particles', 100)
  rospy.set_param('~max_range', 10)
  rospy.set_param('~theta_discretization', 108)
  rospy.set_param('~range_method', 'cddt')
  rospy.set_param('~scan_topic', '/scan')
  rospy.set_param('~odometry_topic', '/odom')
  rospy.set_param('~static_map', 'static_map')

  # Change directory to that of the test, then find params and set in bulk. 
  os.chdir(os.path.realpath(os.path.dirname(__file__)))
  with open(os.path.join(os.path.realpath('..'), 'cfg/params.yaml'), 'r') as f:
    params = yaml.load(f)
    rospy.set_param('/particle_filter', params)
    print('Loaded params from cfg/params.yaml!')

class TestSensorModel(unittest.TestCase):
  def setUp(self):
    # First, set params because PF will retrieve them.
    setup_params()
    self.pf = ParticleFilter()
    print('Set up particle filter for tests.')

    # To avoid waiting for callbacks on laser and odometry, set up derived values here.
    self.pf.SCAN_ANGLE_MIN = -3.0 * math.pi / 4 # Set in first lidarCB.
    self.pf.SCAN_ANGLE_MAX = 3.0 * math.pi / 4 # Set in first lidarCB.
    self.pf.SCAN_ANGLE_DELTA = math.pi / 150.0 # Set in first lidarCB.
    self.pf.NUM_RAYS = int(abs(self.pf.SCAN_ANGLE_MAX - self.pf.SCAN_ANGLE_MIN) / self.pf.SCAN_ANGLE_DELTA)

  def test_visualize_sensor_model(self, show=True):
    """
    Shows the probability distribution of observing a range of scan distances
    against a fixed ground truth distance.
    """
    groundtruth_dist = 7.0 # meters

    # All possible measured distances.
    xs = np.linspace(0, self.pf.MAX_RANGE_METERS, self.pf.MAX_RANGE_PX)
    ys = []
    for x in xs:
      ys.append(self.pf.probability_of_range(x, groundtruth_dist))
    plt.plot(xs, ys)
    plt.title('Sensor Model')
    plt.xlabel('Measured Distance (m)')
    plt.ylabel('P(Measured Distance | Ground Truth Distance = 7m)')
    if show: plt.show()

  def test_probable_scan_01(self):
    """
    We expect the probability of a scan that matches groundtruth to be very high.
    """
    groundtruth = np.linspace(0, self.pf.MAX_RANGE_METERS, self.pf.MAX_RANGE_PX)
    measured = np.linspace(0, self.pf.MAX_RANGE_METERS, self.pf.MAX_RANGE_PX)

    prod = 1.0
    for ii in range(len(groundtruth)):
      meas = measured[ii]
      gt = groundtruth[ii]
      prod *= self.pf.probability_of_range(meas, gt)

    print('Scan probability (identical):', prod)
    return prod

  def test_probable_scan_02(self):
    """
    Creates a scan that matches groundtruth and adds noise. The probability of
    this should still be high.
    """
    groundtruth = np.linspace(0, self.pf.MAX_RANGE_METERS, self.pf.MAX_RANGE_PX)
    measured = np.linspace(0, self.pf.MAX_RANGE_METERS, self.pf.MAX_RANGE_PX)

    # Add gaussian noise.
    measured += np.random.normal(0, 0.1, measured.shape)

    prod = 1.0
    for ii in range(len(groundtruth)):
      meas = measured[ii]
      gt = groundtruth[ii]
      prod *= self.pf.probability_of_range(meas, gt)

    print('Scan probability (identical w/ noise):', prod)
    return prod

  def test_improbable_scan(self):
    """
    Creates a scan that is angled away from the groundtruth scan. We expect the
    sensor model to return a low probability of.
    """
    groundtruth = np.linspace(0, self.pf.MAX_RANGE_METERS, self.pf.MAX_RANGE_PX)

    # This time, the angle of the scan is flipped.
    measured = np.linspace(0, self.pf.MAX_RANGE_METERS, self.pf.MAX_RANGE_PX)
    measured = np.flip(measured, axis=0)

    # Add gaussian noise.
    measured += np.random.normal(0, 0.1, measured.shape)

    prod = 1.0
    for ii in range(len(groundtruth)):
      meas = measured[ii]
      gt = groundtruth[ii]
      prod *= self.pf.probability_of_range(meas, gt)

    print('Scan probability (unlikely):', prod)
    return prod

  def test_probabilities_relative(self):
    """
    Makes sure that probabilities of different scans are ordered in a way
    that makes sense.
    """
    p1 = self.test_probable_scan_01()
    p2 = self.test_probable_scan_02()
    p3 = self.test_improbable_scan()

    # Ensure that p1 > p2 > p3.
    self.assertTrue(p1 > p2)
    self.assertTrue(p2 > p3)

if __name__ == '__main__':
  unittest.main()
