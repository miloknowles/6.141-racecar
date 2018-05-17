#!/usr/bin/env python

'''
Lab 5 Starter Code

- Outlines a basic implementation of particle filter localization algorithm
- Initializes RangeLibc
- Uses ROS params for easier configuration
- Only sends visualization if someone is listening
- Uses locks to avoid concurreny errors
- Includes code for visualizing discretized sensor models with matplotlib
- Includes helper functions
    - Timer
    - CircularArray
    - Utils
        - coordinate space conversions
        - useful ROS object construction functions

While developing, be careful of:
    - coordinate conversions
    - vectorization with numpy
    - caching and reusing large buffers
    - concurrency problems
    - implement visualization early to help debugging

To launch:

    first start the map server:
    $ roslaunch lab5 map_server.launch
    then launch this code with:
    $ roslaunch lab5 localize.launch

Written by Corey Walsh for Spring 2017 6.141 Lab 5
'''
import math

import rospy
import numpy as np

from std_msgs.msg import String, Header, Float32MultiArray, Float32
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from geometry_msgs.msg import (
  Point, Pose, PoseStamped, PoseArray, Quaternion,
  PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped,
  PointStamped, TransformStamped, Vector3
)

from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import tf.transformations
import tf
import matplotlib.pyplot as plt
import range_libc
import time
import utils as Utils

from threading import Lock

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

class ParticleFilter():

  def __init__(self):
    # Sensor model params.
    self.PROBABILITY_OF_KNOWN_OBSTACLE = Utils.getParamOrFail('/particle_filter/sensor_model/probability_of_known_obstacle') # 0.5
    self.PROBABILITY_OF_SHORT_MEASUREMENT = Utils.getParamOrFail('/particle_filter/sensor_model/probability_of_short_measurement') # 0.35
    self.PROBABILITY_OF_MISSED_MEASUREMENT = Utils.getParamOrFail('/particle_filter/sensor_model/probability_of_missed_measurement') # 0.10
    self.PROBABILITY_OF_RANDOM_MEASUREMENT = Utils.getParamOrFail('/particle_filter/sensor_model/probability_of_random_measurement') # 0.10
    self.GAUSSIAN_STD_DEV = Utils.getParamOrFail('/particle_filter/sensor_model/known_obstacle_standard_deviation') #  0.3

    ### Sensor model parameters. ###
    self.SCAN_ANGLE_MIN = -3.0 * math.pi / 4 # Set in first lidarCB.
    self.SCAN_ANGLE_MAX = 3.0 * math.pi / 4 # Set in first lidarCB.
    self.SCAN_ANGLE_DELTA = math.pi / 150.0 # Set in first lidarCB.

    # The number of rays from our lidar. This value is also used for ray casting.
    self.NUM_RAYS = int(abs(self.SCAN_ANGLE_MAX - self.SCAN_ANGLE_MIN) / self.SCAN_ANGLE_DELTA)

    ### Filter parameters. ###
    self.MAX_PARTICLES = int(rospy.get_param("~max_particles"))
    self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles"))
    self.MAX_RANGE_METERS = float(rospy.get_param("~max_range"))
    self.MAX_RANGE_PX = 198 # Note: this gets set to 198 usually in self.get_omap().
    self.METERS_PER_PX = self.MAX_RANGE_METERS * 1. / self.MAX_RANGE_PX
    self.MOTION_MODEL_SIGMA = Utils.getParamOrFail('/particle_filter/motion_model/sigma')

    # cddt and glt range methods are discrete, this defines number of discrete thetas
    self.THETA_DISCRETIZATION = int(rospy.get_param("~theta_discretization"))
    self.WHICH_RANGE_METHOD = rospy.get_param("~range_method", "cddt")

    self.SUBSAMPLING_BIN_SIZE = 10 # Only take 1/this value of the scans

    # various data containers used in the MCL algorithm
    self.map_info = None
    self.map_initialized = False
    self.lidar_initialized = False
    self.odom_initialized = False
    self.range_method = None

    # Use this lock for controlling accesses to the particles necessary for avoiding concurrency errors.
    self.state_lock = Lock()

    # Large array allocation.
    self.queries = None
    self.ground_ranges_1d = None
    self.ground_ranges_2d = None
    self.sensor_model_table = None
    self.laser_angles = None

    ### Filter state. ###
    # Contains the (x, y, theta) pose for each particle.
    self.particles = np.zeros((self.MAX_PARTICLES, 3), dtype=np.float32)
    self.particle_indices = np.arange(self.particles.shape[0])

    # Initialize weights to a uniform prior.
    self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
    self.initialize_particles_gaussian(np.array([0.0, 0.0, 0.0]), np.array([0.1, 0.1, 0.01]))

    # Stores the latest laser scan data received.
    self.latest_scan_data = None

    # Initialize the state.
    self.get_omap()
    self.precompute_sensor_model(plot=False) # Optionally plot in 3D.

    # These topics are for visualization, feel free to add, remove, or change as you see fit.
    self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 10)
    self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 10)
    self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan", LaserScan, queue_size = 10)

    self.ground_truth_delta = rospy.Publisher("/ground_truth_delta", Float32, queue_size=1)

    # Use this for your inferred transformations.
    self.pub_tf = tf.TransformBroadcaster()

    # These topics are to receive data from the racecar.
    self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=10)
    self.odom_sub  = rospy.Subscriber(rospy.get_param("~odometry_topic", "/vesc/odom"), Odometry, self.odomCB, queue_size=10)

    # This is to send an odometry message with the inferred pose, useful for compatibility with
    # the simulator (since simulator odometry data is perfect) and also REQUIRED for autograding.
    self.odom_pub = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 10)

    # These integrate with RViz.
    self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=10)
    self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=10)

    # Store previous position of car relative to start pt of vehicle.
    self.rel_odom_pose = PoseStamped()

    # Store relative offsets of car from last time stamp (delta_x, delta_y, delta_theta).
    self.movement = np.zeros(3)

    self.first_sensor_update = True

    # store sigma values for guassian distribution for clicked pose
    self.sigma = np.array([0.1, 0.1, 0.01])

    rospy.loginfo("Finished initializing ParticleFilter, waiting on messages...")

  def get_omap(self):
    # this way you could give it a different map server as a parameter
    map_service_name = rospy.get_param("~static_map", "static_map")
    rospy.loginfo("Getting map from service: %s", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

    self.map_info = map_msg.info
    oMap = range_libc.PyOMap(map_msg)
    # this value is the max range used internally in RangeLibc
    # it also should be the size of your sensor model table

    # With MAX_RANGE_METERS of 10m and the default map resolution, this gets set to 198.
    self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

    # initialize range method
    rospy.loginfo("Initializing range method: %s", self.WHICH_RANGE_METHOD)
    if self.WHICH_RANGE_METHOD == "bl":
      self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
    elif "cddt" in self.WHICH_RANGE_METHOD:
      self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
      if self.WHICH_RANGE_METHOD == "pcddt":
        rospy.loginfo("Pruning...")
        self.range_method.prune()
    elif self.WHICH_RANGE_METHOD == "rm":
      self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
    elif self.WHICH_RANGE_METHOD == "rmgpu":
      self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
    elif self.WHICH_RANGE_METHOD == "glt":
      self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
    rospy.loginfo("Done loading map.")

     # 0: permissible, -1: unmapped, large value: blocked
    array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

    # 0: not permissible, 1: permissible
    # this may be useful for global particle initialization - don't initialize particles in non-permissible states
    self.permissible_region = np.zeros_like(array_255, dtype=bool)
    self.permissible_region[array_255==0] = 1
    self.map_initialized = True

  def publish_tf(self, pose, stamp=None):
    """
    Publish a tf from map to inferred_scan_frame.
    http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20(Python)
    """
    # Publish the tf from map to inferred_scan_frame.
    self.pub_tf.sendTransform((pose[0], pose[1], 0.0),
                              tf.transformations.quaternion_from_euler(0, 0, pose[2]),
                              stamp,
                              "inferred_scan_frame",
                              "map")

    # Need to account for constant offset from laser frame to base_link.
    self.pub_tf.sendTransform((-0.265, 0.0, 0),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              stamp,
                              "inferred_base_link",
                              "inferred_scan_frame")

    self.pub_tf.sendTransform((0, 0, 0),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              stamp,
                              "base_link",
                              "inferred_base_link")

    # Also publish the inferred pose as an odometry message in the map frame.
    odom = Odometry()
    odom.header = Utils.make_header("/map", stamp)
    odom.pose.pose.position.x = pose[0]
    odom.pose.pose.position.y = pose[1]
    odom.pose.pose.orientation = Utils.angle_to_quaternion(pose[2])
    ground_truth_x = self.rel_odom_pose.pose.position.x
    ground_truth_y = self.rel_odom_pose.pose.position.y
    delta = ((pose[0]-ground_truth_x - math.cos(pose[2]) * 0.265)**2 + (pose[1]-ground_truth_y - math.sin(pose[2]) * 0.265)**2)**0.5
    self.odom_pub.publish(odom)
    self.ground_truth_delta.publish(delta)

  def visualize(self):
    # Visualize the current population of particles.
    if self.particle_pub.get_num_connections() > 0:

      # If too many particles, downsample.
      if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
        proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
        self.publish_particles(self.particles[proposal_indices,:])
      else:
        self.publish_particles(self.particles)

    if self.pub_fake_scan.get_num_connections() > 0 and isinstance(self.latest_scan_data, np.ndarray):
      """
      Generate the scan from the point of view of the inferred position for
      visualization. This should always align with the map, if not then you are
      probably using RangeLibc wrong or you have a coordinate space issue.
      """
      # We query a single pose at many angles.
      self.viz_queries[:,0] = self.inferred_pose[0]
      self.viz_queries[:,1] = self.inferred_pose[1]
      self.viz_queries[:,2] = self.inferred_pose[2]
      # self.viz_queries[:,2] = self.downsampled_angles + self.inferred_pose[2]
      self.range_method.calc_range_repeat_angles(self.viz_queries, self.laser_angles, self.viz_ranges)
      self.publish_scan(self.laser_angles, self.viz_ranges)

  def publish_particles(self, particles):
    pa = PoseArray()
    pa.header = Utils.make_header("map")
    pa.poses = Utils.particles_to_poses(particles)
    self.particle_pub.publish(pa)

  def publish_scan(self, angles, ranges):
    ls = LaserScan()
    ls.header = Utils.make_header("inferred_scan_frame") # This fills in current time for us.
    ls.angle_min = np.min(angles)
    ls.angle_max = np.max(angles)
    ls.angle_increment = np.abs(angles[0] - angles[1])
    ls.range_min = 0
    ls.range_max = np.max(ranges)
    ls.ranges = ranges
    self.pub_fake_scan.publish(ls)

  def lidarCB(self, msg):
    if not isinstance(self.laser_angles, np.ndarray):
      rospy.loginfo('Received first lidarCB.')

      # Update the bounds of the scan data now that they're known.
      self.SCAN_ANGLE_DELTA = msg.angle_increment
      self.SCAN_ANGLE_MIN = msg.angle_min
      self.SCAN_ANGLE_MAX = msg.angle_max
      self.NUM_RAYS = int(abs(self.SCAN_ANGLE_MAX - self.SCAN_ANGLE_MIN) / self.SCAN_ANGLE_DELTA)

      # Get a range of laser angles to be used for RangeLibC queries.
      self.laser_angles = np.linspace(self.SCAN_ANGLE_MIN, self.SCAN_ANGLE_MAX, self.NUM_RAYS).astype(np.float32)

      self.viz_queries = np.zeros((1, 3), dtype=np.float32)
      self.viz_ranges = np.zeros_like(self.laser_angles)

      # store anything used in MCL update
      self.lidar_initialized = True

    self.latest_scan_data = np.array(msg.ranges, dtype=np.float32)

  # Odometry data is accumulated via dead reckoning, so it is very inaccurate
  # this function determines relative shift in the coordinate space of the car
  # on the simulator, the odometry data is perfect state information
  def odomCB(self, msg):
    """
    This is called every time an Odometry message is received. It triggers an MCL update
    since odometry updates arrive at a slower rate than lidar.
    """
    position = msg.pose.pose.position
    heading = Utils.quaternion_to_angle(msg.pose.pose.orientation)

    prev_position = self.rel_odom_pose.pose.position
    prev_heading = Utils.quaternion_to_angle(self.rel_odom_pose.pose.orientation)

    # Update movement vector (the "action" that is ultimately passed into the motion model).
    self.movement[0] = position.x - prev_position.x
    self.movement[1] = position.y - prev_position.y
    self.movement[2] = heading - prev_heading

    # Update cumulative odometry.
    self.rel_odom_pose.header.stamp = msg.header.stamp
    self.rel_odom_pose.pose = msg.pose.pose

    self.odom_initialized = True

    # Do MCL update here.
    self.update()

  def clicked_pose(self, msg):
    """
    This is callled every time you use the '2D Pose Estimate' tool in RViz.
    """
    # YOUR CODE - initialize particle filter when a pose is clicked in RViz
    # either a PointStamped or PoseWithCovarianceStamped depending on which RViz tool is used
    mu = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                  Utils.quaternion_to_angle(msg.pose.pose.orientation)])
    # if(type(msg) is PoseWithCovarianceStamped and sum(msg.pose.covariance) is not 0.0):
      # self.sigma = np.sqrt(np.array([msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35]]))
    self.initialize_particles_gaussian(mu, self.sigma)

  def initialize_particles_gaussian(self, mu, sigma):
    """
    Initalizes particles in a normal distribution described by params mu and sigma.

    mu: (length 3 np.ndarray) The mean for x, y, and theta.
    sigma: (length 3 np.ndarray) The std for x, y, and theta.
    """
    # sigma *= 0.02
    self.state_lock.acquire()
    rospy.loginfo('Initializing particles from gaussian.')
    rospy.loginfo('mu=%s sigma=%s', str(mu), str(sigma))

    self.particles[:,0] = np.random.normal(mu[0], sigma[0], self.particles.shape[0])
    self.particles[:,1] = np.random.normal(mu[1], sigma[1], self.particles.shape[0])
    self.particles[:,2] = np.random.normal(mu[2], sigma[2], self.particles.shape[0])
    self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

    self.state_lock.release()

  def precompute_sensor_model(self, plot=False):
    rospy.loginfo("Precomputing sensor model.")

    # this should be the size of your precomputed table if you are using RangeLibc (which you should)
    # if your table is not this size, you may encounter at best weirdness and at worst segfaults
    # MAX_RANGE_PX is simply the max range in meters scaled by the resolution of the map: self.map_info.resolution
    table_width = int(self.MAX_RANGE_PX) + 1
    self.sensor_model_table = np.zeros((table_width,table_width))

    # Call our sensor probability model for each discrete point in the table.
    for i in xrange(table_width): # Expected pixel.
      for j in xrange(table_width): # Measured pixel.
        x = j * self.MAX_RANGE_METERS * 1. / self.MAX_RANGE_PX # Measured distance (meters).
        y = i * self.MAX_RANGE_METERS * 1. / self.MAX_RANGE_PX # Expected distance (meters).
        self.sensor_model_table[i][j] = self.probability_of_range(x, y)

    # Make sure each row is normalized.
    #for i in xrange(table_width):
    #  self.sensor_model_table[i] /= np.sum(self.sensor_model_table[i])

    # Scale.
    self.sensor_model_table *= 20

    # Upload the sensor model to RangeLib for ultra fast resolution later.
    self.range_method.set_sensor_model(self.sensor_model_table)

    # Visualize the sensor model.
    if plot:
      fig = plt.figure()
      ax = fig.gca(projection='3d')

      # Make a meshgrid to plot probability values over.
      X = np.arange(0, table_width, 1.0)
      Y = np.arange(0, table_width, 1.0)
      X, Y = np.meshgrid(X, Y)

      # Plot the surface.
      surf = ax.plot_surface(X, Y, self.sensor_model_table, cmap="bone", rstride=2, cstride=2,
                             linewidth=0, antialiased=True)

      ax.text2D(0.05, 0.95, "Precomputed Sensor Model", transform=ax.transAxes)
      ax.set_xlabel('Ground truth distance (in px)')
      ax.set_ylabel('Measured Distance (in px)')
      ax.set_zlabel('P(Measured Distance | Ground Truth)')
      plt.show()

  def motion_model(self, proposal_dist, action, sigma=(0.1, 0.1, 0.01)):
    """
    Applies the action and some random perturbation to the proposal distribution.
    proposal_dist: (Nx3 np.ndarray) The proposal distribution.
    action: (length 3 np.ndarray) The difference in odometry measurements.

    >>> proposal_dist = np.array([[0,0,0],
    ...                          [0,0,0],
    ...                          [0,0,0]])
    >>> action = np.array([1,1,1])
    >>> motion_model(proposal_dist, action)
    """
    # Add the odometry and then gaussian noise.
    np.add(proposal_dist, action, proposal_dist)
    return np.add(proposal_dist, np.random.normal(0, sigma, proposal_dist.shape).astype(np.float32))

  def probability_of_range(self, measured_range, expected_range):
    """
    Return the probability of observing measured_range given expected_range (groundtruth).

    measured_range: (float) The measured range value in meters.
    expected_range: (float) The expected range value in meters.
    """

    # Factorization of the sensor model distribution.
    probability_if_known_obstacle = self.METERS_PER_PX \
                                  * (1./ (self.GAUSSIAN_STD_DEV * math.sqrt(2*math.pi))) \
                                  * math.exp(-(measured_range - expected_range)**2 / (2 * self.GAUSSIAN_STD_DEV**2))
    probability_if_short_measurement = 0.0 if expected_range == 0 else self.METERS_PER_PX * max(0., -2. * measured_range / expected_range**2 + 2. / expected_range)
    probability_if_missed_measurement = 1. if measured_range >= (self.MAX_RANGE_METERS - 1e-2) else 0.
    probability_if_random_measurement = self.METERS_PER_PX * 5. / self.MAX_RANGE_METERS

    final_probability = self.PROBABILITY_OF_KNOWN_OBSTACLE * probability_if_known_obstacle \
                      + self.PROBABILITY_OF_SHORT_MEASUREMENT * probability_if_short_measurement \
                      + self.PROBABILITY_OF_MISSED_MEASUREMENT * probability_if_missed_measurement \
                      + self.PROBABILITY_OF_RANDOM_MEASUREMENT * probability_if_random_measurement
    return final_probability

  def sensor_model(self, proposal_dist, obs, weights):
    """
    Given the proposal distribution represented implicitly by a set of particles and weights,
    returns the new weights after applying the sensor model.

    proposal_dist: (Nx3 np.ndarray) The current particle poses (after updating with the motion model).
    obs: (self.NUM_RAYS length np.ndarray) An array of measured ranges from the scan.
    weights: (N length np.ndarray) The weights associated with each particle in proposal_dist.
    """
    self.new_laser_angles = np.array(self.laser_angles[::self.SUBSAMPLING_BIN_SIZE])
    self.NEW_NUM_RAYS = len(self.new_laser_angles)
    new_obs = np.array(obs[::self.SUBSAMPLING_BIN_SIZE])
    # On the first sensor update, allocate memory for all of the large arrays we need.
    if self.first_sensor_update:
      self.queries = np.zeros((self.NEW_NUM_RAYS * self.MAX_PARTICLES, 3), dtype=np.float32)
      self.ground_ranges_1d = np.zeros(self.NEW_NUM_RAYS * self.MAX_PARTICLES, dtype=np.float32)
      self.first_sensor_update = False

    # Angles goes from angle_min to angle_max. The resolution is determined by
    # the angle increment in the scans.
    self.range_method.calc_range_repeat_angles(proposal_dist, self.new_laser_angles, self.ground_ranges_1d)

    # Update weights using the precomputed sensor model.
    self.range_method.eval_sensor_model(new_obs, self.ground_ranges_1d, weights, self.NEW_NUM_RAYS, self.MAX_PARTICLES)

    return weights

  def MCL(self, action, observations):
    """
    Runs the update and reweighting steps using the motion model and sensor model.

    action: (?) The differential odometry measurement between the previous and current timestep.
    observations: (np.ndarray) The current laser scan data.

    Note: this function does not return anything - it stores updated particles and weights
    in self.particles and self.weights, respectively.
    """
    #time1 = time.time()
    # Sample poses from the current distribution.
    samples = np.random.choice(self.particle_indices, size=self.particles.shape[0], p=self.weights)

    X_tm1 = self.particles[samples, :] # Samples from
    #time2 = time.time()
    # Feedforward motion model to update the particles from previous timestep.
    self.particles = self.motion_model(X_tm1, action, self.MOTION_MODEL_SIGMA)
    #time3 = time.time()
    self.weights = self.sensor_model(self.particles, observations, self.weights)
    
    wsum = np.sum(self.weights)
    if wsum == 0:
      rospy.logwarn('The particle weight sum is numerically close to zero: %f.' % wsum)

    self.weights /= wsum

  def _expected_pose(self):
    """
    Computes the expected pose over self.particles (the current set of particles).
    """
    return self.particles[np.argmax(self.weights)]

  def expected_pose(self):
    x_avg = np.average(self.particles[:,0], axis=0, weights=self.weights)
    y_avg = np.average(self.particles[:,1], axis=0, weights=self.weights)

    # Theta must be averaged in cartesian space (wraparound issues).
    theta = self.particles[:,2]
    cartesian = np.vstack([np.cos(theta), np.sin(theta)])
    cartesian_avg = np.average(cartesian, axis=1, weights=self.weights)
    theta_avg = math.atan2(cartesian_avg[1], cartesian_avg[0])
    return np.array([x_avg, y_avg, theta_avg])

  def update(self):
    """
    Safely runs the MCL update. Ensures that the lidar, odometry, and map are
    initialized so that MCL will behave well.
    """
    if self.lidar_initialized and self.odom_initialized and self.map_initialized:

      # Make sure to consider concurrency related issues!
      if self.state_lock.locked():
        rospy.loginfo('Blocked update to avoid concurrency error.')
      else:
        self.state_lock.acquire()

        # Run MCL update on the latest odometry and lidar data.
        self.MCL(self.movement, self.latest_scan_data)

        self.inferred_pose = self.expected_pose()
        self.state_lock.release()
        self.publish_tf(self.inferred_pose, self.rel_odom_pose.header.stamp)
        self.visualize()

if __name__=="__main__":
  rospy.init_node("lab5_localization")
  pf = ParticleFilter()
  rospy.spin()
