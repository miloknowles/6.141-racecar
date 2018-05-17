#!/usr/bin/env python

import rospy
import numpy as np

import range_libc
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from std_msgs.msg import Float64, String
from sensor_msgs.msg import LaserScan, Imu
import tf.transformations
import tf
import matplotlib.pyplot as plt
import time
import utils
import Image, ImageDraw
import traceback

from threading import Lock

EPSILON = 0.000000001

last_print = time.time()
def rate_limited_print(str, rate=5.0):
	global last_print
	if time.time() - last_print > 1.0 / rate:
		print str
		last_print = time.time()

class MotionModel(object):
	""" Base class for describing car dynamics """
	def __init__(self):
		pass
	
	def update(self, action, state, step):
		raise NotImplementedError("Must be defined by child class")

class EmpericalModel(MotionModel):
	""" Emperical model based off measured car data, valid only for low speeds """
	def __init__(self):
		self.max_steer = 0.335
		self.L = 0.23
		self.ackerman_transition = [0.145, 0.155]
		self.polynomial_coeffs = [-4.035, 5.153, -0.018]
		self.poly = lambda a: 1.0 / (a*a*self.polynomial_coeffs[0] + a*self.polynomial_coeffs[1] + self.polynomial_coeffs[2])

		self._EPSILON = 0.0000001

		self.MAX_CENTRIPETAL_ACCEL = rospy.get_param("~max_centripetal_accel")
		self.MAX_LINEAR_ACCEL = rospy.get_param("~max_linear_accel")

	# only accepts positive, nonzero steering angles
	def steering_arc_radius(self, angle):
		arc_radius = 0.0
		if angle <= self.ackerman_transition[0]:
			arc_radius = self.L / np.tan(angle)
		elif angle >= self.ackerman_transition[1]:
			arc_radius = self.poly(min(angle, self.max_steer))
		else:
			width = self.ackerman_transition[1] - self.ackerman_transition[0]
			t = (angle - self.ackerman_transition[0]) / width
			ackL  = self.L / np.tan(angle)
			polyL = self.poly(angle)
			arc_radius = (1.0 - t) * ackL + t * polyL

		if arc_radius == 0.0:
			print "THIS SHOULD NOT HAPPEN"
		return arc_radius

	# updates "state" in place
	def update(self, state, dtime):
		accel_bound = dtime * self.MAX_LINEAR_ACCEL

		# print "test"
		steering_angle = state[3]
		throttle = state[4]
		theta = state[2]
		velocity = state[5]

		# update velocity with bounded acceleration
		desired_accel = throttle - velocity
		bounded_accel = np.clip(desired_accel, -accel_bound, accel_bound)
		state[5] = velocity + bounded_accel

		if abs(velocity) < self._EPSILON:
			return

		if abs(steering_angle) < self._EPSILON:
			dt = 0
			dy = 0
			dx = velocity * dtime
		else:
			sign = np.sign(steering_angle)
			R = self.steering_arc_radius(abs(steering_angle))
			dt = sign * velocity * dtime / R
			dy = sign * (1.0-np.cos(dt))*R
			dx = sign * np.sin(dt)*R

		# convert to global coordinate space deltas
		c, s = np.cos(theta), np.sin(theta)
		state[0] += c*dx - s*dy
		state[1] += s*dx + c*dy
		state[2] += dt
		state[2] = state[2] % (2.0*np.pi)
		return state

class AckermannModel(MotionModel):
	""" Standard kinematic bicycle motion model """
	def __init__(self):
		self._EPSILON = 0.0000001
		self.max_steer = 0.335
		self.L = 0.23

		self.MAX_CENTRIPETAL_ACCEL = rospy.get_param("~max_centripetal_accel")
		self.MAX_LINEAR_ACCEL = rospy.get_param("~max_linear_accel")


	# throttle controls acceleration (basically), velocity undergoes damping

	def update(self, state, dtime):
		self.MAX_LINEAR_ACCEL = 2.5 # units is meters/second squared
		accel_bound = dtime * self.MAX_LINEAR_ACCEL

		steering_angle = state[3]
		throttle = state[4]
		theta = state[2]
		velocity = state[5]

		# update velocity with bounded acceleration
		desired_accel = throttle - velocity
		bounded_accel = np.clip(desired_accel, -accel_bound, accel_bound)
		state[5] = velocity + bounded_accel

		if abs(velocity) < self._EPSILON:
			return

		if abs(steering_angle) < self._EPSILON:
			dt = 0
			dy = 0
			dx = velocity * dtime
		else:
			tanangle = np.tan(steering_angle)

			# moderate hack: increase self.L until centripetal_acceleration is within bounds
			centripetal_acceleration = tanangle * velocity*velocity / self.L
			if abs(centripetal_acceleration) > self.MAX_CENTRIPETAL_ACCEL:
				# find L s.t. centripetal_acceleration == self.MAX_CENTRIPETAL_ACCEL
				effective_l = abs(tanangle * velocity*velocity / self.MAX_CENTRIPETAL_ACCEL)
			else:
				effective_l = self.L

			dt = velocity * dtime * tanangle / effective_l
			dy = (1.0-np.cos(dt))*effective_l / tanangle
			dx = effective_l * np.sin(dt) / tanangle

		# convert to global coordinate space deltas
		c, s = np.cos(theta), np.sin(theta)

		old_state = state.copy()

		state[0] += c*dx - s*dy
		state[1] += s*dx + c*dy
		state[2] += dt
		state[2] = state[2] % (2.0*np.pi)

		dist = np.linalg.norm(old_state[:2] - state[:2])
		if dist > 0.5:
			print "Moved very far, likely an error:", dist, dtime
			traceback.print_stack()

		return state

class RACECAR(object):
	""" A high level wrapper object for maintaining RACECAR state, which includes:
		- position (x,y,theta)
		- forwards velocity (v)
		- steering angle (s)
		- throttle (t)
	"""
	def __init__(self, motion_model=None):
		self.simulation_rate = rospy.get_param("~simulation_rate")
		start_state = map(float, rospy.get_param("~start_state").split(","))

		print "Starting at state:", start_state

		# origin is the back right wheel of the car if the car is pointing along the positive x axis
		# 0.3 meters wide, 0.55 meters long
		# self.dimensions = np.array([0.55, 0.3])
		self.dimensions = np.array([0.5, 0.25])
		# center of lidar, centered side to side and 0.14 meters from front
		self.lidar_position = np.array([0.41, 0.15])
		# point between the rear tires, used as the state in value iteration
		self.base_frame = np.array([0.11, 0.15])

		# x, y, theta, steering angle, throttle, velocity
		self.state = np.zeros(6)
		self.state[:3] = start_state
		self.iter = 0
		self.last_update = 0.0
		self.lock = Lock()

		self.derivatives = None
		self.local_linear_acceleration = None

		if motion_model == None:
			print "ERROR: Must provide a motion model to the RACECAR simulator"
		self.motion_model = motion_model

		self.simulation_timer = utils.Timer(20)

	def set_state(self, x=None, y=None, theta=None, steer=None, throttle=None, velocity=None, raw=None):
		if not raw == None:
			self.state = raw

		if not x == None: self.state[0] = x
		if not y == None: self.state[1] = y
		if not theta == None: self.state[2] = theta
		if not steer == None: self.state[3] = steer
		if not throttle == None: self.state[4] = throttle
		if not velocity == None: self.state[5] = velocity

	def pause(self):
		pass

	def unpause(self):
		self.last_update = rospy.get_time()

	def simulate(self, evt, omap=None):
		with self.lock:
			self.iter += 1
			self.simulation_timer.tick()

			now = rospy.get_time()
			dt = now - self.last_update
			self.last_update = now

			prior_state = self.state.copy()

			# do not simulate if the dt value is very small or very large
			if dt > 0.00001 and dt < 4.0 / self.simulation_rate: 
				self.motion_model.update(self.state, dt)

				if omap:
					is_colliding = omap.check_collision(self.state)
					if is_colliding:
						rate_limited_print("Preventing motion, would cause collision")
						# print "Preventing motion, would cause collision"
						self.state = prior_state
						self.state[5] = 0.0 # zero out the velocity

				# numerically compute velocity
				deltas = self.state - prior_state
				derivatives = deltas / dt

				if isinstance(self.derivatives, np.ndarray):
					self.accelerations = (derivatives - self.derivatives) / dt
					# print "ddx:", self.accelerations[0], self.accelerations[1]

					# need to convert this into the coordinate frame of the robot
					costheta = np.cos(-self.state[2])
					sintheta = np.sin(-self.state[2])
					local_ddx_dtt = self.accelerations[0]*costheta-self.accelerations[1]*sintheta
					local_ddy_dtt = self.accelerations[0]*sintheta+self.accelerations[1]*costheta
					self.local_linear_acceleration = np.array([local_ddx_dtt, local_ddy_dtt])
					# print local_ddx_dtt, local_ddy_dtt

				self.deltas = deltas
				self.derivatives = derivatives

	# theta is the heading of the car, center is the point on the 
	# car which should be at the center of the kernel, if none it
	# is set to the base_frame of the car. cell size is the width/height
	# of one pixel in the output kernel. Padding is how much space should be
	# added to the true dimensions of the car for safety margin (in meters)
	def footprint_kernel(self, theta=0, cell_size=0.01, padding=0.0, center=None):
		dims = self.dimensions / cell_size
		padding = padding / cell_size

		if center == None:
			center = self.base_frame / cell_size

		corners = np.array([[0,0], [dims[0]+padding,0], dims+padding, [0,dims[1]+padding], [0,0]], dtype=np.float64)
		corners -= center
		corners -= padding / 2.0

		# rotation matrix
		R = np.array([[np.cos(theta), -np.sin(theta)],
                  	 [np.sin(theta), np.cos(theta)]])

		corners = np.dot(R, corners.T).T
		bounds = np.ceil(np.max(np.abs(corners), axis=0)).astype(int)
		kernel = np.zeros(bounds[::-1]*2)
		corners = corners + bounds
		
		# draw car footprint onto kernel
		img = Image.fromarray(kernel)
		draw = ImageDraw.Draw(img)
		draw.polygon([tuple(p) for p in corners], fill=1)
		kernel = np.asarray(img)
		return kernel

class Laser(object):
	"""docstring for Laser"""
	def __init__(self, num_rays=None, min_angle=None, max_angle=None, max_range=None, range_method=None, frame=None, omap=None):
		self.max_range = max_range

		self.laser_ranges = np.zeros(num_rays, dtype=np.float32)
		self.laser_angles = np.linspace(min_angle, max_angle, num_rays, endpoint=True)
		self.queries = np.zeros((num_rays, 3), dtype=np.float32)

		map_msg = omap.map_msg
		map_info = map_msg.info
		oMap = range_libc.PyOMap(map_msg)
		self.MAX_RANGE_PX = int(self.max_range / map_info.resolution)
		self.THETA_DISCRETIZATION = 200

		# initialize range method
		print "Initializing range method:", range_method
		if range_method == "bl":
			self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
		elif "cddt" in range_method:
			self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
			if range_method == "pcddt":
				print "Pruning..."
				self.range_method.prune()
		elif range_method == "rm":
			self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
		elif range_method == "rmgpu":
			self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
		elif range_method == "glt":
			self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
		print "Simulated Laser Scanner Initialized..."

	def simulate(self, state):
		self.queries[:,0] = state[0]
		self.queries[:,1] = state[1]
		self.queries[:,2] = self.laser_angles + state[2]
		self.range_method.calc_range_many(self.queries, self.laser_ranges)

class Simulator(object):
	def __init__(self):
		# simulation parameters
		self.simulation_rate = rospy.get_param("~simulation_rate")
		self.motion_model = rospy.get_param("~motion_model")
		self.detect_collision = rospy.get_param("~detect_collision")
		self.actuate_model = rospy.get_param("~actuate_model")
		self.listen_tools = rospy.get_param("~listen_tools")
		self.simulate_laser = rospy.get_param("~simulate_laser")
		self.simulate_imu = rospy.get_param("~simulate_imu")

		# odometer parameters
		self.simulate_odom = rospy.get_param("~simulate_odom")
		if self.simulate_odom:
			self.odom_noise = rospy.get_param("~odom_noise")
			self.odom_rate = rospy.get_param("~odom_rate")
			self.odom_topic = rospy.get_param("~odom_topic")
			self.odom_frame = rospy.get_param("~odom_frame")

			self.odom_msg = Odometry()
			self.odom_msg.header = utils.make_header(self.odom_frame, None)
			self.odom_timer = utils.Timer(20)

		if self.simulate_imu:
			self.imu_rate = rospy.get_param("~imu_rate")
			self.imu_topic = rospy.get_param("~imu_topic")
			self.imu_frame = rospy.get_param("~imu_frame")

			self.imu_msg = Imu()
			self.imu_msg.header = utils.make_header(self.imu_frame, None)

			for i in xrange(len(self.imu_msg.orientation_covariance)):
				self.imu_msg.orientation_covariance[i] = -1

			self.imu_timer = utils.Timer(20)

		self.map_initialized = False
		self.omap = None
		if self.detect_collision or self.simulate_laser:
			self.get_omap()

		# laser parameters
		self.last_pose = None
		
		if self.simulate_laser: 
			self.laser_rays = rospy.get_param("~laser_rays")
			self.laser_topic = rospy.get_param("~laser_topic")
			self.laser_min_angle = rospy.get_param("~laser_min_angle")
			self.laser_max_angle = rospy.get_param("~laser_max_angle")
			self.laser_max_range = rospy.get_param("~laser_max_range")
			self.laser_range_method = rospy.get_param("~laser_range_method")
			self.laser_frame = rospy.get_param("~laser_frame")
			self.laser_rate = rospy.get_param("~laser_rate")

			self.laser_angle_increment = (self.laser_max_angle - self.laser_min_angle) / float(self.laser_rays)

			self.laser = Laser(num_rays=self.laser_rays, 
				min_angle=self.laser_min_angle, 
				max_angle=self.laser_max_angle,
				max_range=self.laser_max_range,
				frame=self.laser_frame,
				range_method=self.laser_range_method,
				omap=self.omap)

			self.scan_msg = LaserScan()
			self.scan_msg.header = utils.make_header(self.laser_frame, stamp=None)
			self.scan_msg.angle_min = self.laser_min_angle
			self.scan_msg.angle_max = self.laser_max_angle
			self.scan_msg.angle_increment = self.laser_angle_increment
			self.scan_msg.range_min = 0
			self.scan_msg.range_max = self.laser_max_range
			self.laser_timer = utils.Timer(20)

		# transformation frame parameters
		self.publish_tf = rospy.get_param("~publish_tf")
		if self.publish_tf:
			self.tf_rate = rospy.get_param("~tf_rate")
			self.tf_to   = rospy.get_param("~tf_to")
			self.tf_from = rospy.get_param("~tf_from")
			self.tf_timer = utils.Timer(20)

		# diagnostics
		self.diagnostic_rate = rospy.get_param("~diagnostic_rate")

		# communication topics
		self.motor_topic = rospy.get_param("~motor_topic")
		self.servo_topic = rospy.get_param("~servo_topic")

		# conversion constants
		self.ms_to_erpm = rospy.get_param("/vesc/speed_to_erpm_gain")
		self.angle_to_servo = rospy.get_param("/vesc/steering_angle_to_servo_gain")
		self.angle_to_servo_offset = rospy.get_param("/vesc/steering_angle_to_servo_offset")

		# generate motion model
		motion_model = None
		if self.motion_model.lower() == "ackermann":
			motion_model = AckermannModel()
		elif self.motion_model.lower() == "empirical":
			motion_model = EmpericalModel()
		
		self.sim_timer = utils.Timer(20)
		self.car = RACECAR(motion_model=motion_model)

		# attributes related to RViz manipulation tools
		self.drag_active = False
		self.rotate_active = False
		self.place_active = False
		if self.listen_tools:
			self.dragging = False
			self.dragstart = None
			self.drag_start_state = None
			self.rotatestart = None
			self.rotate_start_state = None
			self.rotating = False
			self.placestart = None
			self.place_start_state = None
			self.placing = False

		# make configuration space, necessary for accurate collision detection
		if self.detect_collision:
			self.cspace_thetas = rospy.get_param("~cspace_thetas")

			print self.car.footprint_kernel(theta=0, cell_size=self.omap.cell_size())

			kernels = map(lambda a: self.car.footprint_kernel(theta=-a, cell_size=self.omap.cell_size()), 
				np.linspace(0.0, np.pi*2.0, self.cspace_thetas, endpoint=False))
			self.omap.make_cspace(kernels)

		# make publishers
		if self.simulate_odom: 
			self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size = 1)

		if self.simulate_laser: 
			self.laser_pub = rospy.Publisher(self.laser_topic, LaserScan, queue_size = 1)

		if self.simulate_imu: 
			self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size = 3)

		if self.publish_tf or self.actuate_model: 
			self.tf_pub = tf.TransformBroadcaster()

		# subscribe to topics
		self.motor_sub = rospy.Subscriber(self.motor_topic,  Float64, self._motor_callback, queue_size=1)
		self.servo_sub = rospy.Subscriber(self.servo_topic,  Float64, self._servo_callback, queue_size=1)

		if self.listen_tools:
			self.drag_sub = rospy.Subscriber("/headless_sim_rviz/drag",  String, self._drag_callback, queue_size=10)
			self.rotate_sub = rospy.Subscriber("/headless_sim_rviz/rotate",  String, self._rotate_callback, queue_size=10)
			self.place_sub = rospy.Subscriber("/headless_sim_rviz/place",  String, self._place_callback, queue_size=10)

		# periodically perform simulation
		rospy.Timer(rospy.Duration(1.0 / self.simulation_rate), self._simulate)

		if self.simulate_odom:
			print "Simulating Odometry..."
			rospy.Timer(rospy.Duration(1.0 / self.odom_rate), self._simulate_odom)

		if self.simulate_laser:
			print "Simulating Laser..."
			rospy.Timer(rospy.Duration(1.0 / self.laser_rate), self._simulate_laser)

		if self.simulate_imu:
			print "Simulating imu..."
			rospy.Timer(rospy.Duration(1.0 / self.imu_rate), self._simulate_imu)

		if self.publish_tf:
			print "Publishing TF..."
			rospy.Timer(rospy.Duration(1.0 / self.tf_rate), self._publish_tf)

		if self.diagnostic_rate > 0:
			print "Publishing Diagnostics..."
			rospy.Timer(rospy.Duration(1.0 / self.diagnostic_rate), self._diagnostic)

	def _diagnostic(self, evt):
		diag_str = ""
		if self.simulate_odom: diag_str += "Odom rate: %.1f, " % self.odom_timer.fps()
		if self.simulate_laser: diag_str += "Laser rate: %.1f, " % self.laser_timer.fps()
		if self.simulate_imu: diag_str += "IMU rate: %.1f, " % self.imu_timer.fps()
		if self.publish_tf: diag_str += "Publish TF rate: %.1f, " % self.tf_timer.fps()
		diag_str += "Simulation rate: %.1f, " % self.sim_timer.fps()
		print diag_str[:-2]
		print "State: x: %.2f, y: %.2f, theta: %.2f, steering angle: %.2f, throttle: %.2f" % (self.car.state[0],self.car.state[1],self.car.state[2],self.car.state[3],self.car.state[4])

	def get_omap(self):
		map_service_name = rospy.get_param("~static_map", "static_map")
		print "Fetching map from service: ", map_service_name
		rospy.wait_for_service(map_service_name)
		map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
		self.omap = utils.Map(map_msg)
		self.map_initialized = True
		print "Finished loading map"

	def _should_simulate(self):
		return not self.drag_active and not self.rotate_active and not self.place_active

	def _simulate(self, evt):
		if self._should_simulate():

			if self.detect_collision:
				self.car.simulate(evt, self.omap)
			else:
				self.car.simulate(evt)

			# self.car.simulate(evt)
			if self.detect_collision:
				is_colliding = self.omap.check_collision(self.car.state)
				if is_colliding:
					print "Wall collision...", rospy.get_time()
		self.sim_timer.tick()

	def _drag_callback(self, msg):
		dat = msg.data
		if "dragactivate" in dat:
			self.car.lock.acquire()
			self.car.pause()
			self.drag_active = True
			return
		elif "dragdeactivate" in dat and self.drag_active:
			self.drag_active = False
			self.car.unpause()
			self.car.lock.release()
			return

		if not self.drag_active: return
		
		pose = (dat.split('(')[1]).split(")")[0]
		pose = np.array(pose.split(","), dtype=np.float32)

		if "dragstart" in dat:
			dist = np.linalg.norm(self.car.state[:2] - pose)
			if dist < 0.5:
				self.dragging = True
				self.dragstart = pose
				self.drag_start_state = self.car.state.copy()
			print "start", dist
		elif "dragend" in dat:
			if self.dragging:
				self.dragging = False
		elif "dragmove" in dat:
			if self.dragging:
				# diff = pose - self.dragstart
				new_pose = self.drag_start_state.copy()
				new_pose[:2] += pose - self.dragstart
				self.car.set_state(x=new_pose[0], y=new_pose[1])
		# print msg.data

	def _rotate_callback(self, msg):
		dat = msg.data
		if "rotateactivate" in dat:
			self.car.lock.acquire()
			self.car.pause()
			self.rotate_active = True
			return
		elif "rotatedeactivate" in dat and self.rotate_active:
			self.rotate_active = False
			self.car.unpause()
			self.car.lock.release()
			return

		if not self.rotate_active: return
		
		pose = (dat.split('(')[1]).split(")")[0]
		pose = np.array(pose.split(","), dtype=np.float32)

		if "rotatestart" in dat:
			dist = np.linalg.norm(self.car.state[:2] - pose)
			if dist < 2.0:
				self.rotating = True
				self.rotatestart = pose
				self.rotate_start_state = self.car.state.copy()
			print "start", dist
		elif "rotateend" in dat:
			if self.rotating:
				self.rotating = False
		elif "rotatemove" in dat:
			if self.rotating:
				original_vector =  self.rotatestart - self.rotate_start_state[:2]
				original_vector /= np.linalg.norm(original_vector)
				new_vector = pose - self.rotate_start_state[:2]
				new_vector /= np.linalg.norm(new_vector)

				sgn = np.sign(np.cross(new_vector, original_vector))
				angle = sgn*np.arccos(np.dot(original_vector, new_vector))
				self.car.set_state(theta=self.rotate_start_state[2] - angle)

	def _place_callback(self, msg):
		print "PLACE CALLBACK"
		dat = msg.data
		if "placeactivate" in dat:
			self.car.lock.acquire()
			self.car.pause()
			self.place_active = True
			return
		elif "placedeactivate" in dat and self.place_active:
			self.place_active = False
			self.car.unpause()
			self.car.lock.release()
			return

		if not self.place_active: return
		
		pose = (dat.split('(')[1]).split(")")[0]
		pose = np.array(pose.split(","), dtype=np.float32)

		if "placestart" in dat:
			self.placing = True
			self.placestart = pose
			self.place_start_state = self.car.state.copy()
			print "Starting RACECAR place"
		elif "placeend" in dat:
			if self.placing:
				original_vector =  np.array([1, 0])
				new_vector = pose - self.placestart
				new_vector /= np.linalg.norm(new_vector)

				sgn = np.sign(np.cross(new_vector, original_vector))
				angle = sgn*np.arccos(np.dot(original_vector, new_vector))

				self.car.set_state(theta=-angle, x=self.placestart[0], y=self.placestart[1], velocity=0)

				self.placing = False
				print "Placing RACECAR"

	def _motor_callback(self, msg):
		with self.car.lock:
			self.car.set_state(throttle=msg.data/self.ms_to_erpm)

	def _servo_callback(self, msg):
		with self.car.lock:
			steer_angle = (msg.data - self.angle_to_servo_offset)/self.angle_to_servo
			self.car.set_state(steer=steer_angle)
		if self.actuate_model:
			self._set_wheel_angle(steer_angle)

	def _set_wheel_angle(self, angle):
		stamp = rospy.Time.now()
		pose = self.car.state

		# this may cause issues with the TF tree. If so, see the below code.
		self.tf_pub.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(-angle + np.pi/2.0, 0, 0), 
			   stamp , "/left_front_wheel", "/left_steering_hinge")
		self.tf_pub.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(-angle + np.pi/2.0, 0, 0), 
			   stamp , "/right_front_wheel", "/right_steering_hinge")

	def _simulate_odom(self, evt):
		# print "Simulate odometery..."
		pose = self.car.state.copy()
		self.odom_msg.header.stamp = rospy.Time.now()
		self.odom_msg.pose.pose.position.x = pose[0]
		self.odom_msg.pose.pose.position.y = pose[1]
		self.odom_msg.pose.pose.orientation = utils.angle_to_quaternion(pose[2])
		self.odom_pub.publish(self.odom_msg)
		self.odom_timer.tick()

	def _simulate_laser(self, evt):
		pose = self.car.state.copy()
		pose = utils.rear_wheels_to_laser(pose[:3])
		stamp = rospy.Time.now()
		
		if not isinstance(self.last_pose, np.ndarray) or np.linalg.norm(pose[:3] - self.last_pose[:3]) > EPSILON:
			self.laser.simulate(pose)
			self.scan_msg.ranges = self.laser.laser_ranges
			self.last_pose = pose

		self.scan_msg.header.stamp = stamp
		self.laser_pub.publish(self.scan_msg)
		self.laser_timer.tick()

	def _simulate_imu(self, evt):
		# pose = self.car.state.copy()
		# pose = utils.rear_wheels_to_imu(pose[:3])
		stamp = rospy.Time.now()

		# dtheta/dt: angular velocity
		if isinstance(self.car.derivatives, np.ndarray):
			self.imu_msg.angular_velocity.z = self.car.derivatives[2]

		# ddx/ddt and ddy/ddt
		if isinstance(self.car.local_linear_acceleration, np.ndarray):
			self.imu_msg.linear_acceleration.x = self.car.local_linear_acceleration[0]
			self.imu_msg.linear_acceleration.y = self.car.local_linear_acceleration[1]

			# total_accel = np.sqrt(self.imu_msg.linear_acceleration.x * self.imu_msg.linear_acceleration.x + self.imu_msg.linear_acceleration.y * self.imu_msg.linear_acceleration.y)
			# if total_accel > 6.0:
			# 	print "HIGH ACCEL:", total_accel

		self.imu_msg.header.stamp = stamp
		self.imu_pub.publish(self.imu_msg)
		self.imu_timer.tick()
		
	def _publish_tf(self, evt):
		""" Publish a tf for the car. This tells ROS where the car is with respect to the map. """
		
		stamp = rospy.Time.now()
		pose = self.car.state

		# this may cause issues with the TF tree. If so, see the below code.
		self.tf_pub.sendTransform((pose[0],pose[1],0),tf.transformations.quaternion_from_euler(0, 0, pose[2]), 
			   stamp , self.tf_to, self.tf_from)

		self.tf_timer.tick()

		return # below this line is disabled

		"""
		Our particle filter provides estimates for the "laser" frame
		since that is where our laser range estimates are measure from. Thus,
		We want to publish a "map" -> "laser" transform.

		However, the car's position is measured with respect to the "base_link"
		frame (it is the root of the TF tree). Thus, we should actually define
		a "map" -> "base_link" transform as to not break the TF tree.
		"""

		# Get map -> laser transform.
		map_laser_pos = np.array( (pose[0],pose[1],0) )
		map_laser_rotation = np.array( tf.transformations.quaternion_from_euler(0, 0, pose[2]) )
		# Apply laser -> base_link transform to map -> laser transform
		# This gives a map -> base_link transform
		laser_base_link_offset = (0.265, 0, 0)
		map_laser_pos -= np.dot(tf.transformations.quaternion_matrix(tf.transformations.unit_vector(map_laser_rotation))[:3,:3], laser_base_link_offset).T

		# Publish transform
		self.pub_tf.sendTransform(map_laser_pos, map_laser_rotation, stamp , "/base_link", "/map")

import os
def make_flamegraph(filterx=None):
    import flamegraph
    perf_log_path = os.path.join(os.path.dirname(__file__), "/home/racecar/racecar-ws/src/simulator/headless_simulator/tmp/perf2.log")
    flamegraph.start_profile_thread(fd=open(perf_log_path, "w"),
                                    filter=filterx,
                                    interval=0.001)


if __name__ == '__main__':
	rospy.init_node("headless_simulator")
	# make_flamegraph(r"_simulate|_publish_tf|_callback")
	sim = Simulator()
	rospy.spin()
