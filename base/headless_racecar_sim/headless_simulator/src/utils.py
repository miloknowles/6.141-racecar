import rospy
import numpy as np
from yaml import load
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid
import csv
import matplotlib.pyplot as plt
import scipy.ndimage
import skimage.morphology
import json, time, collections, recordclass

State = collections.namedtuple('State', ['x', 'y', 'theta'])
ExtendedState = collections.namedtuple('State', ['x', 'y', 'theta', 'throttle']) # throttle: 1 - forwards, -1: backwards
Action = collections.namedtuple('Action', ['steering_angle', 'throttle'])

import tf.transformations
import tf

import skimage.morphology
from scipy import ndimage
import ujson

EPSILON = 0.00000000001

# converts from the given laser pose to the point between the rear tires
# offset is the distance between the laser and rear tires
def laser_to_rear_wheels(pose, offset=0.295):
    return pose - (offset*np.cos(pose[2]), offset*np.sin(pose[2]), 0.0)

# converts from the given laser pose to the point between the rear tires
# offset is the distance between the laser and rear tires
def rear_wheels_to_laser(pose, offset=0.295):
    return pose + (offset*np.cos(pose[2]), offset*np.sin(pose[2]), 0.0)

class Map(object):
    """ Convenience wrapper for an occupancy grid map object.
        Provides methods to:
            - check distances to nearest objects
            - check if points are permissible
            - mark map regions as explored
            - dilate the map
    """
    def __init__(self, map_msg):
        self.map_msg = map_msg
        self.map_info = map_msg.info

        self.cspace = None
        # 0: permissible, -1: unmapped, 100: blocked
        self.raw_matrix = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # reversed from expectation since this is what distance_transform_edt requires
        self.occupancy_grid = np.ones_like(self.raw_matrix, dtype=bool)
        self.occupancy_grid[self.raw_matrix>50] = 0

        # cache this stuff for a fast world_to_map function
        self.angle = quaternion_to_angle(self.map_info.origin.orientation)
        self.rot = rotation_matrix(-self.angle)
        self.trans = np.array([[self.map_info.origin.position.x],
                          [self.map_info.origin.position.y]])
        self.scale = float(self.map_info.resolution)

    def world_to_map(self, pose):
        world = pose[:2].reshape((2,1))
        map_c = self.rot*((world - self.trans) / self.scale)
        return map_c[0,0],map_c[1,0],pose[2]-self.angle

    def cell_size(self):
        return self.map_info.resolution

    def make_cspace(self, kernels):
        print "Making configuration space...", (self.occupancy_grid.shape[0], self.occupancy_grid.shape[1], len(kernels))
        self.cspace = np.zeros((self.occupancy_grid.shape[0], self.occupancy_grid.shape[1], len(kernels)), dtype=bool)
        
        og = np.zeros_like(self.occupancy_grid, dtype=float)
        omap_slice = np.zeros_like(self.occupancy_grid, dtype=bool)
        for i in xrange(len(kernels)):
            og.fill(0.0)
            og[self.occupancy_grid==0] = 255
            dilated = skimage.morphology.dilation(og, selem=kernels[i])

            omap_slice.fill(0)
            omap_slice[dilated > 0] = 255
            self.cspace[:,:,i] = omap_slice

    # checks for a collision at the given pose by indexing the configuration space at the nearest slice
    def check_collision(self, pose):

        # map_pose = world_to_map_slow(pose[0], pose[1], pose[2], self.map_info)
        map_pose = self.world_to_map(pose)

        indices = np.zeros(3, dtype=int)
        indices[0] = map_pose[1]
        indices[1] = map_pose[0]
        indices[2] = np.round(-1.0*self.cspace.shape[2]*pose[2]/(np.pi*2.0)) % self.cspace.shape[2]

        if np.any(indices < 0): return False
        if np.any(indices >= self.cspace.shape): return False
        return self.cspace[indices[0], indices[1], indices[2]]

        # print discrete_theta

        # plt.imshow(self.cspace[:,:,0], cmap="gray")
        # plt.show()

        # print kernels
        pass

    # def get_permissible(self, queries, check_bounds=False, coord_convert=True):
    #     ''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
    #     '''
    #     if coord_convert:
    #         q = queries.copy()
    #         world_to_map(q, self.map_info)
    #         q = np.round(q[:,:2]).astype(int)
    #     else:
    #         q = queries.astype(int)
        
    #     if check_bounds:
    #         bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
    #                        np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
    #                        np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
    #         q[bad,:] = 0

    #     distances = self.permissible_region[q[:,1], q[:,0]]
    #     if check_bounds:
    #         distances[bad] = np.nan
    #     return distances

    # def get_distances(self, queries, check_bounds=False, coord_convert=True):
    #     ''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
    #     '''
    #     if coord_convert:
    #         q = queries.copy()
    #         world_to_map(q, self.map_info)
    #         q = np.round(q[:,:2]).astype(int)
    #     else:
    #         q = queries.astype(int)

    #     if check_bounds:
    #         bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
    #                        np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
    #                        np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
    #         q[bad,:] = 0

    #     distances = self.distmap[q[:,1], q[:,0]] * self.map_info.resolution
    #     if check_bounds:
    #         distances[bad] = np.nan
    #     return distances

    # def dilate(self, radius):
    #     og = np.zeros_like(self.occupancy_grid, dtype=float)
    #     og[self.occupancy_grid==0] = 255
    #     el = skimage.morphology.disk(10)
    #     return skimage.morphology.dilation(og, selem=el)

def load_params_from_yaml(fp):
    with open(fp, 'r') as infile:
        yaml_data = load(infile)
        for param in yaml_data:
            print "param:", param, ":", yaml_data[param]
            rospy.set_param("~"+param, yaml_data[param])

class CircularArray(object):
    """docstring for CircularArray"""
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])

class Timer:
    def __init__(self, smoothing):
        self.arr = CircularArray(smoothing)
        self.last_time = time.time()

    def tick(self):
        t = time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    def fps(self):
        return self.arr.mean()

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw

def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])

def particle_to_pose(particle):
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = angle_to_quaternion(particle[2])
    return pose

def particles_to_poses(particles):
    return map(particle_to_pose, particles)

def make_header(frame_id, stamp=None):
    if stamp == None:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header

def make_circle_marker(point, scale, color, frame_id, namespace, sid, duration=0):
    marker = Marker()
    marker.header = make_header(frame_id)
    marker.ns = namespace
    marker.id = sid
    marker.type = 2 # sphere
    marker.lifetime = rospy.Duration.from_sec(duration)
    marker.action = 0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = 1.0
    return marker

def point(npt):
    pt = Point32()
    pt.x = npt[0]
    pt.y = npt[1]
    return pt

def points(arr):
    return map(point, arr)

# converts map space coordinates to world space coordinates
def map_to_world_slow(x,y,t,map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    rot = rotation_matrix(angle)
    trans = np.array([[map_info.origin.position.x],
                      [map_info.origin.position.y]])

    map_c = np.array([[x],
                      [y]])
    world = (rot*map_c) * scale + trans

    return world[0,0],world[1,0],t+angle

def map_to_world(poses,map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)

    # rotate

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:,0])
    poses[:,0] = c*poses[:,0] - s*poses[:,1]
    poses[:,1] = s*temp       + c*poses[:,1]

    # scale
    poses[:,:2] *= float(scale)

    # translate
    poses[:,0] += map_info.origin.position.x
    poses[:,1] += map_info.origin.position.y
    poses[:,2] += angle

def world_to_map(poses, map_info):
    # equivalent to map_to_grid(world_to_map(poses))
    # operates in place
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)

    # translation
    poses[:,0] -= map_info.origin.position.x
    poses[:,1] -= map_info.origin.position.y

    # scale
    poses[:,:2] *= (1.0/float(scale))

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:,0])
    poses[:,0] = c*poses[:,0] - s*poses[:,1]
    poses[:,1] = s*temp       + c*poses[:,1]
    if poses.shape[1] > 2:
        poses[:,2] += angle

# converts world space coordinates to map space coordinates
def world_to_map_slow(x,y,t, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    rot = rotation_matrix(-angle)
    trans = np.array([[map_info.origin.position.x],
                      [map_info.origin.position.y]])

    world = np.array([[x],
                      [y]])
    map_c = rot*((world - trans) / float(scale))
    return map_c[0,0],map_c[1,0],t-angle

# coords: Nx2 in polar (r,theta)
# in place modifies to Nx2 (x,y)
def polar_to_euclid(coords):
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    return (xs, ys)

def angular_deflection_magnitude(points):
    # https://mail.python.org/pipermail/tutor/2007-July/055178.html
    # returns a numpy array of angular deflections between consequtive 
    # line segments beginning and ending at the points provided
    # contains two less angles than points, since the angular deflection for the first and last components is ill defined

    lines = np.zeros((points.shape[0]-1, 3))
    thetas = np.zeros(points.shape[0]-2)
    for i in xrange(1,points.shape[0]):
        p0 = points[i-1,:]
        p1 = points[i,:]

        A = p0[1] - p1[1]
        B = p1[0] - p0[0]
        C = p0[0]*p1[1] - p1[0]*p0[1]
        lines[i-1] = (A,B,C)

    for i in xrange(1, lines.shape[0]):
        A1 = lines[i-1,0]
        B1 = lines[i-1,1]
        A2 = lines[i,0]
        B2 = lines[i,1]
        bottom = (A1**2+B1**2)*(A2**2+B2**2)
        if bottom > 0:
            inner = (A1*A2 + B1*B2) / np.sqrt(bottom)
            # thetas[i-1] = np.arccos(inner)
            if np.abs(np.abs(inner) - 1.0) < EPSILON:
                thetas[i-1] = 0.0
            else:
                thetas[i-1] = np.arccos(inner)
    return thetas

class AckermannModel(object):
    """ A wrapper class for useful Ackermann steering geometry related functions
    """
    def __init__(self, wheelbase):
        self.L = wheelbase

    def path_radius(self, steering_angle):
        ''' The radius of the path driven if a constant steering angle is applied
        '''
        return self.L / np.tan(steering_angle)

    def yaw_rate(self, steering_angle, speed):
        ''' Rate of change of heading with a given steering angle and speed
        '''
        if steering_angle == 0.0:
            return 0.0
        return speed / self.path_radius(steering_angle)

    def dx(self, speed, dt, steering_angle):
        ''' Distance traveled in the local x direction given speed and steering_angle
        '''
        if steering_angle == 0.0:
            return speed * dt
        R = self.path_radius(steering_angle)
        d = dt*speed
        dx = R*np.sin(d/R)
        return dx

    def dy(self, speed, dt, steering_angle):
        ''' Distance traveled in the local y direction given speed and steering_angle
        '''
        if steering_angle == 0.0:
            return 0.0
        R = self.path_radius(steering_angle)
        d = dt*speed
        dy = R*(1.0 - np.cos(d/R))
        return dy

    def steering_angle(self, point):
        ''' Returns the steering angle required to pass through the given point
            (in local euclidean coordinates) assuming constant steering angle is applied
        '''
        if point[0] >= 0.0:
            theta = np.arctan(point[1]/point[0])
        else:
            theta = np.arctan(abs(point[0])/point[1]) + np.sign(point[1])*np.pi/2.0

        return np.arctan(2.0*self.L*np.sin(theta)/np.linalg.norm(point))

    def steering_angle_polar(self, polar_point):
        ''' Returns the steering angle required to pass through the given point
            (in local polar coordinates) assuming constant steering angle is applied
        '''
        theta = polar_point[1]
        radius = polar_point[0]
        return np.arctan(2.0*self.L*np.sin(theta)/radius)

def max_angle(min_turning_radius, radius):
    tr2 = 2.0*min_turning_radius
    if radius < tr2:
        r2 = radius*radius
        y = r2 / (2.0*min_turning_radius)
        x = np.sqrt(r2 - y*y)
        max_angle = np.arctan(y/x)
    else:
        max_angle = np.pi / 2.0
    return max_angle

def test_max_angle():
    print max_angle(1.0,2.0)
    print max_angle(1.0,1.999)
    print max_angle(1.0,1.5)
    print max_angle(1.0,1.0)

