import math
import utils
import json
import numpy as np

def norm(v):
  return math.sqrt(v[0]**2 + v[1]**2)

def distance(p1, p2):
  return norm((p2[0]-p1[0], p2[1]-p1[1]))

def dist_to_segment(v, w, p):
  l2 = distance(v, w)**2

  if l2 == 0:
    return distance(v, p)

  t = max(0, min(1, np.dot(p-v, w-v) / l2))
  proj = v + t * (w-v)
  return (distance(p, proj), proj)


def calculate_crosstrack_error(waypoints, robot_path):
  total = 0

  for pidx, pt in enumerate(robot_path):

    # Find corresponding waypoint.
    mindist = float('inf')
    best = waypoints[0]
    for ii in range(len(waypoints)-1):
      dist, proj = dist_to_segment(waypoints[ii], waypoints[ii+1], pt)
      if dist < mindist:
        best = proj
        mindist = dist
        
    total += mindist

  print(total)
  return float(total) / len(robot_path)

def get_points(path):
  xs = []
  ys = []
  with open(path) as json_file:
    json_data = json.load(json_file)
    for p in json_data["points"]:
      xs.append(p['x'])
      ys.append(p['y'])
  
  arr = np.zeros((len(xs), 2))
  arr[:,0] = xs
  arr[:,1] = ys
  return arr

if __name__ == '__main__':
  groundtruth_path = '/home/milo/mit/ros/racecar_ws/src/lab6/trajectories/2018-05-10-21-06-40.traj'
  robot_path = '/home/milo/mit/ros/racecar_ws/src/lab6/src/pure_pursuit_path.json'

  gtpoints = get_points(groundtruth_path)
  rbpoints = get_points(robot_path)

  print('GT: %d RB: %d' % (len(gtpoints), len(rbpoints)))

  err = calculate_crosstrack_error(gtpoints, rbpoints)
  print('Crosstrack error:', err)
