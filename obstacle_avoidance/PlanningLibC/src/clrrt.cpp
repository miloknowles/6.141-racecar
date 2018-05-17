#include "clrrt.hpp"
#include <chrono>
#include <thread>

using Clocktime = std::chrono::steady_clock::time_point;

namespace plc {

void CLRRT::graphUpdatedCallback(const ros::Publisher& pub) {
  std::vector<Point2f> lineList = graphToLineList(tree_);
  pub.publish(utils::makeGraphMarker(lineList, 100, true));
}

float forwardProgressHeuristic(const Pose3& nodePose, const Pose3& samplePose, float costSoFar) {
  // return (costSoFar + nodePose.distance(samplePose));
  return nodePose.distance(samplePose);
}

Pose3 forwardProgressSampling(const Pose3& pose, float meanR, float sigmaR,
                              float meanT, float sigmaT, std::default_random_engine& rng) {
  std::normal_distribution<float> rDist(meanR, sigmaR);
  std::normal_distribution<float> tDist(meanT, sigmaT);
  float randR = rDist(rng);
  float randT = tDist(rng) + pose.theta;
  float x = pose.position.x + randR*std::cos(randT);
  float y = pose.position.y + randR*std::sin(randT);
  return Pose3(x, y, randT);
}

Pose3 biasedForwardProgressSampling(const Pose3& goalPose, float goalSampleRate,
                                    const Pose3& pose, float meanR, float sigmaR,
                                    float meanT, float sigmaT, std::default_random_engine& rng) {
  float randFloat = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  if (randFloat < goalSampleRate) {
    return goalPose;
  } else {
    return forwardProgressSampling(pose, meanR, sigmaR, meanT, sigmaT, rng);
  }
}

Pose3 biasedBoxSampling(const Pose3& pose, const Pose3& goalPose, const float goalSampleRate,
                        const float width) {
  // Return the goal with some probability.
  float randFloat = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  if (randFloat < goalSampleRate) {
    return goalPose;
  }

  // Otherwise return a Pose3 in the box around the car.
  Point2f goalVect = goalPose.position - pose.position;
  float forward = 1.1 * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

  Point2f normal = Point2f(-goalVect.y, goalVect.x) / utils::norm(goalVect);
  float sideways = width * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - (0.5*width);

  // Go forward some fraction of goal vector.
  // Go sideways some fraction of width times sideways vector.
  Point2f sample = pose.position + goalVect*forward + normal*sideways;
  return Pose3(sample, 0); // Theta doesn't matter.
}

bool CLRRT::simulateForward(const Pose3& startPose, const std::vector<Pose3>& refPath,
                            const float targetSpeed, const int goalIdx, float dt,
                            std::vector<Pose3>* path) {
  path->push_back(startPose);

  // This idx should be 1 for most cases (start, goal, end)
  PurePursuitInput goalRefInput(refPath[goalIdx].position, targetSpeed); 

  // This gets updated in the loop.
  Pose3 currentPose = startPose;
  if (currentPose.speed == 0) {
    currentPose.speed = 0.1;
  }
  
  // This makes sure that the controller can't diverge and loop infinitely.
  int maxSteps = (int) (3 * utils::pathLength(refPath) / (targetSpeed * dt));

  float prevGoalDist = std::numeric_limits<float>::max();

  // Keep simulating until divergence or timeout.
  while (currentPose.position.distance(goalRefInput.position) < prevGoalDist &&
         path->size() < maxSteps) {

    prevGoalDist = currentPose.position.distance(goalRefInput.position);

    // The controller will determine a lookahead distance internally.
    // Make sure that it is initialized with good params.
    std::pair<int, Point2f> refPair = controller_.chooseRefInput(currentPose, refPath, 0);

    // The command will use whatever speed is set on the goalRefInput.
    PurePursuitInput refInput(refPair.second, goalRefInput.speed);
    ControlOutput control = controller_.getControlOutput(currentPose, refInput);

    // Simulate forward by one timestep.
    currentPose = controller_.simulateForward(currentPose, control, dt);
    path->push_back(currentPose);
  }

  // Worst case convergence.
  bool success = path->back().position.distance(goalRefInput.position) < (3 * goalRefInput.speed * dt);

  // Success: refine the endpoint of the path.
  if (success) {
    // To avoid accumulating discretization error, look over the last 2 points
    // in the path and find the closest point to goalRefInput.
    if (path->size() > 3) {
      Point2f closestLastSeg = utils::closestPtOnSegment((*path)[path->size()-2].position,
        (*path)[path->size()-1].position, goalRefInput.position);

      Point2f closestPenSeg = utils::closestPtOnSegment((*path)[path->size()-3].position,
        (*path)[path->size()-2].position, goalRefInput.position);

      if (closestLastSeg.distance(goalRefInput.position) < closestPenSeg.distance(goalRefInput.position)) {
        (*path)[path->size()-1].position = closestLastSeg;
      } else {
        path->pop_back();
        (*path)[path->size()-1].position = closestPenSeg;
      }
    }
    return true;
  } else {
    return false;
  }
}

bool recedingHorizonGoalTest(const Pose3& currentPose, const Pose3& goalPose,
                             const Pose3 newPose, float horizonDistance) {
  Point2f goalVect(goalPose.position.x - currentPose.position.x,
                   goalPose.position.y - currentPose.position.y);
  Point2f testVect(newPose.position.x - currentPose.position.x,
                   newPose.position.y - currentPose.position.y);
  Point2f progressVect(testVect - goalVect);
  float progress = utils::dot(progressVect, goalVect);
  return (progress > 0 || newPose.distance(goalPose) < 0.1);
}

visualization_msgs::Marker getPointMarker(const Pose3& pose) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.g = 1.0;
  return marker;
}

visualization_msgs::Marker getPathMarker(const std::vector<Pose3>& path) {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.id = 103;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.lifetime = ros::Duration(100);
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.b = 1.0;
  m.color.a = 0.5;

  // Fill in points.
  for (int ii = 0; ii < path.size()-1; ii++) {
    geometry_msgs::Point pt, ptNext;
    pt.x = path[ii].position.x, pt.y = path[ii].position.y;
    ptNext.x = path[ii+1].position.x, ptNext.y = path[ii+1].position.y;
    m.points.push_back(pt);
    m.points.push_back(ptNext);
  }
  return m;
}

bool CLRRT::run(const Pose3& currentPose, const Pose3& goalPose, const ros::Publisher& graph_pub,
                const ros::Publisher& sample_pub, const ros::Publisher& simulate_pub,
                const OccupancyGrid& map, const Params& params,
                std::vector<Pose3>* path, std::vector<Pose3>* simPath) {

  Clocktime beginTime = std::chrono::steady_clock::now();

  int iter = 0;
  bool foundGoal = false;

  // Add the currentPose to the tree.
  tree_.clear();
  TreeNode startNode(currentPose, PurePursuitInput(currentPose.position, 0), 0, 0);

  // A parent of -1 signifies that this is the root.
  tree_.addNode(startNode, -1, 0);

  while (!foundGoal && iter < params.maxIters) {
    iter++;
    if (params.verbose) { printf("[CLRRT]: Iteration %d \n", iter); }

    // Generate a sample waypoint for the pure pursuit controller in front of the car.
    Pose3 samplePose = biasedBoxSampling(currentPose, goalPose, params.goalSampleRate, 10.0);

    sample_pub.publish(getPointMarker(samplePose));

    // Heuristic: cost_so_far(node) + dist(node, sample). Sort based on heuristic.
    std::vector<std::pair<float, TreeNode>> neighborsSort;
    std::vector<TreeNode>::const_iterator it;
    for (it = tree_.getNodes().begin(); it != tree_.getNodes().end(); it++) {
      TreeNode node = *it;
      neighborsSort.push_back(
        std::pair<float, TreeNode>(forwardProgressHeuristic(node.state, samplePose, node.cost), node));
    }

    // Sort based on the first value of pair.
    std::sort(neighborsSort.begin(), neighborsSort.end(),
      [](const std::pair<float, TreeNode>& p1, const std::pair<float, TreeNode>& p2) {
        return p1.first < p2.first;
      });

    // For each node, connect to the sample, creating an input to controller.
    for (const std::pair<float, TreeNode>& candidate : neighborsSort) {
      Pose3 startPose = candidate.second.state;
      PurePursuitInput startRefInput = candidate.second.input;
      PurePursuitInput goalRefInput(samplePose.position, params.targetSpeed);

      // Forward simulate a path by giving the reference input to pure pursuit.
      // Hack: add a third point that acts as an extension to stabilize the controller.
      std::vector<Pose3> path;
      std::vector<Pose3> refPath = {Pose3(startRefInput.position, 0, params.targetSpeed),
        Pose3(goalRefInput.position, 0, params.targetSpeed),
        Pose3(goalRefInput.position*2.0-startRefInput.position, 0, params.targetSpeed)};

      bool success = simulateForward(startPose, refPath, params.targetSpeed, 1, params.dt, &path);

      if (params.publishGraph) {
        visualization_msgs::Marker pathMark = getPathMarker(path);
        simulate_pub.publish(pathMark);
      }

      if (success) {
        // Check the path for collisions.
        bool hasCollision = map.pathOccupiedVectPose3(path);

        if (!hasCollision) {
          Pose3 finalState = path.back();

          TreeNode newNode(finalState, goalRefInput, tree_.numNodes(),
                           candidate.second.cost + utils::pathLength(path));
          tree_.addNode(newNode, candidate.second.id, candidate.second.cost + utils::pathLength(path));

          // Check if the goal was reached.
          if (recedingHorizonGoalTest(currentPose, goalPose, finalState, params.horizonDistance)) {
            foundGoal = true;
          }
          // Use the publisher that was passed in to show debug graph.
          if (params.publishGraph) {
            graphUpdatedCallback(graph_pub);
          }
          break;
        }
      }
      // Uncomment this to add a loop delay.
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  // Reconstruct path.
  if (foundGoal) {
    std::vector<TreeNode> treePath;
    tree_.reconstructPath(&treePath); // Default endIdx will be the most recently added node.

    std::vector<Pose3> refPath;
    for (const TreeNode& node : treePath) {
      refPath.push_back(node.state);
    }
    (*path) = refPath;

    // std::vector<Pose3> simulatedPath;
    simulateForward(currentPose, refPath, params.targetSpeed, refPath.size()-1, params.dt, simPath);
    // simulate_pub.publish(getPathMarker(*simPath));
  }

  Clocktime endTime = std::chrono::steady_clock::now();
  utils::printElapsed(beginTime, endTime);

  return foundGoal;
}

}
