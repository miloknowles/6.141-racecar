#pragma once

#include <string>
#include <vector>
#include <mutex>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_datatypes.h"

#include "clrrt.hpp"
#include "utils.hpp"
#include "occupancy_grid.hpp"

namespace plc {

float quaternionToAxisAngleZ(float x, float y, float z, float w);

/**
 * @brief Construct a graph visualization using a line list.
 * Every adjacent pair of points in the line list are connected
 * via an edge.
 */
visualization_msgs::Marker makeGraphMarker(const std::vector<Point2f> lineList,
                                           float duration, bool show);

/**
 * @brief Get a line list representation of a tree for visualization.
 */
std::vector<Point2f> graphToLineList(const MotionPlanningTree<TreeNode>& tree);

class CLRRTNode final {
 public:
 	CLRRTNode();

 	~CLRRTNode() = default;

  CLRRTNode(const CLRRTNode& rhs) = delete;
  CLRRTNode& operator=(const CLRRTNode& rhs) = delete;

  CLRRTNode(CLRRTNode&& rhs) = delete;
  CLRRTNode& operator=(CLRRTNode&& rhs) = delete;
  
  void mapUpdateCallback(const nav_msgs::OccupancyGrid& msg);

  void goalUpdateCallback(const geometry_msgs::PointStamped& msg);

  void poseUpdateCallback(const geometry_msgs::PoseStamped& msg);

  void graphUpdatedCallback();

  bool replan();

  void publishPath(const std::vector<Pose3>& path,
                   bool polygon = true, bool marker = true);

 private:
 	ros::NodeHandle nh_{"clrrt_node"};
  ros::NodeHandle pnh_{"~"};

  std::string graph_topic_ = "/clrrt/tree";
  std::string path_topic_ = "/clrrt/path";
  std::string sample_topic_ = "/visualization/sample";
  std::string path_viz_topic_ = "/visualization/path";
  std::string simulate_topic_ = "/visualization/simulate";
  ros::Publisher graph_pub_{};
  ros::Publisher path_pub_{};
  ros::Publisher sample_pub_{};
  ros::Publisher path_viz_pub_{};
  ros::Publisher simulate_pub_{};
  ros::Publisher map_debug_pub_{};

  std::string map_topic_ = "/cartographer/map";
  std::string goal_topic_ = "/clicked_pose";
  std::string pose_topic_ = "/clrrt/pose";
  std::string map_debug_topic_ = "/debug_map";
  ros::Subscriber map_sub_{};
  ros::Subscriber goal_sub_{};
  ros::Subscriber pose_sub_{};

  float mapUpdateRate_ = 5.0;

  bool foundPath_ = false;
  bool goalSet_ = false;

  OccupancyGrid map_{};

  CLRRT::Params params_{};
  CLRRT planner_{};

  Pose3 currentPose_{};
  Pose3 goalPose_{};

  std::vector<Pose3> path_{};
  std::vector<Pose3> simulatedPath_{};

  std::mutex replan_mutex_{};
};

} // namespace plc
