#include "clrrt_node.hpp"

namespace plc {

float quaternionToAxisAngleZ(float x, float y, float z, float w) {
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

CLRRTNode::CLRRTNode() {
	utils::getParamOrFail(pnh_, "node/graph_topic", &graph_topic_);
	utils::getParamOrFail(pnh_, "node/path_topic", &path_topic_);
	utils::getParamOrFail(pnh_, "node/map_topic", &map_topic_);
	utils::getParamOrFail(pnh_, "node/goal_topic", &goal_topic_);
	utils::getParamOrFail(pnh_, "node/pose_topic", &pose_topic_);
	utils::getParamOrFail(pnh_, "node/simulate_topic", &simulate_topic_);
	utils::getParamOrFail(pnh_, "node/map_debug_topic", &map_debug_topic_);

	// Set up CLRRT algorithm params.
	utils::getParamOrFail(pnh_, "clrrt/max_iters", &params_.maxIters);
	utils::getParamOrFail(pnh_, "clrrt/horizon_distance", &params_.horizonDistance);
	utils::getParamOrFail(pnh_, "clrrt/goal_sample_rate", &params_.goalSampleRate);
	utils::getParamOrFail(pnh_, "clrrt/goal_pose_tolerance", &params_.goalPoseTolerance);
	utils::getParamOrFail(pnh_, "clrrt/publish_graph", &params_.publishGraph);
	utils::getParamOrFail(pnh_, "clrrt/sampling/mean_r", &params_.meanR);
	utils::getParamOrFail(pnh_, "clrrt/sampling/sigma_r", &params_.sigmaR);
	utils::getParamOrFail(pnh_, "clrrt/sampling/mean_t", &params_.meanT);
	utils::getParamOrFail(pnh_, "clrrt/sampling/sigma_t", &params_.sigmaT);
	utils::getParamOrFail(pnh_, "clrrt/target_speed", &params_.targetSpeed);

	// Set the PurePursuitController params inside of CLRRT.
	PurePursuitController::Params ppParams;
	utils::getParamOrFail(pnh_, "pure_pursuit/max_turn_angle", &ppParams.maxTurnAngle);
	utils::getParamOrFail(pnh_, "pure_pursuit/max_slew_rate", &ppParams.maxSlewRate);
	utils::getParamOrFail(pnh_, "pure_pursuit/wheelbase_length", &ppParams.wheelbaseLength);
	utils::getParamOrFail(pnh_, "pure_pursuit/max_accel", &ppParams.maxAccel);
	utils::getParamOrFail(pnh_, "pure_pursuit/max_decel", &ppParams.maxDecel);
	utils::getParamOrFail(pnh_, "pure_pursuit/max_speed", &ppParams.maxSpeed);
	planner_ = CLRRT(ppParams);

	bool mapServiceAvailable = false;
	std::string mapServiceName;
	utils::getParamOrFail(pnh_, "node/map_service_available", &mapServiceAvailable);
	utils::getParamOrFail(pnh_, "node/map_service_name", &mapServiceName);

	float dilateRadius = 0.5;
	utils::getParamOrFail(pnh_, "clrrt/dilate_radius", &dilateRadius);

	// Set up subscribers.
	goal_sub_ = nh_.subscribe(goal_topic_, 5, &CLRRTNode::goalUpdateCallback, this);
	map_sub_ = nh_.subscribe(map_topic_, 10, &CLRRTNode::mapUpdateCallback, this);
	pose_sub_ = nh_.subscribe(pose_topic_, 10, &CLRRTNode::poseUpdateCallback, this);

	if (mapServiceAvailable) {
		ROS_INFO("Fetching map from service: %s...", mapServiceName.c_str());

		// Block until map is available.
		ros::service::waitForService(mapServiceName);
		ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>(mapServiceName);
		nav_msgs::GetMap service;
		if (client.call(service)) {
			map_.setOccupancyGrid(service.response.map);
			map_.dilateOccupancyGrid(dilateRadius, true);
			ROS_INFO("Received map response from service!");
		}
	}

	// Set up publishers.
	graph_pub_ = nh_.advertise<visualization_msgs::Marker>(graph_topic_, 5);
	path_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(path_topic_, 5);
	path_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(path_viz_topic_, 5);
	sample_pub_ = nh_.advertise<visualization_msgs::Marker>(sample_topic_, 5);
	simulate_pub_ = nh_.advertise<visualization_msgs::Marker>(simulate_topic_, 5);
	map_debug_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_debug_topic_, 5);
	ROS_INFO("Initialized CLRRT node!");
}

void CLRRTNode::publishPath(const std::vector<Pose3>& path,
														bool polygon, bool marker) {
	ROS_INFO("Waypoints: %zu", path.size());

	// Publish the path as a polygon for the controller to use.
	geometry_msgs::PolygonStamped msg;

	// Also publish path as a marker for RViz.
	visualization_msgs::Marker m;
	m.header.frame_id = "map";
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.lifetime = ros::Duration(100);
	m.action = visualization_msgs::Marker::ADD;
	m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.b = 1.0;
  m.color.a = 0.5;

  // Fill in points.
  if (path.size() >= 2) {
  	for (int ii = 0; ii < path.size()-1; ii++) {
	  	geometry_msgs::Point pt, ptNext;
	  	geometry_msgs::Point32 pt32;
			pt.x = path[ii].position.x, pt.y = path[ii].position.y;
			ptNext.x = path[ii+1].position.x, ptNext.y = path[ii+1].position.y;
			pt32.x = path[ii].position.x, pt32.y = path[ii].position.y;

			msg.polygon.points.push_back(pt32);
	    m.points.push_back(pt);
	    m.points.push_back(ptNext);
	  }
	  geometry_msgs::Point32 pt32;
	  pt32.x = path.back().position.x, pt32.y = path.back().position.y;
	  msg.polygon.points.push_back(pt32);
  }
	if (polygon) { path_pub_.publish(msg); }
  if (marker) { path_viz_pub_.publish(m); }
}

bool CLRRTNode::replan() {
	ROS_INFO("Replanning!");
	ROS_INFO("Current pose: %f %f %f", currentPose_.position.x, currentPose_.position.y, currentPose_.theta);
	ROS_INFO("Goal pose: %f %f %f", goalPose_.position.x, goalPose_.position.y, goalPose_.theta);

	path_.clear();
	simulatedPath_.clear();
	bool success = planner_.run(currentPose_, goalPose_, graph_pub_, sample_pub_,
															simulate_pub_, map_, params_, &path_, &simulatedPath_);
	
	if (success) {
		ROS_INFO("Plan found successfully!");
		publishPath(simulatedPath_, false, true); // Publish simulated path as marker.
		publishPath(path_, true, false); // Publish waypoint path to controller (polygon).
	} else {
		// If unsuccessful, send empty path to controller to command a stop.
		// TODO: command some kind of simple primitive?
		ROS_INFO("No plan found.");
		std::vector<Pose3> empty;
		publishPath(empty, true, false);
	}
	
	return success;
}

void CLRRTNode::mapUpdateCallback(const nav_msgs::OccupancyGrid& msg) {
	// ROS_INFO("Map updated callback.");
	// map_.dynamicUpdate(msg, currentPose_);
	// map_.dynamicUpdate(msg, Pose3(0, 0, 0)); // Full map update.
	map_.setOccupancyGrid(msg);
	map_debug_pub_.publish(map_.getOccupancyGridMsg());

	// Check for collisions on current path.
	bool collision = map_.pathOccupiedVectPose3(path_);
	bool simCollision = map_.pathOccupiedVectPose3(simulatedPath_);

	if ((!foundPath_ || simCollision) && goalSet_) {
		printf("Collision: %d SimCollision: %d foundPath: %d goalSet: %d\n",
					collision, simCollision, foundPath_, goalSet_);
		foundPath_ = replan();
	}
}

void CLRRTNode::goalUpdateCallback(const geometry_msgs::PointStamped& msg) {
	ROS_INFO("Goal updated callback.");
	goalPose_ = Pose3(msg.point.x, msg.point.y, 0.0, params_.targetSpeed);
	goalSet_ = true;
	foundPath_ = replan();
}

void CLRRTNode::poseUpdateCallback(const geometry_msgs::PoseStamped& msg) {
	float theta = quaternionToAxisAngleZ(
		msg.pose.orientation.x,
		msg.pose.orientation.y,
		msg.pose.orientation.z,
		msg.pose.orientation.w);
	currentPose_ = Pose3(msg.pose.position.x, msg.pose.position.y, utils::normalizeAngle(theta), 0);
}

} // namespace plc

int main(int argc, char** argv) {
  ros::init(argc, argv, "clrrt_node");
  plc::CLRRTNode node;
  ros::spin();
	return 0;
}
