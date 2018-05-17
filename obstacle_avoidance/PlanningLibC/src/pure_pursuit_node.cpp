#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "pure_pursuit_controller.hpp"
#include "pose3.hpp"
#include "point2.hpp"
#include "utils.hpp"

namespace plc {

float quaternionToAxisAngleZ(float x, float y, float z, float w) {
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

class PurePursuitNode final {
 public:
 	PurePursuitNode() {
		utils::getParamOrFail(pnh_, "pure_pursuit/max_turn_angle", &params_.maxTurnAngle);
		utils::getParamOrFail(pnh_, "pure_pursuit/max_slew_rate", &params_.maxSlewRate);
		utils::getParamOrFail(pnh_, "pure_pursuit/wheelbase_length", &params_.wheelbaseLength);
		utils::getParamOrFail(pnh_, "pure_pursuit/max_accel", &params_.maxAccel);
		utils::getParamOrFail(pnh_, "pure_pursuit/max_decel", &params_.maxDecel);
		utils::getParamOrFail(pnh_, "pure_pursuit/max_speed", &params_.maxSpeed);
		utils::getParamOrFail(pnh_, "pure_pursuit/min_lookahead_dist", &params_.minLookaheadDist);
		utils::getParamOrFail(pnh_, "pure_pursuit/max_lookahead_dist", &params_.maxLookaheadDist);
		utils::getParamOrFail(pnh_, "pure_pursuit/target_speed", &targetSpeed_);
		controller_ = PurePursuitController(params_);

		utils::getParamOrFail(pnh_, "node/path_topic", &path_topic_);
		utils::getParamOrFail(pnh_, "node/drive_topic", &drive_topic_);
		utils::getParamOrFail(pnh_, "node/pose_topic", &pose_topic_);
		utils::getParamOrFail(pnh_, "node/ref_input_topic", &ref_input_topic_);

		// Set up publishers and subscribers.
		drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic_, 10);
		ref_input_pub_ = nh_.advertise<visualization_msgs::Marker>(ref_input_topic_, 10);

		path_sub_ = nh_.subscribe(path_topic_, 10, &PurePursuitNode::pathUpdatedCallback, this);
		pose_sub_ = nh_.subscribe(pose_topic_, 10, &PurePursuitNode::localizationCallback, this);

		ROS_INFO("Initialized PurePursuitNode!");
 	}

 	void publishDrive(const ControlOutput& control) {
 		// ROS_INFO("Drive command: %f %f", control.steerAngle, control.speed);
 		ackermann_msgs::AckermannDriveStamped msg;
 		msg.drive.steering_angle = control.steerAngle;
 		msg.drive.speed = control.speed;
 		drive_pub_.publish(msg);
 		return;
 	}

 	void publishRefInputMarker(const PurePursuitInput& refInput) {
 		visualization_msgs::Marker m;
	  m.header.frame_id = "map";
	  m.ns = "visualization/ref_input";
	  m.id = 102;
	  m.type = visualization_msgs::Marker::CUBE;
	  m.lifetime = ros::Duration(100);
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.pose.position.x = refInput.position.x;
    m.pose.position.y = refInput.position.y;
	  ref_input_pub_.publish(m);
 	}

 	void localizationCallback(const geometry_msgs::PoseStamped& msg) {
 		// Wait until path set.
 		float theta = quaternionToAxisAngleZ(
 		msg.pose.orientation.x, msg.pose.orientation.y,
 		msg.pose.orientation.z, msg.pose.orientation.w);
 		Pose3 currentPose(msg.pose.position.x, msg.pose.position.y, theta, targetSpeed_);

 		if (active_) {
	 		std::pair<int, Point2f> refInputPair = controller_.chooseRefInput(currentPose, path_, pathIdx_);

	 		// Store the latest point that we are going to.
	 		pathIdx_ = refInputPair.first;
	 		// ROS_INFO("Path idx: %d", pathIdx_);

	 		PurePursuitInput refInput(refInputPair.second, targetSpeed_);
	 		publishRefInputMarker(refInput);

	 		// Generate a control and send to VESC.
	 		ControlOutput control = controller_.getControlOutput(currentPose, refInput);
	 		publishDrive(control);
 		} else {
 			ControlOutput control(0.0, 0.0); // Command a stop.
 		}
 	}

 	void pathUpdatedCallback(const geometry_msgs::PolygonStamped& msg) {
 		ROS_INFO("Received a path with %zu points.", msg.polygon.points.size());
 		if (msg.polygon.points.size() >= 2) {
 			path_.clear();
	 		for (const geometry_msgs::Point32& pt: msg.polygon.points) {
	 			path_.push_back(Pose3(pt.x, pt.y, 0, targetSpeed_));
	 		}
	 		active_ = true;
 		} else {
 			active_ = false;
 		}
 	}

 private:
 	ros::NodeHandle nh_{"pure_pursuit_node"};
 	ros::NodeHandle pnh_{"~"};

 	PurePursuitController controller_{};
 	PurePursuitController::Params params_{};

 	// State of the controller.
 	bool active_ = false;
 	std::vector<Pose3> path_{};
 	int pathIdx_ = 0;
 	float targetSpeed_ = 2.5;

 	// Publisher / subscriber config.
 	std::string drive_topic_ = "/drive";
 	std::string path_topic_ = "/path";
 	std::string pose_topic_ = "/pose";
 	std::string ref_input_topic_ = "/ref_input";

 	ros::Publisher drive_pub_{};
 	ros::Publisher ref_input_pub_{};
 	ros::Subscriber path_sub_{};
 	ros::Subscriber pose_sub_{};
};

} // namespace plc

int main(int argc, char** argv) {
	ros::init(argc, argv, "pure_pursuit_node");
  plc::PurePursuitNode node;
  ros::spin();
	return 0;
}
