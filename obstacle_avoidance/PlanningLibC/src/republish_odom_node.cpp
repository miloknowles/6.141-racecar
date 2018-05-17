#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "utils.hpp"

namespace plc {

class RepublishOdomNode final {
 public:
	RepublishOdomNode() {
		utils::getParamOrFail(pnh_, "node/odom_topic", &odom_topic_);
		utils::getParamOrFail(pnh_, "node/pose_topic", &pose_topic_);
		utils::getParamOrFail(pnh_, "node/world_frame", &world_frame_);
		utils::getParamOrFail(pnh_, "node/car_frame", &car_frame_);

		odom_sub_ = nh_.subscribe(odom_topic_, 10, &RepublishOdomNode::odomUpdateCallback, this);
		pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
		ROS_INFO("Initialized RepublishOdomNode!");
	}

	/**
	 * @brief Republish every odometry measurement as a pose.
	 */
	void odomUpdateCallback(const nav_msgs::Odometry& msg) {
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = world_frame_;
		pose.pose.position = msg.pose.pose.position;
		pose.pose.orientation = msg.pose.pose.orientation;
		pose_pub_.publish(pose);
	}

 private:
 	ros::NodeHandle nh_{"republish_odom_node"};
  ros::NodeHandle pnh_{"~"};

 	std::string odom_topic_{};
 	std::string pose_topic_{};
 	ros::Subscriber odom_sub_{};
 	ros::Publisher pose_pub_{};

 	std::string world_frame_{};
 	std::string car_frame_{};
};

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "republish_odom_node");
  plc::RepublishOdomNode node;
  ros::spin();
	return 0;
}
