#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Throws an exception if a parameter cannot be retrieved.
template <typename T>
void getParamOrFail(const ros::NodeHandle& nh, const std::string& name, T* val) {
  if (!nh.getParam(name, *val)) {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(name, true).c_str());
    exit(1);
  }
  return;
}

// Convenience function for displaying an OpenCV image.
void showImage(const cv::Mat& img, const std::string& name) {
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, img);
  cv::waitKey(20);
}

// Convenience function for printing out an std::vector.
template <typename T>
void printVect(const T& v) {
  for (auto vi: v) {
    std::cout << vi << " ";
  }
  std::cout << "\n";
}

// Fills in a matrix from a vector of size rows*cols.
// Doesn't do error checking, so be careful!
void fillMatFromVec(cv::Mat& m, std::vector<float>& v) {
  int h = m.cols;
  for (int ii = 0; ii < v.size(); ++ii) {
    int row = ii / h;
    int col = ii % h;
    m.at<float>(ii / h, ii % h) = v[ii];
  }
}

// LineFollower class definition.
class LineFollower {
 public:
  LineFollower();
  void main();

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void detectEdges(cv::Mat* mono, cv::Mat* edge, cv::Mat* draw);

  // ROS Stuff.
  ros::NodeHandle nh_{};
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_{};
  ros::Publisher path_pub_{};
  ros::Publisher waypoint_pub_{};
  float lookaheadDistance_ = 1.0;

  // Vision stuff.
  int blurKernelSize_ = 5;
  int cannyThresholdLow_ = 50;
  int cannyThresholdHigh_ = 150;
  cv::Scalar minHSV_ = cv::Scalar(0, 178, 204);
  cv::Scalar maxHSV_ = cv::Scalar(50, 255, 255);
  cv::Mat K_ = cv::Mat(3, 3, CV_32FC1, 0.0);
  cv::Mat TF_CAMERA_ROBOT_ = cv::Mat(3, 3, CV_32FC1, 0.0);
  float cameraHeight_ = 0.1;
  int numWaypoints_ = 20;
};

LineFollower::LineFollower() : it_(nh_) {
  std::string rgb_topic;
  getParamOrFail(nh_, "visual_servoing/line_follower/rgb_topic", &rgb_topic);

  std::string path_topic;
  getParamOrFail(nh_, "visual_servoing/line_follower/path_topic", &path_topic);

  std::string waypoint_topic;
  getParamOrFail(nh_, "visual_servoing/line_follower/waypoint_topic", &waypoint_topic);

  getParamOrFail(nh_, "visual_servoing/line_follower/num_waypoints", &numWaypoints_);
  getParamOrFail(nh_, "visual_servoing/line_follower/lookahead_distance", &lookaheadDistance_);

  getParamOrFail(nh_, "visual_servoing/line_follower/blur_kernel_size", &blurKernelSize_);
  getParamOrFail(nh_, "visual_servoing/line_follower/canny_threshold_low", &cannyThresholdLow_);
  getParamOrFail(nh_, "visual_servoing/line_follower/canny_threshold_high", &cannyThresholdHigh_);
  getParamOrFail(nh_, "visual_servoing/line_follower/camera_height", &cameraHeight_);

  // Have to get the list of HSV values a vector, then cast to cv::Scalar.
  std::vector<int> minHSV, maxHSV;
  getParamOrFail(nh_, "visual_servoing/line_follower/min_hsv", &minHSV);
  getParamOrFail(nh_, "visual_servoing/line_follower/max_hsv", &maxHSV);
  minHSV_ = cv::Scalar(minHSV[0], minHSV[1], minHSV[2]);
  maxHSV_ = cv::Scalar(maxHSV[0], maxHSV[1], maxHSV[2]);

  // Retrieve K matrix from camera info.
  std::vector<float> K;
  getParamOrFail(nh_, "visual_servoing/camera_intrinsics", &K);
  fillMatFromVec(K_, K);

  // Get a transform from RDF to FLU (in this case).
  std::vector<float> tf;
  getParamOrFail(nh_, "visual_servoing/tf_camera_robot", &tf);
  fillMatFromVec(TF_CAMERA_ROBOT_, tf);

  // Create publishers and subscribers.
  image_sub_ = it_.subscribe(rgb_topic, 5, &LineFollower::imageCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 10);
  waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(waypoint_topic, 10);

  ROS_INFO("Initialized LineFollower node!");
}

void LineFollower::main() {
  ros::spin();
  printf("Shutting down LineFollower node!\n");
  return;
}

// Apply Gaussian blur, then apply Canny edge detection.
// https://en.wikipedia.org/wiki/Canny_edge_detector
void LineFollower::detectEdges(cv::Mat* mono, cv::Mat* edge, cv::Mat* draw) {
  cv::GaussianBlur(*mono, *mono, cv::Size(blurKernelSize_, blurKernelSize_), 0, 0);
  cv::Canny(*mono, *edge, cannyThresholdLow_, cannyThresholdHigh_, 3);
  edge->convertTo(*draw, CV_8U);
}

struct LinePoint {
  LinePoint(int x, int y) : pt(x, y) {}
  LinePoint() = default;

  // Allows sorting based on y-coordinate of the point.
  bool operator() (cv::Point pt1, cv::Point pt2) {
    return (pt1.y < pt2.y);
  }
  cv::Point pt;
};

// Note: Mat3f creates a 3 channel matrix!
cv::Point3f project3D(const cv::Mat& K, const cv::Point2f& pt, const cv::Mat& tf) {
  cv::Vec3f ptHomo(pt.x, pt.y, 1.0);
  cv::Mat result = tf * (K.inv() * cv::Mat(ptHomo));
  return cv::Vec3f(result);
}

void LineFollower::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

  // cv::Mat mono, edge, draw;
  // cv::cvtColor(img, mono, CV_BGR2GRAY);
  // detectEdges(&mono, &edge, &draw);
  // showImage(draw, "Canny Edges");

  // Convert to hsv and filter the color of interest.
  cv::Mat hsv, filtered;
  cv::cvtColor(img, hsv, CV_BGR2HSV);
  cv::inRange(hsv, minHSV_, maxHSV_, filtered);

  // showImage(img, "Original");
  showImage(hsv, "HSV Image");
  showImage(filtered, "HSV Image (Filtered)");

  // Find the coordinates of points on the detected line.
  std::vector<cv::Point> linePts;
  cv::findNonZero(filtered, linePts);

  // Extract waypoints from the path.
  std::vector<cv::Point> samples;
  LinePoint lp; // This instance is used to overload sorting key.

  // Sort the points on the line by their y-coordinate.
  // Then, sample them by linearly spacing.
  if (linePts.size() >= numWaypoints_) {
    std::sort(linePts.begin(), linePts.end(), lp);
    int skip = linePts.size() / numWaypoints_;

    for (int ii = 0; ii < numWaypoints_; ++ii) {
      int clusterSize = 40;
      if (ii*skip + clusterSize < linePts.size()) {

        cv::Point avg(0, 0);
        for (int jj = 0; jj < clusterSize; ++jj) {
          avg += linePts[ii*skip + jj];
        }
        avg = avg / clusterSize;
        samples.push_back(avg);
      } else {
        samples.push_back(linePts[ii*skip]);
      }
    }
  }

  // Transform points into robot frame (FLU) and publish as path.
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "base_link";

  float pathDistance = 0;
  geometry_msgs::PoseStamped bestWaypoint;
  bestWaypoint.header.stamp = ros::Time::now();
  bestWaypoint.header.frame_id = "base_link";
  float wayPointError = 100;
  for (int ii = 0; ii < samples.size(); ++ii) {
    // This applies inverse homography, and an RDF to FLU transform.
    cv::Point3f pt_world = project3D(K_, samples[ii], TF_CAMERA_ROBOT_);

    if (pt_world.z < -0.01) {
      // We know line points must lie on the ground plane, and we know the height of the camera
      // relative to the ground, allowing us to infer a scale factor. Z_world must equal height!
      // Clamp scale value from blowing up at the horizon.
      float scale = cameraHeight_ / fabs(pt_world.z);
      scale = std::min(scale, float(5));

      geometry_msgs::PoseStamped p;
      p.header.stamp = ros::Time::now();
      p.header.frame_id = "base_link";
      p.pose.position.x = pt_world.x * scale;
      p.pose.position.y = pt_world.y * scale - 0.06;
      p.pose.position.z = pt_world.z * scale;

      // Incrementally add to pathDistance.
      if (ii == 0) {
        bestWaypoint = p;
      }
      path_msg.poses.push_back(p);

      float dist = std::sqrt(std::pow(pt_world.x*scale, 2) + std::pow(pt_world.y*scale, 2));
      if (std::abs(dist - lookaheadDistance_) < wayPointError) {
        bestWaypoint = p;
        wayPointError = std::abs(dist - lookaheadDistance_);
      }
    }
  }
  path_pub_.publish(path_msg);
  // ROS_INFO("Waypoint Error: %f", wayPointError);
  // ROS_INFO("Path distance: %f", pathDistance);
  waypoint_pub_.publish(bestWaypoint);
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_follower");
  LineFollower node;
  node.main();
}
