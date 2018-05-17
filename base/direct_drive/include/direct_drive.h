// -*- mode:c++; fill-column: 100; -*-

#ifndef DIRECT_DRIVE_H_
#define DIRECT_DRIVE_H_

#include <ros/ros.h>
#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <iostream>
#include <boost/optional.hpp>
#include <vesc_driver/vesc_interface.h>
#include <vesc_driver/vesc_packet.h>

namespace direct_drive
{

typedef sensor_msgs::Joy::ConstPtr JoyMsgPtr;
typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannDriveStampedPtr;

struct Axis
{
  int axis;
  std::string target;
  double scale;
  double offset;
};

class DirectDrive
{
public:

  DirectDrive(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // interface to the VESC
  vesc_driver::VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<vesc_driver::VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // ROS callbacks
  void driveCmdCallback(const AckermannDriveStampedPtr& cmd);
  void joyCmdCallback(const JoyMsgPtr& cmd);
  void timerCallback(const ros::TimerEvent& event);

  // State
  double speed_to_erpm_gain_;
  double steering_angle_to_servo_gain_;
  double steering_angle_to_servo_offset_;
  double speed_min_;
  double speed_max_;
  double servo_min_;
  double servo_max_;

  std::vector<Axis> axes;

  // simple MUX - tied to the triggers on the joystick
  bool teleop_enable;
  bool autonomous_enable;

  double speed_setpoint_;
  double servo_setpoint_;

  double autonomous_speed_setpoint_;
  double autonomous_servo_setpoint_;

  // ----- stuff related to VESC -----
  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  // ROS services
  ros::Subscriber joy_sub_;
  ros::Subscriber drive_sub_;
  ros::Publisher state_pub_;
  ros::Publisher drive_pub_;
  ros::Timer timer_;
};

} // namespace direct_drive

#endif // DIRECT_DRIVE_H_
