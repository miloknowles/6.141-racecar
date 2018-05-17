#include "direct_drive.h"

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>       /* pow */

namespace direct_drive
{

double clip(double value, double min, double max) {
	if (value < min) return min;
	else if (value > max) return max;
	return value;
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value);

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_FATAL("DirectDrive: Parameter %s is required.", name.c_str());
  return false;
}

bool parse_axis(Axis *axis, XmlRpc::XmlRpcValue &params) {
  bool got_axis = false, got_target = false, got_scale = false, got_offset = false;
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = params.begin(); it != params.end(); ++it) {
    XmlRpc::XmlRpcValue non_const_blah = const_cast<XmlRpc::XmlRpcValue&>(it->second);

    if (it->first == "axis") {
      axis->axis = (int)non_const_blah;
      got_axis = true;
    } else if (it->first == "target") {
      axis->target = (std::string)non_const_blah;
      got_target = true;
    } else if (it->first == "scale") {
      axis->scale = (double) non_const_blah;
      got_scale = true;
    } else if (it->first == "offset") {
      axis->offset = (double) non_const_blah;
      got_offset = true;
    } else {
      std::cout << "Received unexpected parameter while parsing control axis: " << it->first << std::endl;
      return false;
    }
  }
  return got_axis && got_target && got_scale && got_offset;
}


DirectDrive::DirectDrive(ros::NodeHandle nh, ros::NodeHandle private_nh) :
  teleop_enable(false), autonomous_enable(false),
  vesc_(std::string(),
        boost::bind(&DirectDrive::vescPacketCallback, this, _1),
        boost::bind(&DirectDrive::vescErrorCallback, this, _1)),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  // get ROS parameters
  if (!getRequiredParam(nh, "steering_angle_to_servo_gain", steering_angle_to_servo_gain_)) return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_offset", steering_angle_to_servo_offset_)) return;
  if (!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_)) return;
  if (!getRequiredParam(nh, "vesc_driver/speed_min", speed_min_)) return;
  if (!getRequiredParam(nh, "vesc_driver/speed_max", speed_max_)) return;
  if (!getRequiredParam(nh, "vesc_driver/servo_min", servo_min_)) return;
  if (!getRequiredParam(nh, "vesc_driver/servo_max", servo_max_)) return;

  XmlRpc::XmlRpcValue axis_mappings;
  nh.getParam("teleop/human_control/axis_mappings", axis_mappings);
  ROS_ASSERT(axis_mappings.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < axis_mappings.size(); ++i)  {
    Axis axis_;
    if (parse_axis(&axis_, axis_mappings[i])) {
      axes.push_back(axis_);
    } else {
      std::cout << "ERROR PARSING AXIS" << std::endl;
    }
  }

  speed_setpoint_ = 0.0;
  servo_setpoint_ = steering_angle_to_servo_offset_;

  autonomous_speed_setpoint_ = speed_setpoint_;
  autonomous_servo_setpoint_ = steering_angle_to_servo_offset_;

  std::string port;
  if (!getRequiredParam(nh, "vesc_driver/port", port)) return;

  // attempt to connect to the serial port
  try {
    vesc_.connect(port);
  } catch (vesc_driver::SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }

  // std::cout << "steering_angle_to_servo_gain: " << steering_angle_to_servo_gain_ << std::endl;
  // subscribe to vesc state and. optionally, servo command
  joy_sub_ = nh.subscribe("/joy", 2, &DirectDrive::joyCmdCallback, this);
  drive_sub_ = nh.subscribe("/direct_drive/control", 2, &DirectDrive::driveCmdCallback, this);

  state_pub_ = nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 40);
  drive_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("setpoints", 40);

  timer_ = nh.createTimer(ros::Duration(1.0/100.0), &DirectDrive::timerCallback, this);
}

void DirectDrive::joyCmdCallback(const JoyMsgPtr& cmd) {
  teleop_enable = cmd->buttons[4];
  autonomous_enable = cmd->buttons[5];

  if (teleop_enable) {
    for (int i = 0; i < axes.size(); ++i) {
      Axis ax = axes[i];

      if (ax.target == "drive.steering_angle") {

        servo_setpoint_ = cmd->axes[ax.axis]*steering_angle_to_servo_gain_*ax.scale+steering_angle_to_servo_offset_;
        
      } else if (ax.target == "drive.speed") {
        
        // non-linear speed mapping because it's more intuitive for the user
        double speed_in = cmd->axes[ax.axis];
        if (speed_in < 0) {
          speed_in = -pow(-speed_in, 0.5);
        } else {
          speed_in = pow(speed_in, 0.5);
        }
        speed_setpoint_ = speed_in*speed_to_erpm_gain_*ax.scale;
      }
    }

    // std::cout << speed_setpoint_  << " " << servo_setpoint_ << std::endl;

    if (cmd->buttons[5]) {
      speed_setpoint_ = 0.0;
    }

    speed_setpoint_ = clip(speed_setpoint_, speed_min_, speed_max_);
    servo_setpoint_ = clip(servo_setpoint_, servo_min_, servo_max_);

    // std::cout << "speed: " << speed_setpoint_ << "   servo: " << servo_setpoint_ << std::endl;
  } else {
    speed_setpoint_ = 0.0;
    servo_setpoint_ = steering_angle_to_servo_offset_;
  }
}


// speed in m/s
// steering angle in radians
void DirectDrive::driveCmdCallback(const AckermannDriveStampedPtr& cmd) {
  if (autonomous_enable) {
    // std::cout << "got drive message: " << cmd->drive.speed << " " << cmd->drive.steering_angle << std::endl;
    autonomous_speed_setpoint_ = cmd->drive.speed * speed_to_erpm_gain_;
    autonomous_servo_setpoint_ = cmd->drive.steering_angle*steering_angle_to_servo_gain_+steering_angle_to_servo_offset_;
  }
}

void DirectDrive::timerCallback(const ros::TimerEvent& event) {
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {

    double speed = (autonomous_enable & !teleop_enable) ? autonomous_speed_setpoint_ : speed_setpoint_;
    double servo = (autonomous_enable & !teleop_enable) ? autonomous_servo_setpoint_ : servo_setpoint_;

  	// send setpoint values
  	vesc_.setSpeed(speed);
  	vesc_.setServo(servo);

  	// poll for vesc state (telemetry)
    vesc_.requestState();

  	ackermann_msgs::AckermannDriveStamped::Ptr drive_msg(new ackermann_msgs::AckermannDriveStamped);
    drive_msg->header.stamp = ros::Time::now();
    drive_msg->drive.steering_angle = servo;
    drive_msg->drive.speed = speed;

    drive_pub_.publish(drive_msg);
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}


// store data from the vesc packet


void DirectDrive::vescPacketCallback(const boost::shared_ptr<vesc_driver::VescPacket const>& packet)
{
	// iters++;


	// std::cout << iters << std::endl;
  if (packet->name() == "Values") {
    boost::shared_ptr<vesc_driver::VescPacketValues const> values =
      boost::dynamic_pointer_cast<vesc_driver::VescPacketValues const>(packet);

    vesc_msgs::VescStateStamped::Ptr state_msg(new vesc_msgs::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();

    state_pub_.publish(state_msg);

    // state_pub_.publish(state_msg);
  } else if (packet->name() == "FWVersion") {
    boost::shared_ptr<vesc_driver::VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<vesc_driver::VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void DirectDrive::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}



} // namespace direct_drive

int main(int argc, char** argv)
{
  ros::init(argc, argv, "direct_drive_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  direct_drive::DirectDrive vesc_to_odom(nh, private_nh);

  ros::spin();

  return 0;
}
