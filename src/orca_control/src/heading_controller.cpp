#include "orca_control/heading_controller.h"
#include <geometry_msgs/Wrench.h>

namespace orca_control {

HeadingController::HeadingController(ros::NodeHandle& nh) : nh_(nh), heading_initialized_(false) {
  // Initialize PID controller
  double p, i, d, i_max, i_min;
  nh_.param<double>("heading_p", p, 5.0);
  nh_.param<double>("heading_i", i, 0.1);
  nh_.param<double>("heading_d", d, 1.0);
  nh_.param<double>("heading_i_max", i_max, 10.0);
  nh_.param<double>("heading_i_min", i_min, -10.0);
  pid_controller_.initPid(p, i, d, i_max, i_min);

  // Initialize subscribers and publishers
  imu_sub_ = nh_.subscribe("/mavros/imu/data", 1, &HeadingController::imuCallback, this);
  setpoint_sub_ = nh_.subscribe("heading_setpoint", 1, &HeadingController::setpointCallback, this);
  torque_pub_ = nh_.advertise<geometry_msgs::Wrench>("desired_wrench", 1);

  // Initialize control timer (10 Hz)
  control_timer_ = nh_.createTimer(ros::Duration(0.1), &HeadingController::controlCallback, this);

  // Initialize state
  current_heading_ = 0.0;
  setpoint_heading_ = 0.0;

  ROS_INFO("Heading controller initialized");
}

HeadingController::~HeadingController() {
}

void HeadingController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // Extract heading from quaternion
  tf2::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w);
  
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  // Store current heading
  current_heading_ = yaw;
  heading_initialized_ = true;
}

void HeadingController::setpointCallback(const std_msgs::Float64::ConstPtr& msg) {
  // Normalize setpoint to [-pi, pi]
  setpoint_heading_ = msg->data;
  while (setpoint_heading_ > M_PI) setpoint_heading_ -= 2.0 * M_PI;
  while (setpoint_heading_ < -M_PI) setpoint_heading_ += 2.0 * M_PI;
  
  ROS_INFO("Heading setpoint set to %.2f degrees", setpoint_heading_ * 180.0 / M_PI);
}

void HeadingController::controlCallback(const ros::TimerEvent& event) {
  if (!heading_initialized_) {
    ROS_WARN_THROTTLE(5.0, "Waiting for IMU data...");
    return;
  }
  
  // Calculate heading error
  double error = calculateHeadingError(setpoint_heading_, current_heading_);
  
  // Compute control output
  ros::Time time = ros::Time::now();
  double control_effort = pid_controller_.computeCommand(error, time - event.last_real);
  
  // Create wrench message
  geometry_msgs::Wrench wrench_msg;
  wrench_msg.force.x = 0.0;
  wrench_msg.force.y = 0.0;
  wrench_msg.force.z = 0.0;
  wrench_msg.torque.x = 0.0;
  wrench_msg.torque.y = 0.0;
  wrench_msg.torque.z = control_effort;
  
  // Publish wrench
  torque_pub_.publish(wrench_msg);
}

double HeadingController::calculateHeadingError(double setpoint, double current) {
  // Calculate heading error with wrap-around
  double error = setpoint - current;
  
  // Normalize error to [-pi, pi]
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;
  
  return error;
}

}  // namespace orca_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "heading_controller");
  ros::NodeHandle nh;

  orca_control::HeadingController controller(nh);

  ros::spin();

  return 0;
}
