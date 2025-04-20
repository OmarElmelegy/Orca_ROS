#include "orca_control/depth_controller.h"
#include <geometry_msgs/Wrench.h>

namespace orca_control {

DepthController::DepthController(ros::NodeHandle& nh) : nh_(nh) {
  // Initialize parameters
  nh_.param<double>("atmospheric_pressure", atmospheric_pressure_, 101325.0);  // Pa
  nh_.param<double>("water_density", water_density_, 1000.0);  // kg/m^3
  nh_.param<double>("gravity", gravity_, 9.80665);  // m/s^2
  nh_.param<bool>("use_mavros_altitude", use_mavros_altitude_, true);

  // Initialize PID controller
  double p, i, d, i_max, i_min;
  nh_.param<double>("depth_p", p, 100.0);
  nh_.param<double>("depth_i", i, 10.0);
  nh_.param<double>("depth_d", d, 20.0);
  nh_.param<double>("depth_i_max", i_max, 100.0);
  nh_.param<double>("depth_i_min", i_min, -100.0);
  pid_controller_.initPid(p, i, d, i_max, i_min);

  // Initialize subscribers and publishers
  pressure_sub_ = nh_.subscribe("pressure", 1, &DepthController::pressureCallback, this);
  altitude_sub_ = nh_.subscribe("/mavros/altitude", 1, &DepthController::altitudeCallback, this);
  setpoint_sub_ = nh_.subscribe("depth_setpoint", 1, &DepthController::setpointCallback, this);
  force_pub_ = nh_.advertise<geometry_msgs::Wrench>("desired_wrench", 1);

  // Initialize control timer (10 Hz)
  control_timer_ = nh_.createTimer(ros::Duration(0.1), &DepthController::controlCallback, this);

  // Initialize state
  current_depth_ = 0.0;
  setpoint_depth_ = 0.0;

  ROS_INFO("Depth controller initialized");
}

DepthController::~DepthController() {
}

void DepthController::pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
  if (!use_mavros_altitude_) {
    current_depth_ = pressureToDepth(msg->fluid_pressure);
  }
}

void DepthController::altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg) {
  if (use_mavros_altitude_) {
    // Note: MAVROS altitude is negative underwater
    current_depth_ = -msg->relative;
  }
}

void DepthController::setpointCallback(const std_msgs::Float64::ConstPtr& msg) {
  setpoint_depth_ = msg->data;
  ROS_INFO("Depth setpoint set to %.2f meters", setpoint_depth_);
}

void DepthController::controlCallback(const ros::TimerEvent& event) {
  // Calculate depth error
  double error = setpoint_depth_ - current_depth_;
  
  // Compute control output
  ros::Time time = ros::Time::now();
  double control_effort = pid_controller_.computeCommand(error, time - event.last_real);
  
  // Create wrench message
  geometry_msgs::Wrench wrench_msg;
  wrench_msg.force.x = 0.0;
  wrench_msg.force.y = 0.0;
  wrench_msg.force.z = control_effort;
  wrench_msg.torque.x = 0.0;
  wrench_msg.torque.y = 0.0;
  wrench_msg.torque.z = 0.0;
  
  // Publish wrench
  force_pub_.publish(wrench_msg);
}

double DepthController::pressureToDepth(double pressure) {
  // Convert pressure to depth using the hydrostatic pressure equation
  // depth = (pressure - atmospheric_pressure) / (water_density * gravity)
  return (pressure - atmospheric_pressure_) / (water_density_ * gravity_);
}

}  // namespace orca_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_controller");
  ros::NodeHandle nh;

  orca_control::DepthController controller(nh);

  ros::spin();

  return 0;
}
