#ifndef ORCA_CONTROL_HEADING_CONTROLLER_H
#define ORCA_CONTROL_HEADING_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <control_toolbox/pid.h>

namespace orca_control {

/**
 * @brief Heading controller for the Orca ROV
 * 
 * This class implements a PID controller for heading control of the Orca ROV.
 * It uses IMU data to estimate heading and generates yaw torque commands
 * to maintain the desired heading.
 */
class HeadingController {
public:
  /**
   * @brief Constructor
   * @param nh ROS node handle
   */
  HeadingController(ros::NodeHandle& nh);

  /**
   * @brief Destructor
   */
  ~HeadingController();

private:
  /**
   * @brief Callback for IMU data
   * @param msg IMU message
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for heading setpoint
   * @param msg Heading setpoint message
   */
  void setpointCallback(const std_msgs::Float64::ConstPtr& msg);

  /**
   * @brief Timer callback for control loop
   * @param event Timer event
   */
  void controlCallback(const ros::TimerEvent& event);

  /**
   * @brief Calculate heading error with wrap-around
   * @param setpoint Desired heading in radians
   * @param current Current heading in radians
   * @return Heading error in radians
   */
  double calculateHeadingError(double setpoint, double current);

  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber setpoint_sub_;
  ros::Publisher torque_pub_;
  ros::Timer control_timer_;

  control_toolbox::Pid pid_controller_;

  double current_heading_;
  double setpoint_heading_;
  bool heading_initialized_;
};

}  // namespace orca_control

#endif  // ORCA_CONTROL_HEADING_CONTROLLER_H
