#ifndef ORCA_CONTROL_DEPTH_CONTROLLER_H
#define ORCA_CONTROL_DEPTH_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/FluidPressure.h>
#include <mavros_msgs/Altitude.h>
#include <control_toolbox/pid.h>

namespace orca_control {

/**
 * @brief Depth controller for the Orca ROV
 * 
 * This class implements a PID controller for depth control of the Orca ROV.
 * It uses pressure sensor data to estimate depth and generates vertical
 * force commands to maintain the desired depth.
 */
class DepthController {
public:
  /**
   * @brief Constructor
   * @param nh ROS node handle
   */
  DepthController(ros::NodeHandle& nh);

  /**
   * @brief Destructor
   */
  ~DepthController();

private:
  /**
   * @brief Callback for pressure sensor data
   * @param msg Pressure sensor message
   */
  void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);

  /**
   * @brief Callback for MAVROS altitude data
   * @param msg Altitude message
   */
  void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);

  /**
   * @brief Callback for depth setpoint
   * @param msg Depth setpoint message
   */
  void setpointCallback(const std_msgs::Float64::ConstPtr& msg);

  /**
   * @brief Timer callback for control loop
   * @param event Timer event
   */
  void controlCallback(const ros::TimerEvent& event);

  /**
   * @brief Convert pressure to depth
   * @param pressure Pressure in Pascals
   * @return Depth in meters
   */
  double pressureToDepth(double pressure);

  ros::NodeHandle nh_;
  ros::Subscriber pressure_sub_;
  ros::Subscriber altitude_sub_;
  ros::Subscriber setpoint_sub_;
  ros::Publisher force_pub_;
  ros::Timer control_timer_;

  control_toolbox::Pid pid_controller_;

  double current_depth_;
  double setpoint_depth_;
  double atmospheric_pressure_;
  double water_density_;
  double gravity_;
  bool use_mavros_altitude_;
};

}  // namespace orca_control

#endif  // ORCA_CONTROL_DEPTH_CONTROLLER_H
