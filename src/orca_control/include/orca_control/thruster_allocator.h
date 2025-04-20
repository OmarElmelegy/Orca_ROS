#ifndef ORCA_CONTROL_THRUSTER_ALLOCATOR_H
#define ORCA_CONTROL_THRUSTER_ALLOCATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>

namespace orca_control {

/**
 * @brief Allocates desired wrench to individual thruster commands
 * 
 * This class implements a thruster allocation algorithm for the Orca ROV
 * with 8 thrusters. It maps a desired body wrench (force and torque)
 * to individual thruster commands.
 */
class ThrusterAllocator {
public:
  /**
   * @brief Constructor
   * @param nh ROS node handle
   */
  ThrusterAllocator(ros::NodeHandle& nh);

  /**
   * @brief Destructor
   */
  ~ThrusterAllocator();

private:
  /**
   * @brief Callback for desired wrench
   * @param msg Desired wrench message
   */
  void wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg);

  /**
   * @brief Allocate wrench to thruster commands
   * @param wrench Desired wrench
   * @return Thruster commands
   */
  Eigen::VectorXd allocate(const Eigen::VectorXd& wrench);

  /**
   * @brief Initialize thruster configuration matrix
   */
  void initThrusterConfig();

  ros::NodeHandle nh_;
  ros::Subscriber wrench_sub_;
  ros::Publisher thruster_pub_;

  Eigen::MatrixXd thruster_config_;  // Thruster configuration matrix
  int num_thrusters_;                // Number of thrusters
};

}  // namespace orca_control

#endif  // ORCA_CONTROL_THRUSTER_ALLOCATOR_H
