#include "orca_control/thruster_allocator.h"

namespace orca_control {

ThrusterAllocator::ThrusterAllocator(ros::NodeHandle& nh) : nh_(nh), num_thrusters_(8) {
  // Initialize subscribers and publishers
  wrench_sub_ = nh_.subscribe("desired_wrench", 1, &ThrusterAllocator::wrenchCallback, this);
  thruster_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("thruster_commands", 1);

  // Initialize thruster configuration matrix
  initThrusterConfig();

  ROS_INFO("Thruster allocator initialized with %d thrusters", num_thrusters_);
}

ThrusterAllocator::~ThrusterAllocator() {
}

void ThrusterAllocator::wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg) {
  // Convert wrench message to Eigen vector
  Eigen::VectorXd wrench(6);
  wrench << msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z;

  // Allocate wrench to thruster commands
  Eigen::VectorXd thruster_commands = allocate(wrench);

  // Publish thruster commands
  std_msgs::Float32MultiArray cmd_msg;
  cmd_msg.data.resize(num_thrusters_);
  for (int i = 0; i < num_thrusters_; ++i) {
    cmd_msg.data[i] = thruster_commands(i);
  }
  thruster_pub_.publish(cmd_msg);
}

Eigen::VectorXd ThrusterAllocator::allocate(const Eigen::VectorXd& wrench) {
  // Use pseudo-inverse to solve for thruster commands
  Eigen::VectorXd thruster_commands = thruster_config_.transpose() * 
                                     (thruster_config_ * thruster_config_.transpose()).inverse() * 
                                     wrench;

  // Apply saturation to thruster commands
  for (int i = 0; i < num_thrusters_; ++i) {
    thruster_commands(i) = std::max(-1.0, std::min(1.0, thruster_commands(i)));
  }

  return thruster_commands;
}

void ThrusterAllocator::initThrusterConfig() {
  // Initialize thruster configuration matrix
  // Each row corresponds to a thruster, each column to a DOF (x, y, z, roll, pitch, yaw)
  thruster_config_ = Eigen::MatrixXd::Zero(num_thrusters_, 6);

  // Get thruster positions from ROS parameters
  std::vector<double> positions_x, positions_y, positions_z;
  std::vector<double> orientations_x, orientations_y, orientations_z;

  // Default thruster positions and orientations if parameters are not available
  if (!nh_.getParam("thruster_positions_x", positions_x)) {
    positions_x = {0.25, 0.25, -0.25, -0.25, 0.15, 0.15, -0.15, -0.15};
  }
  if (!nh_.getParam("thruster_positions_y", positions_y)) {
    positions_y = {-0.2, 0.2, -0.2, 0.2, -0.1, 0.1, -0.1, 0.1};
  }
  if (!nh_.getParam("thruster_positions_z", positions_z)) {
    positions_z = {0.0, 0.0, 0.0, 0.0, 0.15, 0.15, 0.15, 0.15};
  }
  if (!nh_.getParam("thruster_orientations_x", orientations_x)) {
    orientations_x = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (!nh_.getParam("thruster_orientations_y", orientations_y)) {
    orientations_y = {1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (!nh_.getParam("thruster_orientations_z", orientations_z)) {
    orientations_z = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  }

  // Fill in the thruster configuration matrix
  for (int i = 0; i < num_thrusters_; ++i) {
    // Force contribution
    thruster_config_(i, 0) = orientations_x[i];  // X force
    thruster_config_(i, 1) = orientations_y[i];  // Y force
    thruster_config_(i, 2) = orientations_z[i];  // Z force

    // Torque contribution (cross product of position and force)
    thruster_config_(i, 3) = positions_y[i] * orientations_z[i] - positions_z[i] * orientations_y[i];  // Roll
    thruster_config_(i, 4) = positions_z[i] * orientations_x[i] - positions_x[i] * orientations_z[i];  // Pitch
    thruster_config_(i, 5) = positions_x[i] * orientations_y[i] - positions_y[i] * orientations_x[i];  // Yaw
  }

  ROS_INFO("Thruster configuration matrix initialized");
}

}  // namespace orca_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "thruster_allocator");
  ros::NodeHandle nh;

  orca_control::ThrusterAllocator allocator(nh);

  ros::spin();

  return 0;
}
