#include "orca_hw_interface/ms5837.h"
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pressure_sensor_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Get parameters
  std::string i2c_bus;
  int i2c_address;
  double fluid_density;
  double update_rate;

  pnh.param<std::string>("i2c_bus", i2c_bus, "/dev/i2c-1");
  pnh.param<int>("i2c_address", i2c_address, 0x76);
  pnh.param<double>("fluid_density", fluid_density, 1000.0);
  pnh.param<double>("update_rate", update_rate, 10.0);

  // Create publishers
  ros::Publisher pressure_pub = nh.advertise<sensor_msgs::FluidPressure>("pressure/data", 10);
  ros::Publisher temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature/data", 10);
  ros::Publisher depth_pub = nh.advertise<std_msgs::Float64>("depth", 10);

  // Create sensor driver
  orca_hw_interface::MS5837 sensor(i2c_bus, i2c_address);

  // Initialize sensor
  if (!sensor.init()) {
    ROS_ERROR("Failed to initialize MS5837 pressure sensor");
    return 1;
  }

  ROS_INFO("MS5837 pressure sensor initialized");

  // Main loop
  ros::Rate rate(update_rate);
  while (ros::ok()) {
    // Read sensor data
    if (sensor.read()) {
      // Create and publish pressure message
      sensor_msgs::FluidPressure pressure_msg;
      pressure_msg.header.stamp = ros::Time::now();
      pressure_msg.fluid_pressure = sensor.getPressure();
      pressure_msg.variance = 0.0;  // Unknown variance
      pressure_pub.publish(pressure_msg);

      // Create and publish temperature message
      sensor_msgs::Temperature temperature_msg;
      temperature_msg.header.stamp = ros::Time::now();
      temperature_msg.temperature = sensor.getTemperature();
      temperature_msg.variance = 0.0;  // Unknown variance
      temperature_pub.publish(temperature_msg);

      // Create and publish depth message
      std_msgs::Float64 depth_msg;
      depth_msg.data = sensor.getDepth(fluid_density);
      depth_pub.publish(depth_msg);

      ROS_DEBUG("Pressure: %.2f Pa, Temperature: %.2f C, Depth: %.2f m",
                sensor.getPressure(), sensor.getTemperature(), sensor.getDepth(fluid_density));
    } else {
      ROS_WARN_THROTTLE(1, "Failed to read from MS5837 pressure sensor");
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
