# Orca ROV Control Package

This package contains the control system for the Orca ROV.

## Overview

The Orca ROV control package provides:
- Thruster allocation for mapping desired forces/torques to individual thruster commands
- Depth control using pressure sensor data
- Heading control using IMU data
- Gripper control via MAVROS

## Nodes

### Thruster Allocator

The thruster allocator maps desired body wrench (force and torque) to individual thruster commands.

```bash
rosrun orca_control thruster_allocator
```

#### Subscribed Topics
- `desired_wrench` (geometry_msgs/Wrench): Desired body wrench

#### Published Topics
- `thruster_commands` (std_msgs/Float32MultiArray): Individual thruster commands

### Depth Controller

The depth controller maintains the ROV at a desired depth using a PID controller.

```bash
rosrun orca_control depth_controller
```

#### Subscribed Topics
- `pressure/data` (sensor_msgs/FluidPressure): Pressure sensor data
- `/mavros/altitude` (mavros_msgs/Altitude): Altitude data from MAVROS
- `depth_setpoint` (std_msgs/Float64): Desired depth

#### Published Topics
- `desired_wrench` (geometry_msgs/Wrench): Desired body wrench for depth control

### Heading Controller

The heading controller maintains the ROV at a desired heading using a PID controller.

```bash
rosrun orca_control heading_controller
```

#### Subscribed Topics
- `/mavros/imu/data` (sensor_msgs/Imu): IMU data from MAVROS
- `heading_setpoint` (std_msgs/Float64): Desired heading

#### Published Topics
- `desired_wrench` (geometry_msgs/Wrench): Desired body wrench for heading control

### Gripper Control

The gripper control node interfaces with the Newton grippers via MAVROS.

```bash
rosrun orca_control gripper_control.py
```

#### Subscribed Topics
- `left_gripper/command` (std_msgs/Bool): Command to open/close the left gripper
- `right_gripper/command` (std_msgs/Bool): Command to open/close the right gripper
- `left_gripper/position` (std_msgs/Float64): Position command for the left gripper (0.0-1.0)
- `right_gripper/position` (std_msgs/Float64): Position command for the right gripper (0.0-1.0)

#### Services Used
- `/mavros/cmd/command` (mavros_msgs/CommandLong): MAVROS command service

## Launch Files

### control.launch

Launches the complete control system.

```bash
roslaunch orca_control control.launch
```

## Configuration

The control system is configured via the `controllers.yaml` file in the `config` directory. This file includes:
- Thruster configuration (positions and orientations)
- PID gains for depth and heading controllers
- Gripper parameters
- Joint controller configuration

## Usage Examples

### Setting Depth Setpoint

```bash
rostopic pub /depth_setpoint std_msgs/Float64 "data: 1.0"
```

### Setting Heading Setpoint

```bash
rostopic pub /heading_setpoint std_msgs/Float64 "data: 0.0"
```

### Opening/Closing Grippers

```bash
# Open left gripper
rostopic pub /left_gripper/command std_msgs/Bool "data: true"

# Close left gripper
rostopic pub /left_gripper/command std_msgs/Bool "data: false"
```
