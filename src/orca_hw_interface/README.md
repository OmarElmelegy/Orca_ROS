# Orca ROV Hardware Interface Package

This package contains the hardware interface for the Orca ROV.

## Overview

The Orca ROV hardware interface package provides:
- Pressure sensor driver for the MS5837-30BA sensor
- Camera configuration and calibration
- MAVROS integration for the Pixhawk flight controller

## Nodes

### Pressure Sensor Node

The pressure sensor node interfaces with the MS5837-30BA pressure sensor via I2C.

```bash
rosrun orca_hw_interface pressure_sensor_node
```

#### Published Topics
- `pressure/data` (sensor_msgs/FluidPressure): Pressure sensor data
- `temperature/data` (sensor_msgs/Temperature): Temperature sensor data
- `depth` (std_msgs/Float64): Calculated depth based on pressure

#### Parameters
- `i2c_bus` (string, default: "/dev/i2c-1"): I2C bus for the sensor
- `i2c_address` (int, default: 0x76): I2C address of the sensor
- `fluid_density` (double, default: 1000.0): Fluid density in kg/m^3
- `update_rate` (double, default: 10.0): Sensor update rate in Hz

### Camera Configuration Node

The camera configuration node manages camera calibration and configuration.

```bash
rosrun orca_hw_interface camera_config.py
```

#### Published Topics
- `/{camera_name}/camera_info` (sensor_msgs/CameraInfo): Camera calibration information

#### Parameters
- `camera_names` (list): List of camera names
- `camera_urls` (dict): Dictionary of camera calibration file URLs
- `camera_frames` (dict): Dictionary of camera frame IDs

## Launch Files

### hardware.launch

Launches the complete hardware interface.

```bash
roslaunch orca_hw_interface hardware.launch
```

This launch file starts:
- Pressure sensor node
- Camera configuration node
- MAVROS for Pixhawk communication
- RealSense camera driver
- USB cameras (front and rear)

## Configuration

The hardware interface is configured via the `hardware.yaml` file in the `config` directory. This file includes:
- Pressure sensor parameters
- Camera configuration
- MAVROS parameters

### Camera Calibration

Camera calibration files are stored in the `config/calibration` directory:
- `front_camera.yaml`: Calibration for the front camera
- `rear_camera.yaml`: Calibration for the rear camera
- `realsense.yaml`: Calibration for the RealSense camera

## Hardware Setup

### Pressure Sensor

The MS5837-30BA pressure sensor is connected to the Raspberry Pi via I2C:
- Connect VCC to 3.3V
- Connect GND to GND
- Connect SDA to SDA (GPIO 2)
- Connect SCL to SCL (GPIO 3)

### Cameras

- The RealSense D415 camera is connected via USB
- The front and rear cameras are connected via USB

### Pixhawk

The Pixhawk 2.4.8 is connected to the Raspberry Pi via USB:
- Connect the Pixhawk's TELEM2 port to the Raspberry Pi's USB port
- Configure the Pixhawk to use the ArduSub firmware

## Troubleshooting

### I2C Issues

If the pressure sensor is not detected:
```bash
# Check if the I2C device is visible
i2cdetect -y 1
```

### Camera Issues

If cameras are not detected:
```bash
# List available video devices
ls -l /dev/video*
```

### MAVROS Issues

If MAVROS is not connecting to the Pixhawk:
```bash
# Check the serial port
ls -l /dev/ttyACM*
# Check MAVROS status
rostopic echo /mavros/state
```
