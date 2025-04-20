#!/bin/bash
# This script should be run inside the Docker container
echo "Testing all components of the Orca ROV system..."

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
fi

# Test 1: Check if ROS is working
echo
echo "Test 1: Checking if ROS is working..."
roscore & sleep 5 && killall -9 roscore
if [ $? -ne 0 ]; then
    echo "FAILED: ROS core could not start"
else
    echo "PASSED: ROS core started successfully"
fi

# Test 2: Check if the URDF model can be parsed
echo
echo "Test 2: Checking if the URDF model can be parsed..."
check_urdf src/orca_description/urdf/orca.urdf.xacro
if [ $? -ne 0 ]; then
    echo "FAILED: URDF model has errors"
else
    echo "PASSED: URDF model parsed successfully"
fi

# Test 3: Check if the launch files are valid
echo
echo "Test 3: Checking if launch files are valid..."
roslaunch --nodes orca_bringup orca_core.launch
if [ $? -ne 0 ]; then
    echo "FAILED: Launch file has errors"
else
    echo "PASSED: Launch file is valid"
fi

# Test 4: Check if the control system can be launched
echo
echo "Test 4: Checking if the control system can be launched..."
roslaunch orca_control control.launch --dry-run
if [ $? -ne 0 ]; then
    echo "FAILED: Control system launch has errors"
else
    echo "PASSED: Control system launch is valid"
fi

# Test 5: Check if the perception system can be launched
echo
echo "Test 5: Checking if the perception system can be launched..."
roslaunch orca_perception perception.launch --dry-run
if [ $? -ne 0 ]; then
    echo "FAILED: Perception system launch has errors"
else
    echo "PASSED: Perception system launch is valid"
fi

echo
echo "All tests completed."
echo "To run the full system, use:"
echo "roslaunch orca_bringup orca_complete.launch"
echo
echo "To visualize the ROV in RViz, use:"
echo "roslaunch orca_description display.launch"
echo
echo "To run the ROV in Gazebo simulation, use:"
echo "roslaunch orca_bringup orca_simulation.launch"
