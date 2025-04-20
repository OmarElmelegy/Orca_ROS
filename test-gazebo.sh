#!/bin/bash
# This script should be run inside the Docker container
echo "Testing Gazebo simulation..."

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
fi

# Launch Gazebo with the Orca ROV model
roslaunch orca_bringup orca_simulation.launch

echo "Gazebo test completed."
