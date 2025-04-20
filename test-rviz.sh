#!/bin/bash
# This script should be run inside the Docker container
echo "Testing RViz visualization..."

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
fi

# Launch RViz with the Orca ROV model
roslaunch orca_description display.launch

echo "RViz test completed."
