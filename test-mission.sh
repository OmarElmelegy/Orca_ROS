#!/bin/bash
# This script should be run inside the Docker container
echo "Testing mission..."

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
fi

# Check which mission to run
if [ "$1" == "water" ]; then
    echo "Running water sampling mission..."
    roslaunch orca_bringup water_sampling_mission.launch
elif [ "$1" == "object" ]; then
    echo "Running object retrieval mission..."
    roslaunch orca_bringup object_retrieval_mission.launch
else
    echo "Please specify a mission: water or object"
    echo "Example: ./test-mission.sh water"
    exit 1
fi

echo "Mission test completed."
