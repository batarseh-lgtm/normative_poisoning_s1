#!/bin/bash
# Startup script for S1 simulation environment

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"

# Set PX4 environment (adjust path if needed)
if [ -d "$HOME/PX4-Autopilot" ]; then
    export PX4_DIR="$HOME/PX4-Autopilot"
    export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PX4_DIR/Tools/sitl_gazebo/models"
    export GAZEBO_PLUGIN_PATH="$GAZEBO_PLUGIN_PATH:$PX4_DIR/build/px4_sitl_default/build_gazebo"
fi

echo "S1 Simulation environment ready!"
echo ""
echo "Available launch files:"
echo "  ros2 launch s1_attack_simulation baseline.launch.py"
echo "  ros2 launch s1_attack_simulation injection.launch.py"
echo "  ros2 launch s1_attack_simulation test.launch.py"
echo ""
