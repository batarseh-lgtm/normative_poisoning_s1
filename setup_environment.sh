#!/bin/bash
# Setup script for S1 Attack Simulation ROS 2 workspace

set -e

echo "====================================="
echo "S1 Attack Simulation - Workspace Setup"
echo "====================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$SCRIPT_DIR/ros2_ws"

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "Sourced ROS 2 Humble"
else
    echo "ERROR: ROS 2 Humble not found. Please install ROS 2 first."
    exit 1
fi

# Navigate to workspace
cd "$WORKSPACE_DIR"

# Install dependencies
echo ""
echo "Installing ROS 2 package dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo ""
echo "Building ROS 2 workspace..."
colcon build --symlink-install

# Source the workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "Workspace built successfully"
else
    echo "ERROR: Build failed"
    exit 1
fi

# Create startup script
STARTUP_SCRIPT="$SCRIPT_DIR/start_s1_simulation.sh"
cat > "$STARTUP_SCRIPT" << 'EOF'
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
EOF

chmod +x "$STARTUP_SCRIPT"

echo ""
echo "====================================="
echo "Setup Complete!"
echo "====================================="
echo ""
echo "To use the simulation environment:"
echo "  source $STARTUP_SCRIPT"
echo ""
echo "Or add this to your ~/.bashrc:"
echo "  source $STARTUP_SCRIPT"
echo ""
