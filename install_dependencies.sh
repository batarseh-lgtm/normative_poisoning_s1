#!/bin/bash
# Installation script for S1 Attack Simulation dependencies
# This script installs ROS 2, Gazebo, PX4, and MAVROS

set -e

echo "====================================="
echo "S1 Attack Simulation - Dependency Installer"
echo "====================================="

# Detect OS
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$ID
    VER=$VERSION_ID
else
    echo "Cannot detect OS version"
    exit 1
fi

echo "Detected OS: $OS $VER"

# Check for Ubuntu 22.04 (recommended)
if [ "$OS" != "ubuntu" ] || [ "$VER" != "22.04" ]; then
    echo "WARNING: This script is designed for Ubuntu 22.04"
    echo "Your system: $OS $VER"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Install ROS 2 Humble
echo ""
echo "Step 1: Installing ROS 2 Humble..."
if command_exists ros2; then
    echo "ROS 2 already installed"
else
    echo "Installing ROS 2 Humble..."
    
    # Setup sources
    sudo apt update
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS 2 packages
    sudo apt update
    sudo apt install -y ros-humble-desktop
    sudo apt install -y ros-humble-ros-base
    sudo apt install -y ros-dev-tools
    
    echo "ROS 2 Humble installed"
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install MAVROS
echo ""
echo "Step 2: Installing MAVROS..."
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets (required by MAVROS)
echo "Installing GeographicLib datasets..."
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# Install Gazebo
echo ""
echo "Step 3: Installing Gazebo..."
if command_exists gazebo; then
    echo "Gazebo already installed"
else
    sudo apt install -y gazebo
    sudo apt install -y libgazebo-dev
    sudo apt install -y ros-humble-gazebo-ros-pkgs
fi

# Install PX4 Dependencies
echo ""
echo "Step 4: Installing PX4 build dependencies..."
sudo apt install -y \
    git \
    make \
    cmake \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    ninja-build \
    ccache \
    astyle \
    libgtest-dev

# Python dependencies
pip3 install --user \
    pytest \
    pytest-cov \
    numpy \
    toml \
    jinja2 \
    empy \
    packaging \
    jsonschema \
    pyros-genmsg

echo ""
echo "Step 5: PX4 Autopilot setup..."
echo "Note: PX4 should be cloned and built separately"
echo "To install PX4:"
echo "  cd ~"
echo "  git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
echo "  cd PX4-Autopilot"
echo "  bash ./Tools/setup/ubuntu.sh"
echo "  make px4_sitl_default gazebo"

# Additional Python packages for S1 simulation
echo ""
echo "Step 6: Installing Python dependencies for S1 simulation..."
pip3 install --user \
    pyyaml \
    numpy

echo ""
echo "====================================="
echo "Installation Complete!"
echo "====================================="
echo ""
echo "Next steps:"
echo "1. Install PX4 Autopilot (see instructions above)"
echo "2. Run ./setup_environment.sh to build the ROS 2 workspace"
echo "3. Follow the README for running the simulation"
echo ""
echo "IMPORTANT: Close and reopen your terminal, or run:"
echo "  source ~/.bashrc"
