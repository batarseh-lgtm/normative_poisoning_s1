#!/bin/bash
# Simplified Gazebo S1 Attack Execution Script
# Runs baseline, injection, and test phases automatically

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PX4_DIR="${HOME}/PX4-Autopilot"
HEADLESS="${HEADLESS:-true}"  # Default to headless mode
QUICK_TEST="${1:-false}"

echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  S1 Attack Gazebo Simulation Runner   ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
echo ""

# Check if PX4 exists
if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}ERROR: PX4-Autopilot not found at $PX4_DIR${NC}"
    echo "Please clone PX4 first:"
    echo "  cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
    exit 1
fi

# Function to cleanup background processes
cleanup() {
    echo -e "\n${YELLOW}Cleaning up background processes...${NC}"
    pkill -f px4 || true
    pkill -f gazebo || true
    sleep 2
}

trap cleanup EXIT

# Source ROS 2
echo -e "${YELLOW}[1/6] Setting up ROS 2 environment...${NC}"
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/ros2_ws/install/setup.bash"

# Build PX4 if needed
echo -e "${YELLOW}[2/6] Checking PX4 build...${NC}"
if [ ! -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]; then
    echo "Building PX4 SITL..."
    cd "$PX4_DIR"
    DONT_RUN=1 make px4_sitl gazebo-classic
else
    echo "PX4 already built ✓"
fi

# Set PX4 environment
export PX4_DIR
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:${PX4_DIR}/build/px4_sitl_default/build_gazebo-classic"

# For headless mode
if [ "$HEADLESS" = "true" ]; then
    echo -e "${YELLOW}Running in HEADLESS mode (no GUI)${NC}"
    export HEADLESS=1
fi

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}  Phase 1: BASELINE (Normal Behavior)   ${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""

# Just run the Python scripts directly without Gazebo
echo -e "${YELLOW}[3/6] Running baseline phase...${NC}"
cd "${SCRIPT_DIR}"
python3 scripts/run_baseline.py

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}  Phase 2: INJECTION (Norm Poisoning)   ${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""

echo -e "${YELLOW}[4/6] Running injection phase...${NC}"
python3 scripts/run_injection.py

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}  Phase 3: TEST (Attack Verification)   ${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""

echo -e "${YELLOW}[5/6] Running test phase...${NC}"
python3 scripts/run_test.py

echo ""
echo -e "${YELLOW}[6/6] Generating results report...${NC}"
python3 generate_report.py

echo ""
echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║         ATTACK COMPLETED ✓             ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
echo ""
echo -e "Results:"
echo -e "  📊 Report: ${GREEN}REPORT.md${NC}"
echo -e "  📁 Data:   ${GREEN}data/${NC}"
echo ""

# Display summary
if [ -f "REPORT.md" ]; then
    echo -e "${YELLOW}Attack Summary:${NC}"
    grep -A 2 "Sector B Scanned" REPORT.md || true
fi

echo ""
echo -e "${GREEN}Attack execution complete!${NC}"
