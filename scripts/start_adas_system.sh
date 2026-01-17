#!/bin/bash
# =============================================================================
# ADAS System Launch Script
# 启动ADAS系统的统一脚本
#
# This script properly sources both workspaces and starts the ADAS nodes
# =============================================================================

set -e

echo "=========================================="
echo "  ADAS System Startup"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
CARLA_ROS_BRIDGE_WS="${CARLA_ROS_BRIDGE_WS:-$HOME/carla-ros-bridge/catkin_ws}"
ADAS_WS="$HOME/AutoPilot/catkin_ws"

# Step 1: Source ROS
echo -e "${YELLOW}[1/3] Sourcing ROS Noetic...${NC}"
source /opt/ros/noetic/setup.bash

# Step 2: Source ADAS workspace (now overlays carla-ros-bridge automatically)
echo -e "${YELLOW}[2/3] Sourcing ADAS workspace (includes carla-ros-bridge)...${NC}"
if [ -f "$ADAS_WS/devel/setup.bash" ]; then
    source "$ADAS_WS/devel/setup.bash"
    echo -e "${GREEN}  ✓ ADAS workspace found${NC}"
else
    echo -e "${RED}  ✗ ADAS workspace NOT found at: $ADAS_WS${NC}"
    echo -e "${RED}    Please run: cd $ADAS_WS && catkin_make${NC}"
    exit 1
fi

# Step 3: Verify critical packages
echo -e "${YELLOW}[3/3] Verifying packages...${NC}"

# Check carla_msgs
python3 -c "import carla_msgs.msg" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ carla_msgs available${NC}"
else
    echo -e "${RED}  ✗ carla_msgs NOT available${NC}"
    echo -e "${RED}    Ensure carla-ros-bridge is built correctly${NC}"
    exit 1
fi

# Check adas_msgs
python3 -c "import adas_msgs.msg" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ adas_msgs available${NC}"
else
    echo -e "${RED}  ✗ adas_msgs NOT available${NC}"
    echo -e "${RED}    Run: cd $ADAS_WS && catkin_make${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}=========================================="
echo "  Environment ready!"
echo "==========================================${NC}"
echo ""

# Parse arguments
MODE=${1:-full}

case $MODE in
    "full")
        echo "Starting FULL ADAS system..."
        roslaunch adas_bringup simulation.launch
        ;;
    "perception")
        echo "Starting PERCEPTION only..."
        roslaunch adas_bringup simulation.launch \
            enable_decision:=false \
            enable_control:=false \
            enable_safety:=false
        ;;
    "test")
        echo "Starting minimal TEST setup..."
        roslaunch adas_bringup simulation.launch \
            camera_enabled:=false
        ;;
    "shell")
        echo "Environment set up. Starting interactive shell..."
        exec bash
        ;;
    *)
        echo "Usage: $0 [full|perception|test|shell]"
        echo "  full       - Start complete ADAS system"
        echo "  perception - Start perception modules only"
        echo "  test       - Start minimal test configuration"
        echo "  shell      - Set up environment and drop to shell"
        exit 1
        ;;
esac
