#!/bin/bash
# =============================================================================
# ADAS Quick Start Script
# 快速启动所有ADAS相关组件
# =============================================================================

# 配置
CARLA_ROOT="${CARLA_ROOT:-$HOME/CARLA}"
CARLA_ROS_BRIDGE_WS="$HOME/carla-ros-bridge/catkin_ws"
ADAS_WS="$HOME/AutoPilot/catkin_ws"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ADAS Quick Start Script${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查CARLA_ROOT
if [ ! -d "$CARLA_ROOT" ]; then
    echo -e "${RED}Error: CARLA_ROOT not found at $CARLA_ROOT${NC}"
    echo "Please set CARLA_ROOT environment variable"
    exit 1
fi

# 检查工作空间
if [ ! -f "$ADAS_WS/devel/setup.bash" ]; then
    echo -e "${RED}Error: ADAS workspace not built${NC}"
    echo "Please run: cd $ADAS_WS && catkin_make"
    exit 1
fi

# 检测终端模拟器
if command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
elif command -v konsole &> /dev/null; then
    TERMINAL="konsole"
elif command -v xterm &> /dev/null; then
    TERMINAL="xterm"
else
    echo -e "${RED}Error: No supported terminal emulator found${NC}"
    echo "Please install gnome-terminal, konsole, or xterm"
    exit 1
fi

echo -e "${YELLOW}Using terminal: $TERMINAL${NC}"
echo ""

# 启动函数
start_terminal() {
    local title="$1"
    local cmd="$2"
    local delay="${3:-0}"
    
    echo -e "${GREEN}[Starting]${NC} $title"
    
    if [ "$TERMINAL" = "gnome-terminal" ]; then
        gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" &
    elif [ "$TERMINAL" = "konsole" ]; then
        konsole --new-tab -p tabtitle="$title" -e bash -c "$cmd; exec bash" &
    else
        xterm -title "$title" -e bash -c "$cmd; exec bash" &
    fi
    
    sleep "$delay"
}

# =============================================================================
# 1. CARLA Server
# =============================================================================
start_terminal "CARLA Server" \
    "cd $CARLA_ROOT && echo '>>> Starting CARLA Server...' && ./CarlaUE4.sh -quality-level=Low" \
    5

# =============================================================================
# 2. CARLA-ROS Bridge
# =============================================================================
start_terminal "ROS Bridge" \
    "source /opt/ros/noetic/setup.bash && \
     source $CARLA_ROS_BRIDGE_WS/devel/setup.bash && \
     echo '>>> Waiting for CARLA...' && sleep 5 && \
     echo '>>> Starting ROS Bridge...' && \
     roslaunch carla_ros_bridge carla_ros_bridge.launch" \
    3

# =============================================================================
# 3. Spawn Ego Vehicle
# =============================================================================
start_terminal "Spawn Vehicle" \
    "source /opt/ros/noetic/setup.bash && \
     source $CARLA_ROS_BRIDGE_WS/devel/setup.bash && \
     echo '>>> Waiting for ROS Bridge...' && sleep 8 && \
     echo '>>> Spawning ego vehicle...' && \
     roslaunch carla_spawn_objects carla_spawn_objects.launch" \
    3

# =============================================================================
# 4. ADAS System
# =============================================================================
start_terminal "ADAS System" \
    "source /opt/ros/noetic/setup.bash && \
     source $ADAS_WS/devel/setup.bash && \
     echo '>>> Waiting for vehicle spawn...' && sleep 12 && \
     echo '>>> Starting ADAS System...' && \
     roslaunch adas_bringup adas_full_sim.launch" \
    3

# =============================================================================
# 5. Manual Control
# =============================================================================
start_terminal "Manual Control" \
    "source /opt/ros/noetic/setup.bash && \
     source $CARLA_ROS_BRIDGE_WS/devel/setup.bash && \
     echo '>>> Waiting for ADAS...' && sleep 15 && \
     echo '>>> Starting Manual Control...' && \
     roslaunch carla_manual_control carla_manual_control.launch" \
    0

# =============================================================================
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  All components started!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "Terminals launched:"
echo -e "  1. ${YELLOW}CARLA Server${NC}     - CARLA simulator"
echo -e "  2. ${YELLOW}ROS Bridge${NC}       - CARLA-ROS communication"
echo -e "  3. ${YELLOW}Spawn Vehicle${NC}    - Ego vehicle spawner"
echo -e "  4. ${YELLOW}ADAS System${NC}      - AEB and perception"
echo -e "  5. ${YELLOW}Manual Control${NC}   - Keyboard control (WASD)"
echo ""
echo -e "${YELLOW}Tips:${NC}"
echo -e "  - Use ${GREEN}WASD${NC} keys in Manual Control window to drive"
echo -e "  - Press ${GREEN}B${NC} to toggle manual override"
echo -e "  - AEB will automatically brake when obstacles detected"
echo ""
