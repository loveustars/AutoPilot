#!/bin/bash
# Run OpenSCENARIO test scenario
# 运行OpenSCENARIO测试场景

SCENARIO_FILE=${1:-"lead_vehicle_sudden_stop.xosc"}
SCENARIO_DIR="$(dirname "$0")/../scenarios"

echo "Running scenario: $SCENARIO_FILE"

# Source ROS workspace
source ~/AutoPilot/catkin_ws/devel/setup.bash

# Run scenario runner (requires scenario_runner from CARLA)
python3 ${CARLA_ROOT}/PythonAPI/examples/scenario_runner.py \
    --scenario $SCENARIO_DIR/$SCENARIO_FILE \
    --reloadWorld
