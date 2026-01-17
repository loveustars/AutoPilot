#!/bin/bash
# Launch CARLA simulator
# 启动CARLA仿真器

CARLA_ROOT=${CARLA_ROOT:-~/CARLA}
QUALITY=${1:-Low}

echo "Starting CARLA with quality: $QUALITY"
echo "CARLA_ROOT: $CARLA_ROOT"

cd $CARLA_ROOT
./CarlaUE4.sh -quality-level=$QUALITY -ResX=800 -ResY=600
