#!/bin/bash
# Bridge ROS 2 <-> Gazebo per single drone (tethys).
# Usa config YAML per mappare topic ROS2 validi ai topic Ignition reali.
# Le pinne JPC in Fortress usano /joint/<name>/0/cmd_pos (non bridgiabile direttamente
# con parameter_bridge perché "0" è token numerico, illegale nei topic ROS2).
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$SCRIPT_DIR/ponte_single_config.yaml"

echo "Avvio bridge single-drone (config: $CONFIG)..."

ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="$CONFIG"
