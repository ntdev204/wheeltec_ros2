#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/wheeltec_ros2}"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

exec ros2 launch turn_on_wheeltec_robot prod_bringup.launch.py
