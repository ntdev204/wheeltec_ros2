#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/wheeltec_ros2}"

# setup.bash của ROS có thể tham chiếu biến chưa set
set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u

exec ros2 launch turn_on_wheeltec_robot prod_bringup.launch.py
