#!/usr/bin/env bash
set -e

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"

export QT_QPA_PLATFORM=xcb
exec ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
  robot_model:=crx3ia \
  use_mock:=true \
  launch_rviz:=true