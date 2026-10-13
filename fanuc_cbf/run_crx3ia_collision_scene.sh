#!/usr/bin/env bash
set -e

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/install/local_setup.bash"

export QT_QPA_PLATFORM=xcb
exec ros2 launch fanuc_custom_tools crx3ia_collision_scene.launch.py "$@"