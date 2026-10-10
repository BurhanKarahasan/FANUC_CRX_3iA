#!/usr/bin/env bash
# Reopen just the Cartesian jog panel against an already-running scene.
# Needs servo_node, so the scene must have been launched with launch_servo:=true.
set -e

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/install/local_setup.bash"

export QT_QPA_PLATFORM=xcb
exec ros2 run fanuc_scene_tools cartesian_jog_panel "$@"
