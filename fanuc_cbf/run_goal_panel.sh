#!/usr/bin/env bash
# Reopen just the Cartesian goal panel against an already-running scene.
# For the full scene, use run_crx3ia_collision_scene.sh with launch_goal_panel:=true.
set -e

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/install/local_setup.bash"

export QT_QPA_PLATFORM=xcb
exec ros2 run fanuc_scene_tools cartesian_goal_panel "$@"
