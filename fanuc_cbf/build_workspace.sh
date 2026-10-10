#!/usr/bin/env bash
set -e

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
set -u

colcon build --symlink-install --event-handlers console_direct+ \
  --cmake-args \
  "-DFETCHCONTENT_SOURCE_DIR_SOCKPP=${workspace_dir}/src/fanuc_driver/fanuc_libs/dependencies/sockpp" \
  "-DFETCHCONTENT_SOURCE_DIR_READERWRITERQUEUE=${workspace_dir}/src/fanuc_driver/fanuc_libs/dependencies/readerwriterqueue" \
  "-DFETCHCONTENT_SOURCE_DIR_YAML-CPP=${workspace_dir}/src/fanuc_driver/fanuc_libs/dependencies/yaml-cpp" \
  "-DFETCHCONTENT_SOURCE_DIR_REFLECT-CPP=${workspace_dir}/src/fanuc_driver/fanuc_libs/dependencies/reflect-cpp"
