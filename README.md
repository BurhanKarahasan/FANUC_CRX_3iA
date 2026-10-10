# FANUC CRX ROS 2 Workspace Archive

This repository collects the FANUC CRX ROS 2 development work currently stored
under one workspace. It includes the official driver and robot-description
source, collision-based velocity filtering, MoveIt Servo integration,
position/trajectory controllers, reachability tools, and historical workspace
snapshots retained for comparison.

## Repository map

| Folder | Role | Status |
| --- | --- | --- |
| `fanuc_cbf/` | Active ROS 2 Jazzy workspace: official driver/description plus CBF, Servo, scene tools, panels, controllers, and reachability tools | **Primary active development area** |
| `fanuc_crx_ros2/` | FANUC ROS 2 driver, hardware interface, robot descriptions, controllers, messages, and MoveIt configuration | **Driver foundation / upstream-derived source** |
| `official_fanuc_ws/` | Parallel source snapshot for historical comparison | **Older or reference iteration; compare before editing** |
| `killer.sh` | Convenience script for stopping running ROS/Gazebo/RViz processes | Workspace utility |
| `docs/` | Orientation and history notes for working with this aggregate repository | Documentation |

The generated `build/`, `install/`, and `log/` directories are intentionally
ignored. They can be recreated with `colcon build` and are not source history.
The original nested Git metadata is also excluded so this repository contains
the files from each snapshot as ordinary folders rather than nested Git links.

## What is where

### `fanuc_cbf`

This is the best starting point for the latest experimental work. Its source
packages are under `src/`:

- `fanuc_driver`: driver sources used by the workspace.
- `fanuc_description`: robot description and model assets.
- `fanuc_scene_tools`: collision scene publishing, CBF velocity filtering,
  Cartesian goal/jog panels, launch files, RViz configurations, and analysis
  utilities.

The `run_*.sh` scripts and `build_workspace.sh` are the quick entry points for
the CRX-3iA simulation, panels, and reproducible build. Generated analysis
results belong under an experiment-specific directory and are not kept at the
workspace root.

### `fanuc_crx_ros2`

This folder contains the broader ROS 2 driver stack:

- `crx_description`: CRX robot meshes, URDF/Xacro files, and Gazebo/RViz launch
  support.
- `fanuc_hardware_interface`: mock and physical hardware interfaces plus robot
  descriptions.
- `fanuc_controllers`, `fanuc_forward_command`, and `slider_publisher`:
  controller and command examples.
- `fanuc_moveit_config`: MoveIt planning configuration.
- `fanuc_msgs`: custom ROS 2 messages.
- `fanuc_libs`: third-party and supporting libraries.

This folder originated as a separate repository. In the aggregate repository it
is preserved as a source snapshot; its previous GitHub remote should not be
used for publishing this combined workspace.

### `official_fanuc_ws`

This is a second workspace snapshot containing the driver/description source
under `src/`. Its duplicated root-level analysis artifacts and run wrappers
were removed; use `fanuc_cbf` for current builds and launch scripts. Treat
this tree as reference material when tracing how an experiment evolved. Do not
assume changes made in `fanuc_cbf` are present here, or vice versa.

The detailed comparison guidance is in [`docs/REPOSITORY_MAP.md`](docs/REPOSITORY_MAP.md).

## Prerequisites

The primary runtime targets are Linux, ROS 2, MoveIt 2, and a Python 3
environment with the dependencies required by the selected ROS packages. A
real FANUC controller is not required for the mock simulation workflows, but
physical control requires the appropriate robot network configuration and
hardware permissions.

## Build and run

Source the ROS 2 installation, then build from the workspace that contains the
packages you want to run. For the CBF workspace, for example:

```bash
cd fanuc_cbf
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./build_workspace.sh
source install/setup.bash
```

The driver-only source under `fanuc_crx_ros2` can be built independently:

```bash
cd fanuc_crx_ros2
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build only one workspace at a time when diagnosing package or overlay issues.
Avoid sourcing several generated `install/setup.bash` files in an unknown order.

## Common launch commands

From `fanuc_cbf` after building and sourcing its install space:

```bash
./run_crx3ia_sim.sh
./run_crx3ia_collision_scene.sh
./run_goal_panel.sh
./run_jog_panel.sh
```

The complete package, dependency, controller, collision-avoidance, and
analysis-tool reference is in
[`docs/WORKSPACE_GUIDE.md`](docs/WORKSPACE_GUIDE.md). The collision scene launch
file exposes parameters for workspace meshes, TCP offsets, CBF filtering,
Servo, panels, and orthographic RViz views.

## Safety

The mock workflows are suitable for development, but physical robot control can
move industrial machinery. Validate limits, frames, collision geometry, network
configuration, and emergency-stop operation before connecting to a real robot.
Do not treat reachability plots or simulation output as a substitute for a
commissioning review.

## Licensing and provenance

The driver source includes its own licensing files and provenance notices under
`fanuc_crx_ros2/LICENSES/` and in individual source files. Review those notices
before redistributing the aggregate repository. This README documents the
workspace organization; it does not replace the licenses shipped with the
original components.
