# FANUC CRX-3iA workspace guide

This guide describes the supported build and run workflow for the active
workspace. It intentionally separates source code from generated reachability
and placement-study output.

## Workspace layout

- [`fanuc_cbf/`](../fanuc_cbf/) is the active ROS 2 Jazzy workspace. Use its
  `src/` directory, build it there, and run the CRX-3iA examples from there.
- [`fanuc_crx_ros2/`](../fanuc_crx_ros2/) is the earlier driver foundation
  snapshot. It is useful for historical comparison, but is not the active
  build root.
- [`official_fanuc_ws/`](../official_fanuc_ws/) is a retained reference
  snapshot. Do not build it together with `fanuc_cbf`; duplicate overlays can
  produce ambiguous package resolution.
- `build/`, `install/`, and `log/` are local colcon products and are ignored.
  Analysis CSV/SVG/HTML files at workspace roots are also ignored and should
  be generated into a separate experiment directory.

The active source tree combines the official FANUC repositories:

- `src/fanuc_driver/` contains the driver, controllers, MoveIt configuration,
  messages, RMI examples, and supporting libraries.
- `src/fanuc_description/` contains the official robot descriptions and meshes.
- `src/fanuc_scene_tools/` contains the collision-scene, CBF, Servo, panel, and
  reachability tools added by this workspace.

## Prerequisites

The supported baseline is Ubuntu 24.04 (Noble), ROS 2 Jazzy Jalisco, and a
working C++17 toolchain. Install the ROS desktop and build tooling:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool
```

Install package dependencies from the active workspace:

```bash
cd ~/FANUC_CRX_3iA/fanuc_cbf
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

The mock launch uses MoveIt 2, ros2_control, RViz, Xacro, and the controller
plugins. If rosdep reports a missing force/torque broadcaster, install it with:

```bash
sudo apt install -y ros-jazzy-force-torque-sensor-broadcaster
```

## Compile

Use the checked-in build script. The local FANUC archive contains the pinned
`sockpp`, `readerwriterqueue`, `yaml-cpp`, and `reflect-cpp` sources, so the
script supplies their paths to CMake and avoids an invalid network fetch:

```bash
cd ~/FANUC_CRX_3iA/fanuc_cbf
./build_workspace.sh
source install/setup.bash
```

The build currently contains 22 packages, including the official descriptions,
driver, custom messages, MoveIt configuration, controllers, examples, and
`fanuc_scene_tools`.

## Run the examples

All commands below assume that `source install/setup.bash` has been run.

### Mock MoveIt and RViz

```bash
cd ~/FANUC_CRX_3iA/fanuc_cbf
./run_crx3ia_sim.sh
```

This launches the CRX-3iA model with the mock hardware interface, MoveIt
planning, ros2_control, and RViz. It does not connect to a physical robot.

### Collision scene, CBF, and Servo

```bash
./run_crx3ia_collision_scene.sh
```

Useful launch arguments include:

```bash
./run_crx3ia_collision_scene.sh \
  launch_servo:=true \
  launch_goal_panel:=true \
  launch_jog_panel:=true
```

The collision scene publisher adds workspace and stepper-motor geometry to the
MoveIt planning scene. `cbf_velocity_filter.py` filters Cartesian velocity
commands against collision constraints before forwarding them to Servo.

### Cartesian panels

Run these against an already-running collision scene:

```bash
./run_goal_panel.sh
./run_jog_panel.sh
```

The goal panel sends six-degree-of-freedom Cartesian goals through MoveIt and
checks inverse-kinematics and collision validity. The jog panel publishes
Cartesian jog commands through MoveIt Servo and reports Servo status.

## Launch configuration and registration

The launch files also select the robot model, hardware mode, controller, GPIO
mapping, frames, TCP geometry, collision policy, and plugin registrations.
Change these values deliberately and keep the related URDF, SRDF, YAML, and
controller settings consistent.

### Main MoveIt launch arguments

`fanuc_moveit.launch.py` accepts:

| Argument | Default | Purpose |
| --- | --- | --- |
| `robot_model` | required; use `crx3ia` here | Selects the URDF/Xacro and SRDF. Choices include `crx3ia`, `crx5ia`, `crx10ia`, `crx10ia_l`, `crx20ia_l`, and `crx30ia`. |
| `use_mock` | `false` | Selects mock control instead of physical control. |
| `robot_ip` | `192.168.1.100` | Physical controller address; not used by mock control. |
| `ros2_control_config` | `fanuc_hardware_interface/config/ros2_controllers.yaml` | Controller-manager YAML file. |
| `gpio_config_package` / `gpio_config_path` | `fanuc_hardware_interface` / `config/example_gpio_config.yaml` | GPIO register mapping. |
| `motion_control` | `1` | Initial FANUC motion-control state for physical operation. |
| `initial_controller` | `joint_trajectory_controller` | Activates the trajectory controller, RMI controller, or none. |
| `encoding` | `UTF-8` | Physical controller encoding; use `SHIFT-JIS` for the documented Japanese R-30iB+ case. |
| `launch_rviz` | `true` | Starts RViz with generated robot, semantic, kinematics, limits, and pipeline parameters. |

The launch selects the model Xacro and SRDF, loads `kinematics.yaml`,
`moveit_controllers.yaml`, and the OMPL pipeline, then starts `move_group`.
The standard virtual joint is `world` to `base_link`, and the planning group is
`manipulator`.

Example:

```bash
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
  robot_model:=crx3ia use_mock:=true launch_rviz:=true \
  initial_controller:=joint_trajectory_controller
```

### Collision-scene and Servo arguments

`crx3ia_collision_scene.launch.py` starts the CRX-3iA mock MoveIt launch and a
`collision_scene_publisher`. Its configuration includes:

- Scene geometry: `mesh_scale` defaults to `0.001`; `workspace_x/y/z` defaults to
  `0.65/0.0/0.0`; and workspace roll/pitch/yaw default to zero.
- Optional tool geometry: `load_stepper:=true` loads `stepper_motor.stl`;
  `stepper_x/y/z`, `stepper_roll/pitch/yaw`, and `stepper_link` position it.
- Collision policy: `workspace_collision:=true` controls whether the workspace
  object is an active collision object.
- Goal configuration: `launch_goal_panel`, `goal_group` (`manipulator`),
  `goal_frame` (`world`), `goal_tcp_link` (`tool_tcp`), and
  `tcp_parent_link` (`fanuc_flange`).
- TCP transform: `tcp_x/y/z` and `tcp_roll/pitch/yaw` define the static
  transform from `tcp_parent_link` to `goal_tcp_link`.
- Servo and filtering: `launch_servo`, `launch_cbf` (default `true` when Servo
  is enabled), `cbf_prediction_horizon` (`0.20` seconds), and
  `servo_ee_link` (`flange`).
- Visualization: `launch_ortho_views:=true` starts XY, XZ, and YZ RViz views
  with helper TF frames.

Examples:

```bash
./run_crx3ia_collision_scene.sh launch_goal_panel:=true
./run_crx3ia_collision_scene.sh launch_servo:=true launch_cbf:=true \
  tcp_y:=0.11
./run_crx3ia_collision_scene.sh workspace_collision:=false
```

The scene wires `/cbf_velocity_filter/input` through the CBF filter to
`/servo_node/delta_twist_cmds`. If CBF is disabled, the jog panel publishes
directly to Servo. CBF parameters include `planning_frame`, `tcp_link`,
`tcp_parent_link`, `tcp_offset`, `group_name`, and `prediction_horizon`; these
must agree with the goal/TCP launch arguments.

### Controller-manager registrations

`fanuc_hardware_interface/config/ros2_controllers.yaml` registers these names
at a 500 Hz update rate:

| Controller name | Plugin type | Function |
| --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes joint state. |
| `joint_trajectory_controller` | `fanuc_controllers/ScaledJointTrajectoryController` | Six-joint position trajectory control. |
| `forward_position_controller` | `forward_command_controller/ForwardCommandController` | Direct six-joint position commands. |
| `fanuc_gpio_controller` | `fanuc_controllers/FanucGPIOController` | FANUC GPIO/register mapping. |
| `fanuc_force_sensor_broadcaster` | `fanuc_controllers/FanucForceSensorBroadcaster` | FANUC force sensor data. |
| `force_torque_sensor_broadcaster` | `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster` | Standard ros2_control FT broadcaster. |
| `fanuc_rmi_controller` | `fanuc_controllers/FanucRMIController` | FANUC RMI instruction/control path. |

The trajectory controller registers `position` command/state interfaces for
`J1` through `J6`. Servo emits `trajectory_msgs/JointTrajectory`. Custom
plugins are exported through the plugin XML files in `fanuc_controllers/`;
the hardware plugin is exported through
`fanuc_hardware_interface/hardware_interface_plugin.xml`, and MoveIt’s
trajectory plugin through
`fanuc_moveit_config/fanuc_controllers_moveit_trajectory_plugin.xml`.

If controller loading fails, verify the package overlay, controller name,
spawner name, and plugin XML registration. The standard FT broadcaster also
requires `ros-jazzy-force-torque-sensor-broadcaster`.

### Servo parameter registration

`fanuc_moveit_config/config/servo.yaml` configures `speed_units` input, a
10 ms publish period, 50% default velocity scaling, and
`trajectory_msgs/JointTrajectory` output. It uses the `manipulator` group,
publishes status at `~/status`, consumes `~/delta_twist_cmds` and
`~/delta_joint_cmds`, reads `/joint_states`, and outputs to
`/servo_controller/joint_trajectory`. Collision checking runs at 10 Hz with
0.006 m self-collision and 0.02 m scene-collision proximity thresholds.
Singularity thresholds are 30.0 for deceleration and 50.0 for hard stop.

The collision launch uses `config/servo_crx3ia.yaml` and supplies robot
description, semantic description, and kinematics parameters at runtime.

### Physical versus mock registration

`use_mock:=true` includes `fanuc_mock_control.launch.py`; `use_mock:=false`
includes `fanuc_physical_control.launch.py` and passes `robot_ip`,
`robot_model`, `ros2_control_config`, GPIO settings, `motion_control`,
`initial_controller`, and `encoding`. The physical path requires the FANUC
network/controller configuration and must not be tested without validated
limits and an emergency-stop procedure.

## Capabilities in the source tree

### Motion and hardware

- `fanuc_hardware_interface` provides the mock and physical ros2_control
  hardware interfaces.
- `fanuc_controllers` provides the scaled joint-trajectory controller, RMI
  controller, GPIO controller, and FANUC force-sensor broadcaster plugins.
- `fanuc_forward_command` contains a sine-wave forward-command example.
- The controller configuration supports position/joint-trajectory operation;
  Servo and the CBF filter provide velocity-based Cartesian operation.
- `fanuc_moveit_config` provides MoveIt planning, OMPL, controller-manager
  integration, Servo configuration, joint limits, and RViz launch support.

### Collision avoidance and scene interaction

- `collision_scene_publisher.py` publishes workspace and tool collision objects.
- `cbf_velocity_filter.py` applies control-barrier-function velocity filtering
  using MoveIt state validity and inverse-kinematics services.
- `cartesian_goal_panel.py` provides collision-aware Cartesian goal execution.
- `cartesian_jog_panel.py` provides Servo-based Cartesian jogging.
- `crx3ia_collision_scene.launch.py` composes MoveIt, Servo, collision objects,
  panels, and optional orthographic RViz views.

### Analysis and reachability

- `reachability_sweep.py` samples IK reachability and dexterity over a volume.
- `workspace_boundary_sweep.py` samples reachable workspace boundaries.
- `orientation_access_volume.py` evaluates orientation accessibility.
- `task_reachability_atlas.py` evaluates task-specific TCP and screen regions.
- `placement_optimizer.py` scores candidate workspace placements from a
  reachability field.
- `render_workspace_map.py` renders sampled results.

These tools write CSV/SVG output. Keep experiment output under an
experiment-specific directory, for example:

```bash
mkdir -p analysis/2026-10-10-crx3ia
ros2 run fanuc_scene_tools workspace_boundary_sweep \
  --output analysis/2026-10-10-crx3ia/workspace_boundary.csv
```

Record the command, robot model, planning frame, TCP, collision-scene
parameters, and sampling resolution beside each experiment. Do not copy result
files into both `fanuc_cbf/` and `official_fanuc_ws/`.

### Driver examples

- `fanuc_dual_arm_example` demonstrates MoveIt control for a dual-arm setup.
- `fanuc_rmi_controller_example` demonstrates FANUC RMI instructions and
  controller switching. It is intended for a configured physical controller,
  not the mock simulation.
- `fanuc_libs` contains the streaming-motion and RMI client libraries.
- `fanuc_msgs` and `rmi_msgs` define the custom driver interfaces.

## Safety and troubleshooting

The mock launch is for development only. Physical operation requires validated
robot limits, network configuration, teach pendant state, collision geometry,
and emergency-stop readiness.

If a launch cannot find packages, source only these environments in order:

```bash
source /opt/ros/jazzy/setup.bash
source ~/FANUC_CRX_3iA/fanuc_cbf/install/setup.bash
```

Do not source `fanuc_crx_ros2/install` or `official_fanuc_ws/install` on top of
the active workspace while diagnosing package resolution.
