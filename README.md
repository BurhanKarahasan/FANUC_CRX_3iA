# FANUC CRX ROS 2 Workspace and CRX-3iA Simulation

This repository contains the official FANUC ROS 2 driver and descriptions,
the active CRX-3iA ROS 2 Jazzy simulation, MoveIt configuration, Cartesian
goal/jog panels, collision-scene tools, tool meshes, TCP configuration, and an
optional predictive CBF-style velocity filter.

The active ROS workspace is [`fanuc_cbf/`](fanuc_cbf/). The source files and
meshes are portable; `build/`, `install/`, and `log/` are generated locally and
should be rebuilt on the new computer. Commands in this document that invoke
scripts or `ros2 run` are run from `fanuc_cbf/` unless stated otherwise.

## Repository layout

- `fanuc_cbf/`: active workspace and the single set of current build/run
  scripts.
- `fanuc_crx_ros2/`: earlier driver-oriented source snapshot retained for
  comparison.
- `official_fanuc_ws/`: historical/reference source snapshot retained for
  comparison.
- `killer.sh`: utility for stopping local ROS, Gazebo, and RViz processes.

The active source tree combines the official FANUC repositories:

- `fanuc_cbf/src/fanuc_driver/`: driver, hardware interface, controllers,
  MoveIt configuration, messages, RMI examples, and libraries.
- `fanuc_cbf/src/fanuc_description/`: official robot descriptions and meshes.
- `fanuc_cbf/src/fanuc_scene_tools/`: collision scene, CBF, Servo, panels,
  launch files, and reachability/placement tools.

## Contents

Important source locations:

- `src/fanuc_driver/`: FANUC descriptions, mock hardware, controllers, and MoveIt configuration.
- `src/fanuc_scene_tools/launch/crx3ia_collision_scene.launch.py`: main combined launch file.
- `src/fanuc_scene_tools/config/servo_crx3ia.yaml`: MoveIt Servo configuration.
- `src/fanuc_scene_tools/fanuc_scene_tools/cbf_velocity_filter.py`: optional predictive collision filter.
- `src/fanuc_scene_tools/fanuc_scene_tools/collision_scene_publisher.py`: workspace and attached-tool collision objects.
- `src/fanuc_scene_tools/fanuc_scene_tools/cartesian_jog_panel.py`: hold-to-jog Cartesian and joint jog GUI.
- `src/fanuc_scene_tools/fanuc_scene_tools/cartesian_goal_panel.py`: TCP goal, IK, planning, execution, and scene GUI.
- `src/fanuc_scene_tools/assets/KUBTEC_workspace_10cm.stl`: workspace collision/visualization mesh.
- `src/fanuc_scene_tools/assets/KUBTEC_workspace_20cm.stl`: larger workspace mesh for analysis tools.
- `src/fanuc_scene_tools/assets/stepper_motor.stl`: attached tool/stepper collision mesh.

The tool and collision objects are already included in the repository under
`fanuc_cbf/src/fanuc_scene_tools/assets/`. STL source dimensions are assumed
to be millimetres and the default launch scale is `0.001`, converting them to
metres.

## Prerequisites

The tested environment is:

- Ubuntu/Linux
- ROS 2 Jazzy
- MoveIt 2 for Jazzy
- Python 3 with `xacro`, `PyYAML`, `rclpy`, and Tk support
- A graphical session for RViz and Tk panels

Install the common ROS dependencies if the new computer does not already have them:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  python3-tk
```

Do not copy the old generated workspace state as a substitute for building. Copy the repository, source ROS, and build locally.

## First Build

From the workspace root:

```bash
cd /path/to/FANUC_CRX_3iA/fanuc_cbf
source /opt/ros/jazzy/setup.bash
./build_workspace.sh
source install/local_setup.bash
```

For a small source-only change:

```bash
cd /path/to/FANUC_CRX_3iA/fanuc_cbf
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fanuc_scene_tools --symlink-install
source install/local_setup.bash
```

If a copied workspace reports that `CMakeCache.txt` belongs to another directory, remove only the affected generated package cache and rebuild:

```bash
rm -rf build/fanuc_scene_tools
colcon build --packages-select fanuc_scene_tools --symlink-install
```

## Main Collision Scene

Run commands from `fanuc_cbf/`. The script is located at
`fanuc_cbf/run_crx3ia_collision_scene.sh`.

Minimal scene with RViz and the workspace collision object:

```bash
./run_crx3ia_collision_scene.sh \
  workspace_collision:=true \
  launch_servo:=false \
  launch_goal_panel:=false \
  launch_cbf:=false
```

The previously used full configuration, including the attached stepper, TCP, Servo, CBF, goal panel, and orthographic views:

```bash
./run_crx3ia_collision_scene.sh \
  workspace_x:=-0.20 workspace_y:=-0.23 workspace_z:=-0.02 \
  workspace_roll:=0.0 workspace_pitch:=0.0 workspace_yaw:=1.5708 \
  workspace_collision:=true \
  load_stepper:=true \
  stepper_x:=0.0 stepper_y:=0.05 stepper_z:=0.04 \
  stepper_roll:=1.5708 stepper_pitch:=0.0 stepper_yaw:=0.0 \
  goal_tcp_link:=tool_tcp \
  tcp_parent_link:=fanuc_flange \
  tcp_x:=0.0 tcp_y:=-0.06 tcp_z:=0.04 \
  tcp_roll:=0.0 tcp_pitch:=0.0 tcp_yaw:=0.0 \
  launch_goal_panel:=true \
  launch_servo:=true \
  launch_ortho_views:=true \
  launch_cbf:=false \
  cbf_prediction_horizon:=0.20
```

To use the CBF filter in the same scene, change only:

```bash
launch_cbf:=true
```

A common CBF test command is:

```bash
./run_crx3ia_collision_scene.sh \
  workspace_x:=-0.20 workspace_y:=-0.23 workspace_z:=-0.02 \
  workspace_roll:=0.0 workspace_pitch:=0.0 workspace_yaw:=1.5708 \
  workspace_collision:=true \
  load_stepper:=true \
  stepper_x:=0.0 stepper_y:=0.05 stepper_z:=0.04 \
  stepper_roll:=1.5708 stepper_pitch:=0.0 stepper_yaw:=0.0 \
  goal_tcp_link:=tool_tcp tcp_parent_link:=fanuc_flange \
  tcp_x:=0.0 tcp_y:=-0.06 tcp_z:=0.04 \
  launch_goal_panel:=true launch_servo:=true \
  launch_cbf:=true cbf_prediction_horizon:=0.15
```

Run from the correct directory. For example:

```bash
cd /path/to/FANUC_CRX_3iA/fanuc_cbf
./run_crx3ia_collision_scene.sh ...
```

Using `./run_crx3ia_collision_scene.sh` from `/home/user/FANUC` instead of
`/home/user/FANUC/fanuc_cbf` gives `No such file or directory`.

## Launch Arguments

All distances are metres and all roll/pitch/yaw values are radians.

| Argument | Default | Meaning |
|---|---:|---|
| `workspace_mesh` | package asset | Workspace STL path, normally internal to the launch file |
| `mesh_scale` | `0.001` | STL scale; converts millimetres to metres |
| `workspace_x/y/z` | `0.65/0/0` | Workspace mesh origin in `world` |
| `workspace_roll/pitch/yaw` | `0/0/0` | Workspace mesh orientation in radians |
| `workspace_collision` | `true` | Add workspace mesh to MoveIt collision checking; false leaves it visual-only |
| `load_stepper` | `true` | Attach `stepper_motor.stl` as a collision object |
| `stepper_x/y/z` | `0/0/0` | Tool mesh pose relative to `stepper_link` |
| `stepper_roll/pitch/yaw` | `0/0/0` | Tool mesh orientation relative to `stepper_link` |
| `stepper_link` | `fanuc_flange` | Link to which the tool mesh is attached |
| `goal_tcp_link` | `tool_tcp` | TCP TF frame used by the panels and CBF |
| `tcp_parent_link` | `fanuc_flange` | Robot-model parent link used for IK |
| `tcp_x/y/z` | `0/0/0` | TCP offset from `tcp_parent_link` |
| `tcp_roll/pitch/yaw` | `0/0/0` | TCP frame orientation from `tcp_parent_link` |
| `goal_frame` | `world` | Goal, twist, and CBF planning frame |
| `goal_group` | `manipulator` | MoveIt planning group |
| `launch_servo` | `false` | Start MoveIt Servo and Cartesian jog panel |
| `launch_goal_panel` | `false` | Start Cartesian goal/IK/planning GUI |
| `launch_cbf` | `true` | Route jog commands through the predictive filter |
| `cbf_prediction_horizon` | `0.20` | CBF look-ahead time in seconds |
| `launch_ortho_views` | `false` | Start XY, XZ, and YZ orthographic RViz views |

### Removing the Tool From Collision Checking

To test the bare robot against the workspace:

```bash
load_stepper:=false
```

This omits the attached stepper mesh. It does not remove the robot links or the workspace collision object.

### Visualizing a Region Without Treating It as an Obstacle

If the KUBTEC mesh represents a workspace region rather than solid material:

```bash
workspace_collision:=false
```

The mesh is then visualized but is not inserted into the MoveIt planning scene. Do not use this for a real fixture or enclosure that the robot must avoid.

## Controllers and Motion Paths

### MoveIt Servo Cartesian Jogging

`cartesian_jog_panel` publishes `geometry_msgs/TwistStamped` commands while a jog button is held. The command units are real speed units:

- linear: metres/second
- angular: radians/second

The panel defaults are `0.05 m/s` linear and `0.30 rad/s` angular. The panel publishes at 100 Hz.

The command path is:

```text
Cartesian jog panel
  -> /servo_node/delta_twist_cmds
  -> MoveIt Servo
  -> /joint_trajectory_controller/joint_trajectory
  -> position-only joint trajectory controller
  -> mock hardware
```

This is Cartesian velocity behavior at the input, but the simulated controller receives joint **position** trajectory points. Servo integrates the twist at its `publish_period` of `0.01 s`.

The panel also has joint jog controls. Servo switches between Twist and JointJog input modes as needed. The underlying controller remains position-command based.

### MoveIt Goal Planning

`cartesian_goal_panel` provides:

- TCP pose entry in `goal_frame`;
- IK checking through `/compute_ik`;
- whole-body collision checking through `/check_state_validity`;
- OMPL planning through `/move_action`;
- plan-only and plan-and-execute modes;
- stop/cancel behavior;
- live scene parameter updates.

Run it against an already running scene with:

```bash
./run_goal_panel.sh
```

Or start it with the full scene:

```bash
launch_goal_panel:=true
```

### Standalone Simulation

To start the CRX MoveIt simulation without the custom collision-scene tools:

```bash
./run_crx3ia_sim.sh
```

To reopen only the jog panel against an already running Servo:

```bash
./run_jog_panel.sh
```

The scene must already have been started with `launch_servo:=true`.

## TCP Configuration

The TCP is not a new robot-model link. The launch publishes a static TF frame named `tool_tcp` and converts TCP goals back to the MoveIt IK parent link `fanuc_flange`.

For a TCP 11 cm along the parent link's positive Z axis:

```bash
goal_tcp_link:=tool_tcp \
tcp_parent_link:=fanuc_flange \
tcp_x:=0.0 tcp_y:=0.0 tcp_z:=0.11 \
tcp_roll:=0.0 tcp_pitch:=0.0 tcp_yaw:=0.0
```

The tool mesh pose and TCP offset are separate:

- `stepper_x/y/z` and `stepper_roll/pitch/yaw` position the collision mesh.
- `tcp_x/y/z` and `tcp_roll/pitch/yaw` position the TCP frame.

For the current tool example, the collision mesh and TCP were tested with:

```bash
stepper_x:=0.0 stepper_y:=0.05 stepper_z:=0.04 \
stepper_roll:=1.5708 stepper_pitch:=0.0 stepper_yaw:=0.0 \
tcp_x:=0.0 tcp_y:=-0.06 tcp_z:=0.04
```

Positive TCP axes are the axes of `tcp_parent_link` after applying the specified TCP frame rotation. Check the frame visually in RViz before relying on the sign of an offset.

## Collision Scene Geometry

The scene publisher inserts:

1. `kubtec_workspace`: a fixed world-frame collision mesh, when `workspace_collision:=true`.
2. `stepper_motor`: an attached collision mesh on `stepper_link`, when `load_stepper:=true`.

The stepper has touch links for the physical mounting overlap:

```text
fanuc_flange, flange, end_effector, J6_link
```

J5 is intentionally not a touch link so genuine wrist/tool collisions remain detectable.

The workspace mesh is treated as a solid object. A mesh that describes an occupied work region must not be placed into the collision scene; use `workspace_collision:=false` for visualization-only regions.

## Servo Collision Scaling

Current settings are in
`fanuc_cbf/src/fanuc_scene_tools/config/servo_crx3ia.yaml`:

```yaml
check_collisions: true
collision_check_rate: 10.0
self_collision_proximity_threshold: 0.006
scene_collision_proximity_threshold: 0.012
```

MoveIt Servo begins reducing velocity when scene geometry is within 12 mm and checks the scene at 10 Hz. `self_collision_proximity_threshold` is 6 mm. These are early-warning thresholds, not mesh-scale parameters.

For simulation experiments only, the scene warning can be reduced, for example:

```yaml
scene_collision_proximity_threshold: 0.008
```

Keep `check_collisions: true` unless collision behavior is intentionally being isolated for a controlled test. A smaller threshold reduces warning distance; it does not improve an inaccurate mesh or calibration.

Other independent slowdown causes include:

- `lower_singularity_threshold: 30.0`;
- `hard_stop_singularity_threshold: 50.0`;
- `joint_limit_margins: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]`;
- Servo command timeout and latency;
- the attached stepper mesh, even when the TCP itself appears clear.

## Predictive CBF-Style Filter

The implementation in `cbf_velocity_filter.py` is a discrete predictive safety filter, not a formal signed-distance CBF-QP. The available MoveIt Python services return Boolean validity, not a signed collision distance and gradient.

When enabled:

```text
Cartesian jog panel
  -> /cbf_velocity_filter/input
  -> cbf_velocity_filter
  -> /servo_node/delta_twist_cmds
  -> MoveIt Servo
```

At 20 Hz it:

1. Reads the latest twist and current joint state.
2. Reads the current TCP pose from TF.
3. Predicts future TCP poses over `cbf_prediction_horizon`.
4. Converts each TCP pose to a `fanuc_flange` IK pose using the TCP offset.
5. Calls `/compute_ik`.
6. Calls `/check_state_validity` for the IK solution.
7. Sends the largest valid command fraction to Servo.

The current candidate fractions are:

```text
1.00, 0.75, 0.50, 0.25, 0.00
```

The filter is binary per candidate: a candidate is valid or invalid. It does not calculate a continuous distance-based scale. MoveIt Servo remains the independent final collision and singularity safeguard.

At `0.05 m/s`:

- horizon `0.20 s` predicts about 10 mm of translation;
- horizon `0.50 s` predicts about 25 mm;
- longer horizons intervene earlier and can feel conservative.

For close simulation work, test:

```bash
launch_cbf:=true cbf_prediction_horizon:=0.10
```

Use `launch_cbf:=false` to isolate native Servo behavior.

## Velocity Scaling Summary

There are several different scales and they should not be confused:

1. **Panel command speed**: the jog GUI's `linear_speed` and `angular_speed`, currently 0.05 m/s and 0.30 rad/s.
2. **CBF fraction**: 1.0, 0.75, 0.50, 0.25, or 0.0 after predictive validity checks.
3. **Servo collision scaling**: continuous internal reduction near the configured collision threshold.
4. **Servo singularity scaling**: internal reduction near the Jacobian singularity thresholds.
5. **Planner velocity scaling**: the goal panel's `velocity_scaling` and `acceleration_scaling`, defaulting to 0.1 for planned trajectories.
6. **Controller command type**: the final mock controller accepts joint positions, even though Servo receives Cartesian velocity commands.

The effective jog speed is approximately:

$$
 v_{out} = v_{panel} \times s_{CBF} \times s_{Servo,collision} \times s_{Servo,singularity}
$$

The exact Servo factors are computed internally and are not exposed as a single user parameter.

## Analysis and Workspace Tools

The package also contains these executable tools:

```bash
ros2 run fanuc_scene_tools workspace_boundary_sweep --help
ros2 run fanuc_scene_tools reachability_sweep --help
ros2 run fanuc_scene_tools orientation_access_volume --help
ros2 run fanuc_scene_tools placement_optimizer --help
ros2 run fanuc_scene_tools task_reachability_atlas --help
ros2 run fanuc_scene_tools render_workspace_map --help
```

Their purpose is:

- `workspace_boundary_sweep`: sample collision-free workspace boundaries.
- `reachability_sweep`: sample TCP reachability with IK and collision checks.
- `orientation_access_volume`: test multiple flange orientations at workspace points.
- `placement_optimizer`: evaluate workspace/tool placement candidates.
- `task_reachability_atlas`: produce task-specific TCP/orientation reachability data.
- `render_workspace_map`: render sampled reachability or collision data.

For dexterity studies, record more than reachable/not reachable: IK branch count, minimum singular value, Jacobian condition number, joint-limit margin, and minimum collision distance.

## Useful Diagnostics

List installed package executables:

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 pkg executables fanuc_scene_tools
```

List launch arguments:

```bash
ros2 launch fanuc_scene_tools crx3ia_collision_scene.launch.py --show-args
```

Inspect active topics:

```bash
ros2 topic list | sort
ros2 topic echo /servo_node/status
ros2 topic echo /joint_states
```

Check whether the CBF is receiving and publishing commands:

```bash
ros2 topic echo /cbf_velocity_filter/input
ros2 topic echo /servo_node/delta_twist_cmds
```

Inspect TF:

```bash
ros2 run tf2_ros tf2_echo fanuc_flange tool_tcp
```

Typical status meanings:

- `slowing - collision ahead`: Servo's scene/self-collision monitor is reducing speed.
- `HALTED - collision`: Servo has stopped due to collision safety.
- `slowing - approaching singularity`: change posture or approach orientation; collision tolerances will not fix it.
- `at a joint limit`: increase joint-limit clearance or choose another IK branch/posture.

## Reproducibility Checklist For A New Computer

1. Copy the complete `fanuc_cbf` workspace, including `src/fanuc_driver` and
   `src/fanuc_scene_tools/assets`.
2. Install ROS 2 Jazzy, MoveIt, Xacro, TF2, and Tk dependencies.
3. Run `source /opt/ros/jazzy/setup.bash`.
4. Run `./build_workspace.sh` from the workspace root.
5. Run `source install/local_setup.bash` in every new terminal before ROS commands.
6. Verify `ros2 pkg executables fanuc_scene_tools` lists `cbf_velocity_filter`, `collision_scene_publisher`, `cartesian_jog_panel`, and `cartesian_goal_panel`.
7. Start with `launch_cbf:=false` to validate native Servo.
8. Add the TCP and tool geometry arguments.
9. Enable `launch_cbf:=true` only after the native scene and TCP frames are correct.
10. Keep generated `build/`, `install/`, and `log/` local to the new machine when possible.

## Safety Note

This is a simulation and development configuration. The CBF-style filter uses discrete Boolean collision validity and does not provide a formal continuous-distance safety proof. Before using a physical robot, validate URDF/STL geometry, frame calibration, controller latency, stopping distance, speed limits, and the robot manufacturer's safety requirements.
