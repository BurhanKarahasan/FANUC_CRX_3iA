# Repository Map and Iteration Notes

## Why three workspace folders exist

This aggregate repository was assembled from three directories that were
developed or copied at different points:

1. `fanuc_crx_ros2/` is the standalone driver-oriented repository. It has the
   broadest set of ROS packages and was previously connected to a GitHub remote
   owned by another account.
2. `official_fanuc_ws/` is a workspace snapshot with driver and description
   sources under `src/`, plus the scene scripts and analysis results at its
   root.
3. `fanuc_cbf/` is the active experimental workspace. It adds the
   `fanuc_scene_tools` package and contains the current collision-based filter,
   Servo integration, UI panels, launch files, and reachability analysis.

## Suggested source of truth

Use `fanuc_cbf/src/fanuc_scene_tools/` when working on:

- collision-scene publication;
- control-barrier-function velocity filtering;
- Cartesian goal and jog panels;
- Servo launch integration;
- reachability and workspace analysis tooling.

Use `fanuc_crx_ros2/` when working on the general driver stack, robot
descriptions, hardware interfaces, controllers, MoveIt configuration, or custom
messages. `official_fanuc_ws/src/` is useful for comparing the corresponding
driver and description snapshot, but should not be edited casually because it
duplicates source maintained elsewhere in this archive.

## Repeated analysis outputs

The two analysis-oriented workspaces contain similarly named outputs:

- `reachability_*.csv`: sampled reachability results across boundary and
  orientation conditions.
- `task_reachability_*.csv`: task-specific TCP and screen reachability results.
- `workspace_boundary_*.csv`: workspace boundary samples at selected heights.
- `placement_scores*.csv`: placement evaluation results.
- `orientation_access_volume.csv`: orientation accessibility measurements.
- `tcp_probe.csv`: TCP probe samples.
- matching `.svg` files: plotted visualizations of selected datasets.
- `reach_study.html`: interactive or report-style reachability visualization.

These are results, not package inputs. Keep them when they provide experiment
history, but record the generating command and parameter set before replacing a
file. Same-named files in `fanuc_cbf` and `official_fanuc_ws` may represent
different runs even when their filenames match.

## Comparing iterations

Use explicit path-qualified comparisons so the duplicate workspaces are not
confused:

```bash
diff -ru --exclude=build --exclude=install --exclude=log \
  official_fanuc_ws/src fanuc_cbf/src
```

For a package-specific comparison:

```bash
diff -ru official_fanuc_ws/src/fanuc_scene_tools \
  fanuc_cbf/src/fanuc_scene_tools
```

When a package exists only in `fanuc_cbf`, it is an experimental addition rather
than a missing upstream package. When a file exists only in
`official_fanuc_ws/src`, preserve it until its role has been checked against
the driver and launch dependencies.

## Recommended workflow

1. Start with `fanuc_cbf` for collision and reachability work.
2. Reuse driver packages from `fanuc_crx_ros2` through the workspace's normal
   ROS overlay/build process.
3. Consult `official_fanuc_ws` only when investigating historical behavior or
   comparing an earlier configuration.
4. Keep generated `build/`, `install/`, and `log/` directories local. They are
   reproducible outputs and are excluded by the root `.gitignore`.
5. Add a short note beside new analysis outputs describing the command,
   robot model, frame, TCP, and sampling parameters used to create them.
