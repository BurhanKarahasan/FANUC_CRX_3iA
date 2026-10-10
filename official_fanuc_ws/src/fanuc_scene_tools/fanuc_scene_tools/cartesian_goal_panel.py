#!/usr/bin/env python3

"""Tk panel that sends 6-DOF Cartesian goals to MoveIt and executes them.

Runs the rclpy executor on a background thread and the Tk main loop on the main
thread. ROS work is dispatched asynchronously; results are marshalled back to
the Tk thread through a queue so no widget is ever touched off-thread.
"""

import csv
import datetime
import math
import os
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

ERROR_CODES = {
    1: "SUCCESS",
    0: "UNDEFINED",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -26: "START_STATE_INVALID",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}


def error_name(value):
    return ERROR_CODES.get(value, f"UNKNOWN({value})")


def describe_error(error_code):
    """Error name plus whatever detail move_group attached to it."""
    text = error_name(error_code.val)
    detail = getattr(error_code, "message", "")
    return f"{text} ({detail})" if detail else text


def quaternion_from_rpy(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    return (
        math.sin(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
        - math.cos(half_roll) * math.sin(half_pitch) * math.sin(half_yaw),
        math.cos(half_roll) * math.sin(half_pitch) * math.cos(half_yaw)
        + math.sin(half_roll) * math.cos(half_pitch) * math.sin(half_yaw),
        math.cos(half_roll) * math.cos(half_pitch) * math.sin(half_yaw)
        - math.sin(half_roll) * math.sin(half_pitch) * math.cos(half_yaw),
        math.cos(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
        + math.sin(half_roll) * math.sin(half_pitch) * math.sin(half_yaw),
    )


def rpy_from_quaternion(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class CartesianGoalNode(Node):
    """ROS side of the panel: TF, IK/validity services and the MoveGroup action."""

    def __init__(self):
        super().__init__("cartesian_goal_panel")
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "fanuc_flange")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("position_tolerance", 0.001)
        self.declare_parameter("orientation_tolerance", 0.01)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("scene_node", "collision_scene_publisher")
        self.declare_parameter("log_csv", "design_sweep_log.csv")

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self.callback_group,
        )
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.callback_group
        )
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity", callback_group=self.callback_group
        )
        self.move_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self.callback_group
        )
        self.goal_handle = None
        self.scene_parameters = AsyncParameterClient(
            self, self.get_parameter("scene_node").value,
            callback_group=self.callback_group,
        )

    def set_scene_parameters(self, values):
        """Push scene pose/mesh parameters onto the collision scene publisher.

        Atomically, so the publisher rebuilds the scene once rather than once per
        parameter.
        """
        return self.scene_parameters.set_parameters_atomically(
            [Parameter(name, value=value) for name, value in values.items()]
        )

    def _on_joint_state(self, message):
        self.joint_state = message

    def current_pose(self, reference_frame, tcp_link):
        """Latest TF pose of tcp_link in reference_frame, or None if unavailable."""
        transform = self.tf_buffer.lookup_transform(
            reference_frame, tcp_link, rclpy.time.Time()
        ).transform
        roll, pitch, yaw = rpy_from_quaternion(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        return (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            roll,
            pitch,
            yaw,
        )

    def build_pose(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = goal["frame"]
        pose.pose.position.x = goal["x"]
        pose.pose.position.y = goal["y"]
        pose.pose.position.z = goal["z"]
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion_from_rpy(goal["roll"], goal["pitch"], goal["yaw"])
        return pose

    def build_constraints(self, goal):
        pose = self.build_pose(goal)

        position = PositionConstraint()
        position.header.frame_id = goal["frame"]
        position.link_name = goal["tcp"]
        position.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        region = BoundingVolume()
        region.primitives = [
            SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[goal["position_tolerance"]]
            )
        ]
        region_pose = Pose()
        region_pose.position = pose.pose.position
        region_pose.orientation.w = 1.0
        region.primitive_poses = [region_pose]
        position.constraint_region = region
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = goal["frame"]
        orientation.link_name = goal["tcp"]
        orientation.orientation = pose.pose.orientation
        orientation.absolute_x_axis_tolerance = goal["orientation_tolerance"]
        orientation.absolute_y_axis_tolerance = goal["orientation_tolerance"]
        orientation.absolute_z_axis_tolerance = goal["orientation_tolerance"]
        orientation.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.name = "cartesian_goal"
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def build_move_goal(self, goal, plan_only):
        message = MoveGroup.Goal()
        request = message.request
        request.group_name = goal["group"]
        request.pipeline_id = "ompl"
        request.num_planning_attempts = 10
        request.allowed_planning_time = goal["planning_time"]
        request.max_velocity_scaling_factor = goal["velocity_scaling"]
        request.max_acceleration_scaling_factor = goal["acceleration_scaling"]
        request.start_state.is_diff = True
        request.goal_constraints = [self.build_constraints(goal)]

        message.planning_options.plan_only = plan_only
        message.planning_options.replan = False
        message.planning_options.planning_scene_diff.is_diff = True
        message.planning_options.planning_scene_diff.robot_state.is_diff = True
        return message

    def build_ik_request(self, goal):
        request = GetPositionIK.Request()
        request.ik_request.group_name = goal["group"]
        request.ik_request.ik_link_name = goal["tcp"]
        request.ik_request.pose_stamped = self.build_pose(goal)
        request.ik_request.avoid_collisions = False
        if self.joint_state is not None:
            request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.timeout.sec = 1
        return request


CSV_COLUMNS = [
    "timestamp", "workspace_mesh",
    "workspace_x", "workspace_y", "workspace_z",
    "workspace_roll_deg", "workspace_pitch_deg", "workspace_yaw_deg",
    "tool_x", "tool_y", "tool_z",
    "tool_roll_deg", "tool_pitch_deg", "tool_yaw_deg",
    "tcp_link", "goal_x", "goal_y", "goal_z",
    "goal_roll_deg", "goal_pitch_deg", "goal_yaw_deg",
    "action", "ik", "collision_free", "collision_pairs",
    "plan", "plan_time_s", "trajectory_points",
]


class SweepLog:
    """Appends one row per Check IK / Plan / Execute to a CSV for later analysis."""

    def __init__(self, path, logger):
        self.path = Path(path).expanduser().resolve() if path else None
        self.logger = logger
        self.started = False

    def append(self, row):
        if self.path is None:
            return None
        full = {name: "" for name in CSV_COLUMNS}
        full["timestamp"] = datetime.datetime.now().isoformat(timespec="seconds")
        full.update({k: v for k, v in row.items() if k in full})
        try:
            fresh = not self.path.exists() or self.path.stat().st_size == 0
            with self.path.open("a", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
                if fresh:
                    writer.writeheader()
                writer.writerow(full)
        except OSError as error:
            if not self.started:
                self.logger.warning(f"Cannot write {self.path}: {error}")
                self.started = True
            return None
        self.started = True
        return self.path


class CartesianGoalPanel:
    """Tk front end. All widget access happens on the Tk main thread."""

    def __init__(self, node):
        self.node = node
        self.events = queue.Queue()
        self.busy = False
        self.seeded = False

        self.root = tk.Tk()
        self.root.title("CRX-3iA Cartesian Goal")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.columnconfigure(0, weight=1)

        parameters = {
            name: node.get_parameter(name).value
            for name in (
                "group_name",
                "tcp_link",
                "reference_frame",
                "position_tolerance",
                "orientation_tolerance",
                "planning_time",
                "velocity_scaling",
                "acceleration_scaling",
                "log_csv",
            )
        }
        self.log = SweepLog(parameters["log_csv"], node.get_logger())
        self.frame_var = tk.StringVar(value=parameters["reference_frame"])
        self.tcp_var = tk.StringVar(value=parameters["tcp_link"])
        self.group_var = tk.StringVar(value=parameters["group_name"])
        self.pose_vars = {name: tk.StringVar(value="0.000") for name in
                          ("x", "y", "z", "roll", "pitch", "yaw")}
        self.position_tolerance_var = tk.StringVar(
            value=f"{parameters['position_tolerance']:g}"
        )
        self.orientation_tolerance_var = tk.StringVar(
            value=f"{parameters['orientation_tolerance']:g}"
        )
        self.planning_time_var = tk.StringVar(value=f"{parameters['planning_time']:g}")
        self.velocity_var = tk.StringVar(value=f"{parameters['velocity_scaling']:g}")
        self.acceleration_var = tk.StringVar(
            value=f"{parameters['acceleration_scaling']:g}"
        )
        self.current_var = tk.StringVar(value="current TCP: waiting for TF...")
        self.status_var = tk.StringVar(value="Idle.")

        # Scene configuration, pushed to the collision scene publisher on Apply.
        self.scene_vars = {
            f"{prefix}_{field}": tk.StringVar(value="0.000")
            for prefix in ("workspace", "tool")
            for field in ("x", "y", "z", "roll", "pitch", "yaw")
        }
        self.mesh_var = tk.StringVar(value="")
        self.mesh_choices = self._discover_meshes()
        self.scene_loaded = False

        # Sliders and entries are two views of pose_vars. The guard stops the write
        # trace and the slider command from bouncing updates off each other.
        self.pose_scales = {}
        self.syncing = False

        self._build_widgets()
        self.root.after(50, self._drain_events)
        self.root.after(200, self._refresh_current_pose)
        self.root.after(500, self._load_scene_parameters)

    def _discover_meshes(self):
        """Absolute paths of the meshes shipped in the package's assets directory."""
        try:
            from ament_index_python.packages import get_package_share_directory

            assets = Path(get_package_share_directory("fanuc_scene_tools")) / "assets"
            return sorted(str(path) for path in assets.glob("*.stl"))
        except Exception:  # noqa: BLE001 - the dropdown is a convenience only
            return []

    def _load_scene_parameters(self):
        """Seed the scene fields from whatever the scene publisher was launched with."""
        names = ["workspace_mesh"] + [
            f"{prefix}_{field}"
            for prefix in ("workspace", "stepper")
            for field in ("x", "y", "z", "roll", "pitch", "yaw")
        ]
        if not self.node.scene_parameters.services_are_ready():
            self._set_status(
                "Scene publisher not reachable; scene fields left at zero.", ok=None
            )
            return
        future = self.node.scene_parameters.get_parameters(names)
        future.add_done_callback(lambda done: self._post(self._apply_loaded, done, names))

    def _apply_loaded(self, future, names):
        try:
            values = future.result().values
        except Exception as error:  # noqa: BLE001
            self._set_status(f"Could not read scene parameters: {error}", ok=None)
            return
        for name, value in zip(names, values):
            # PARAMETER_STRING = 4, PARAMETER_DOUBLE = 3
            if name == "workspace_mesh":
                if value.type == 4 and value.string_value:
                    self.mesh_var.set(value.string_value)
                continue
            if value.type != 3:
                continue
            key = name.replace("stepper_", "tool_")
            field = key.rsplit("_", 1)[1]
            if field in ("roll", "pitch", "yaw"):
                self.scene_vars[key].set(f"{math.degrees(value.double_value):.2f}")
            else:
                self.scene_vars[key].set(f"{value.double_value:.4f}")
        self.scene_loaded = True
        self._set_status("Scene fields loaded from the running scene publisher.", ok=True)

    def _build_widgets(self):
        padding = {"padx": 4, "pady": 2}

        frames = ttk.LabelFrame(self.root, text="Frames")
        frames.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))
        for column in (1, 3, 5):
            frames.columnconfigure(column, weight=1)
        ttk.Label(frames, text="Reference").grid(row=0, column=0, sticky="w", **padding)
        ttk.Entry(frames, textvariable=self.frame_var, width=12).grid(
            row=0, column=1, sticky="ew", **padding
        )
        ttk.Label(frames, text="TCP link").grid(row=0, column=2, sticky="w", **padding)
        ttk.Entry(frames, textvariable=self.tcp_var, width=14).grid(
            row=0, column=3, sticky="ew", **padding
        )
        ttk.Label(frames, text="Group").grid(row=0, column=4, sticky="w", **padding)
        ttk.Entry(frames, textvariable=self.group_var, width=12).grid(
            row=0, column=5, sticky="ew", **padding
        )

        pose = ttk.LabelFrame(self.root, text="Goal pose")
        pose.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        pose.columnconfigure(3, weight=1)
        fields = (
            ("X", "x", "m", -0.9, 0.9),
            ("Y", "y", "m", -0.9, 0.9),
            ("Z", "z", "m", -0.2, 1.0),
            ("Roll", "roll", "deg", -180.0, 180.0),
            ("Pitch", "pitch", "deg", -180.0, 180.0),
            ("Yaw", "yaw", "deg", -180.0, 180.0),
        )
        for row, (label, key, unit, low, high) in enumerate(fields):
            ttk.Label(pose, text=label, width=5).grid(
                row=row, column=0, sticky="w", **padding
            )
            ttk.Entry(pose, textvariable=self.pose_vars[key], width=10).grid(
                row=row, column=1, sticky="w", **padding
            )
            ttk.Label(pose, text=unit, width=4).grid(
                row=row, column=2, sticky="w", **padding
            )
            scale = ttk.Scale(
                pose, from_=low, to=high, orient="horizontal",
                command=lambda value, k=key: self._on_slider(k, value),
            )
            scale.grid(row=row, column=3, sticky="ew", **padding)
            self.pose_scales[key] = scale
            self.pose_vars[key].trace_add(
                "write", lambda *_, k=key: self._on_pose_typed(k)
            )

        limits = ttk.LabelFrame(self.root, text="Planning")
        limits.grid(row=2, column=0, sticky="ew", padx=8, pady=4)
        for column in (1, 3):
            limits.columnconfigure(column, weight=1)
        entries = (
            ("Pos tol (m)", self.position_tolerance_var),
            ("Ori tol (rad)", self.orientation_tolerance_var),
            ("Plan time (s)", self.planning_time_var),
            ("Vel scale", self.velocity_var),
            ("Acc scale", self.acceleration_var),
        )
        for index, (label, variable) in enumerate(entries):
            row, column = divmod(index, 2)
            ttk.Label(limits, text=label).grid(
                row=row, column=column * 2, sticky="w", **padding
            )
            ttk.Entry(limits, textvariable=variable, width=10).grid(
                row=row, column=column * 2 + 1, sticky="ew", **padding
            )

        scene = ttk.LabelFrame(self.root, text="Scene (applies without relaunching)")
        scene.grid(row=3, column=0, sticky="ew", padx=8, pady=4)
        for column in (1, 3, 5):
            scene.columnconfigure(column, weight=1)
        for row, (label, prefix) in enumerate(
            (("Workspace", "workspace"), ("Tool mount", "tool"))
        ):
            ttk.Label(scene, text=label, font=("", 9, "bold")).grid(
                row=row * 2, column=0, sticky="w", **padding
            )
            for index, field in enumerate(("x", "y", "z")):
                ttk.Label(scene, text=f"{field.upper()} (m)").grid(
                    row=row * 2, column=index * 2 + 1, sticky="e", **padding
                )
                ttk.Entry(
                    scene, textvariable=self.scene_vars[f"{prefix}_{field}"], width=9
                ).grid(row=row * 2, column=index * 2 + 2, sticky="ew", **padding)
            for index, field in enumerate(("roll", "pitch", "yaw")):
                ttk.Label(scene, text=f"{field[:1].upper()}{field[1:]} (deg)").grid(
                    row=row * 2 + 1, column=index * 2 + 1, sticky="e", **padding
                )
                ttk.Entry(
                    scene, textvariable=self.scene_vars[f"{prefix}_{field}"], width=9
                ).grid(row=row * 2 + 1, column=index * 2 + 2, sticky="ew", **padding)

        ttk.Label(scene, text="Mesh").grid(row=4, column=0, sticky="w", **padding)
        mesh_box = ttk.Combobox(
            scene, textvariable=self.mesh_var, values=self.mesh_choices, width=44
        )
        mesh_box.grid(row=4, column=1, columnspan=5, sticky="ew", **padding)
        self.apply_scene_button = ttk.Button(
            scene, text="Apply scene", command=self._on_apply_scene
        )
        self.apply_scene_button.grid(
            row=5, column=0, columnspan=6, sticky="ew", **padding
        )

        ttk.Label(self.root, textvariable=self.current_var, foreground="#333333").grid(
            row=4, column=0, sticky="w", padx=12, pady=(4, 0)
        )

        buttons = ttk.Frame(self.root)
        buttons.grid(row=5, column=0, sticky="ew", padx=8, pady=6)
        for column in range(2):
            buttons.columnconfigure(column, weight=1)
        self.copy_button = ttk.Button(
            buttons, text="Copy current pose", command=self._on_copy
        )
        self.copy_button.grid(row=0, column=0, sticky="ew", **padding)
        self.check_button = ttk.Button(buttons, text="Check IK", command=self._on_check)
        self.check_button.grid(row=0, column=1, sticky="ew", **padding)
        self.plan_button = ttk.Button(
            buttons, text="Plan", command=lambda: self._on_move(True)
        )
        self.plan_button.grid(row=1, column=0, sticky="ew", **padding)
        self.execute_button = ttk.Button(
            buttons, text="Plan & Execute", command=lambda: self._on_move(False)
        )
        self.execute_button.grid(row=1, column=1, sticky="ew", **padding)
        self.stop_button = ttk.Button(
            buttons, text="Stop / Cancel", command=self._on_stop, state="disabled"
        )
        self.stop_button.grid(row=2, column=0, columnspan=2, sticky="ew", **padding)

        self.status_label = ttk.Label(
            self.root, textvariable=self.status_var, wraplength=420, justify="left"
        )
        self.status_label.grid(row=6, column=0, sticky="ew", padx=12, pady=(0, 10))

    # --- helpers -----------------------------------------------------------

    def _set_status(self, text, ok=None):
        colour = {True: "#1a7f37", False: "#b3261e", None: "#333333"}[ok]
        self.status_var.set(text)
        self.status_label.configure(foreground=colour)

    def _set_busy(self, busy):
        self.busy = busy
        state = "disabled" if busy else "normal"
        for button in (
            self.copy_button,
            self.check_button,
            self.plan_button,
            self.execute_button,
            self.apply_scene_button,
        ):
            button.configure(state=state)
        self.stop_button.configure(state="normal" if busy else "disabled")

    def _read_goal(self):
        """Parse the widgets into a goal dict, raising ValueError on bad input."""
        def number(variable, label):
            try:
                return float(variable.get())
            except ValueError:
                raise ValueError(f"{label} is not a number: {variable.get()!r}")

        frame = self.frame_var.get().strip()
        tcp = self.tcp_var.get().strip()
        group = self.group_var.get().strip()
        if not (frame and tcp and group):
            raise ValueError("Reference frame, TCP link and group must all be set.")
        goal = {
            "frame": frame,
            "tcp": tcp,
            "group": group,
            "position_tolerance": number(self.position_tolerance_var, "Pos tol"),
            "orientation_tolerance": number(self.orientation_tolerance_var, "Ori tol"),
            "planning_time": number(self.planning_time_var, "Plan time"),
            "velocity_scaling": number(self.velocity_var, "Vel scale"),
            "acceleration_scaling": number(self.acceleration_var, "Acc scale"),
        }
        for key in ("x", "y", "z"):
            goal[key] = number(self.pose_vars[key], key.upper())
        for key in ("roll", "pitch", "yaw"):
            goal[key] = math.radians(number(self.pose_vars[key], key.capitalize()))
        if goal["position_tolerance"] <= 0.0 or goal["orientation_tolerance"] <= 0.0:
            raise ValueError("Tolerances must be greater than zero.")
        for key in ("velocity_scaling", "acceleration_scaling"):
            if not 0.0 < goal[key] <= 1.0:
                raise ValueError(f"{key} must be in (0, 1].")
        return goal

    def _post(self, callback, *args):
        """Hand a UI update from a ROS thread back to the Tk thread."""
        self.events.put((callback, args))

    def _drain_events(self):
        while True:
            try:
                callback, args = self.events.get_nowait()
            except queue.Empty:
                break
            callback(*args)
        self.root.after(50, self._drain_events)

    def _refresh_current_pose(self):
        frame = self.frame_var.get().strip()
        tcp = self.tcp_var.get().strip()
        if frame and tcp:
            try:
                x, y, z, roll, pitch, yaw = self.node.current_pose(frame, tcp)
            except Exception:
                self.current_var.set(f"current TCP: no TF {frame} -> {tcp}")
            else:
                self.current_var.set(
                    f"current TCP: x={x:.3f} y={y:.3f} z={z:.3f} m  "
                    f"r={math.degrees(roll):.1f} p={math.degrees(pitch):.1f} "
                    f"y={math.degrees(yaw):.1f} deg"
                )
                if not self.seeded:
                    # Start from where the robot actually is rather than an
                    # all-zero pose that no arm can reach.
                    self.seeded = True
                    self._write_pose(x, y, z, roll, pitch, yaw)
                    self._set_status("Ready. Fields seeded with the current TCP pose.")
        self.root.after(200, self._refresh_current_pose)

    def _on_slider(self, key, value):
        """Slider dragged: publish the value into the shared pose variable."""
        if self.syncing:
            return
        self.syncing = True
        try:
            digits = 2 if key in ("roll", "pitch", "yaw") else 4
            self.pose_vars[key].set(f"{float(value):.{digits}f}")
        finally:
            self.syncing = False

    def _on_pose_typed(self, key):
        """Pose variable changed by typing or by seeding: move the slider to match."""
        if self.syncing:
            return
        try:
            value = float(self.pose_vars[key].get())
        except ValueError:
            return  # mid-edit text such as "-" or "0." is not an error
        scale = self.pose_scales.get(key)
        if scale is None:
            return
        self.syncing = True
        try:
            scale.set(value)
        finally:
            self.syncing = False

    def _write_pose(self, x, y, z, roll, pitch, yaw):
        for key, value in (("x", x), ("y", y), ("z", z)):
            self.pose_vars[key].set(f"{value:.4f}")
        for key, value in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
            self.pose_vars[key].set(f"{math.degrees(value):.2f}")

    # --- button handlers ---------------------------------------------------

    def _on_copy(self):
        frame = self.frame_var.get().strip()
        tcp = self.tcp_var.get().strip()
        try:
            x, y, z, roll, pitch, yaw = self.node.current_pose(frame, tcp)
        except Exception as error:
            self._set_status(f"TF lookup {frame} -> {tcp} failed: {error}", ok=False)
            return
        self._write_pose(x, y, z, roll, pitch, yaw)
        self._set_status("Goal fields set to the current TCP pose.", ok=True)

    def _read_scene(self):
        """Parse the scene fields into collision_scene_publisher parameter values."""
        values = {}
        for prefix, target in (("workspace", "workspace"), ("tool", "stepper")):
            for field in ("x", "y", "z", "roll", "pitch", "yaw"):
                raw = self.scene_vars[f"{prefix}_{field}"].get()
                try:
                    number = float(raw)
                except ValueError:
                    raise ValueError(f"Scene {prefix} {field} is not a number: {raw!r}")
                if field in ("roll", "pitch", "yaw"):
                    number = math.radians(number)
                values[f"{target}_{field}"] = number
        mesh = self.mesh_var.get().strip()
        if mesh:
            if not os.path.isfile(mesh):
                raise ValueError(f"Mesh file does not exist: {mesh}")
            values["workspace_mesh"] = mesh
        return values

    def _on_apply_scene(self):
        try:
            values = self._read_scene()
        except ValueError as error:
            self._set_status(str(error), ok=False)
            return
        if not self.node.scene_parameters.services_are_ready():
            self._set_status(
                "Scene publisher is not running; cannot apply scene changes.", ok=False
            )
            return
        self._set_busy(True)
        self._set_status("Applying scene...")
        future = self.node.set_scene_parameters(values)
        future.add_done_callback(self._on_scene_applied)

    def _on_scene_applied(self, future):
        try:
            result = future.result().result
        except Exception as error:  # noqa: BLE001
            self._post(self._finish, f"Applying scene failed: {error}", False)
            return
        if not result.successful:
            self._post(
                self._finish, f"Scene rejected: {result.reason}", False
            )
            return
        self._post(
            self._finish,
            "Scene applied. Re-run Check IK to evaluate this configuration.",
            True,
        )

    def _log_row(self, action, **fields):
        """Record one evaluation, tagged with the scene configuration in the fields."""
        try:
            scene = self._read_scene()
        except ValueError:
            scene = {}
        row = {"action": action, "workspace_mesh": self.mesh_var.get().strip(),
               "tcp_link": self.tcp_var.get().strip()}
        for target, prefix in (("workspace", "workspace"), ("stepper", "tool")):
            for field in ("x", "y", "z"):
                row[f"{prefix}_{field}"] = f"{scene.get(f'{target}_{field}', ''):.4f}" \
                    if f"{target}_{field}" in scene else ""
            for field in ("roll", "pitch", "yaw"):
                key = f"{target}_{field}"
                row[f"{prefix}_{field}_deg"] = (
                    f"{math.degrees(scene[key]):.2f}" if key in scene else ""
                )
        for key in ("x", "y", "z"):
            row[f"goal_{key}"] = self.pose_vars[key].get()
        for key in ("roll", "pitch", "yaw"):
            row[f"goal_{key}_deg"] = self.pose_vars[key].get()
        row.update(fields)
        return self.log.append(row)

    def _on_check(self):
        try:
            goal = self._read_goal()
        except ValueError as error:
            self._set_status(str(error), ok=False)
            return
        if not self.node.ik_client.service_is_ready():
            self._set_status("/compute_ik is not available yet.", ok=False)
            return

        self._set_busy(True)
        self._set_status("Solving IK...")
        future = self.node.ik_client.call_async(self.node.build_ik_request(goal))
        future.add_done_callback(lambda done: self._on_ik_done(done, goal))

    def _on_ik_done(self, future, goal):
        try:
            response = future.result()
        except Exception as error:
            self._post(self._finish, f"IK call failed: {error}", False)
            return
        if response.error_code.val != 1:
            self._post(self._log_row, "check_ik",
                       ik=error_name(response.error_code.val))
            self._post(
                self._finish,
                f"No IK solution for this pose ({describe_error(response.error_code)}).",
                False,
            )
            return
        if not self.node.validity_client.service_is_ready():
            self._post(
                self._finish,
                "IK solution found, but /check_state_validity is unavailable "
                "so collisions were not checked.",
                None,
            )
            return
        request = GetStateValidity.Request()
        # An empty group name checks every link, matching the whole-body check the
        # CheckStartStateCollision planning adapter runs. Restricting to the group
        # skips links that do not move with it, such as base_link.
        request.group_name = ""
        request.robot_state = response.solution
        # The solution must be applied as a diff, otherwise move_group replaces the
        # scene's robot state wholesale and silently drops attached collision
        # objects, which reports poses as collision free when they are not.
        request.robot_state.is_diff = True
        validity = self.node.validity_client.call_async(request)
        validity.add_done_callback(self._on_validity_done)

    def _on_validity_done(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._post(self._finish, f"State validity call failed: {error}", False)
            return
        pairs = sorted(
            {f"{c.contact_body_1}<->{c.contact_body_2}" for c in response.contacts}
        )
        self._post(self._log_row, "check_ik", ik="SUCCESS",
                   collision_free=response.valid,
                   collision_pairs=";".join(pairs))
        if response.valid:
            self._post(self._finish, "IK solution found and collision free.", True)
            return
        detail = f" Contacts: {', '.join(pairs)}." if pairs else ""
        self._post(
            self._finish, f"IK solution found but it is IN COLLISION.{detail}", False
        )

    def _on_move(self, plan_only):
        try:
            goal = self._read_goal()
        except ValueError as error:
            self._set_status(str(error), ok=False)
            return
        if not self.node.move_client.server_is_ready():
            self._set_status("/move_action is not available yet.", ok=False)
            return

        self._set_busy(True)
        self._set_status("Planning..." if plan_only else "Planning and executing...")
        self.plan_only = plan_only
        future = self.node.move_client.send_goal_async(
            self.node.build_move_goal(goal, plan_only)
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self._post(self._finish, f"Goal submission failed: {error}", False)
            return
        if not handle.accepted:
            self._post(self._finish, "move_group rejected the goal.", False)
            return
        self.node.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.node.goal_handle = None
        try:
            wrapper = future.result()
        except Exception as error:
            self._post(self._finish, f"Motion failed: {error}", False)
            return
        if wrapper.status == GoalStatus.STATUS_CANCELED:
            self._post(self._finish, "Motion canceled.", None)
            return

        result = wrapper.result
        action = "plan" if self.plan_only else "execute"
        points = len(result.planned_trajectory.joint_trajectory.points)
        self._post(self._log_row, action,
                   plan=error_name(result.error_code.val),
                   plan_time_s=f"{result.planning_time:.3f}",
                   trajectory_points=points)
        if result.error_code.val != 1:
            self._post(
                self._finish, f"Failed: {describe_error(result.error_code)}.", False
            )
            return

        verb = "Plan found" if self.plan_only else "Executed"
        self._post(
            self._finish,
            f"{verb} ({result.planning_time:.2f} s planning, {points} trajectory points).",
            True,
        )

    def _on_stop(self):
        handle = self.node.goal_handle
        if handle is None:
            self._set_status("Nothing to cancel.", ok=None)
            self._set_busy(False)
            return
        self._set_status("Canceling...")
        handle.cancel_goal_async()

    def _finish(self, text, ok):
        self._set_busy(False)
        self._set_status(text, ok=ok)

    def _on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CartesianGoalNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        CartesianGoalPanel(node).run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
