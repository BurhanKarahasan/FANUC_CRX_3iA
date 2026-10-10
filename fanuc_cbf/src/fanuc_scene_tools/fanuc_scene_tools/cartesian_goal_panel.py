#!/usr/bin/env python3

"""Tk panel that sends 6-DOF Cartesian goals to MoveIt and executes them.

Runs the rclpy executor on a background thread and the Tk main loop on the main
thread. ROS work is dispatched asynchronously; results are marshalled back to
the Tk thread through a queue so no widget is ever touched off-thread.
"""

import csv
import datetime
import math
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from xml.etree import ElementTree

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
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


def quaternion_multiply(first, second):
    first_x, first_y, first_z, first_w = first
    second_x, second_y, second_z, second_w = second
    return (
        first_w * second_x + first_x * second_w + first_y * second_z - first_z * second_y,
        first_w * second_y - first_x * second_z + first_y * second_w + first_z * second_x,
        first_w * second_z + first_x * second_y - first_y * second_x + first_z * second_w,
        first_w * second_w - first_x * second_x - first_y * second_y - first_z * second_z,
    )


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(quaternion, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)),
        quaternion_conjugate(quaternion),
    )
    return rotated[:3]


class CartesianGoalNode(Node):
    """ROS side of the panel: TF, IK/validity services and the MoveGroup action."""

    def __init__(self):
        super().__init__("cartesian_goal_panel")
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "fanuc_flange")
        self.declare_parameter("tcp_parent_link", "fanuc_flange")
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("position_tolerance", 0.001)
        self.declare_parameter("orientation_tolerance", 0.01)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("log_csv", "design_sweep_log.csv")
        self.declare_parameter("joint_tolerance", 0.001)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self.callback_group,
        )
        self.joint_limits = {}
        self.create_subscription(
            String, "/robot_description", self._on_robot_description,
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
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

    def _on_robot_description(self, message):
        """Read joint limits straight from the URDF.

        Hard-coding slider ranges would silently drift from the robot; the limits are
        already in the description that move_group publishes, so use those.
        """
        try:
            root = ElementTree.fromstring(message.data)
        except ElementTree.ParseError as error:
            self.get_logger().warning(f"Could not parse robot_description: {error}")
            return
        limits = {}
        for joint in root.findall("joint"):
            limit = joint.find("limit")
            if joint.get("type") in ("revolute", "prismatic") and limit is not None:
                limits[joint.get("name")] = (
                    float(limit.get("lower")), float(limit.get("upper"))
                )
        if limits:
            self.joint_limits = limits

    def build_joint_constraints(self, targets):
        """Goal constraints pinning every joint, for a joint-space goal."""
        constraints = Constraints()
        constraints.name = "joint_goal"
        tolerance = float(self.get_parameter("joint_tolerance").value)
        for name, position in targets.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def build_joint_move_goal(self, goal, targets, plan_only):
        message = self.build_move_goal(goal, plan_only)
        message.request.goal_constraints = [self.build_joint_constraints(targets)]
        return message

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

    def build_parent_pose(self, goal):
        """Convert a requested TCP pose to the pose of MoveIt's parent link."""
        tcp_pose = self.build_pose(goal)
        offset = [float(value) for value in self.get_parameter("tcp_offset").value]
        tcp_rotation = (
            tcp_pose.pose.orientation.x, tcp_pose.pose.orientation.y,
            tcp_pose.pose.orientation.z, tcp_pose.pose.orientation.w,
        )
        parent_rotation = quaternion_multiply(
            tcp_rotation, quaternion_conjugate((0.0, 0.0, 0.0, 1.0))
        )
        offset_in_world = rotate_vector(parent_rotation, offset)
        parent_pose = PoseStamped()
        parent_pose.header = tcp_pose.header
        parent_pose.pose.position.x = tcp_pose.pose.position.x - offset_in_world[0]
        parent_pose.pose.position.y = tcp_pose.pose.position.y - offset_in_world[1]
        parent_pose.pose.position.z = tcp_pose.pose.position.z - offset_in_world[2]
        (parent_pose.pose.orientation.x, parent_pose.pose.orientation.y,
         parent_pose.pose.orientation.z, parent_pose.pose.orientation.w) = parent_rotation
        return parent_pose

    def build_constraints(self, goal):
        pose = self.build_parent_pose(goal)
        parent_link = self.get_parameter("tcp_parent_link").value

        position = PositionConstraint()
        position.header.frame_id = goal["frame"]
        position.link_name = parent_link
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
        orientation.link_name = parent_link
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
        request.ik_request.ik_link_name = self.get_parameter("tcp_parent_link").value
        request.ik_request.pose_stamped = self.build_parent_pose(goal)
        request.ik_request.avoid_collisions = False
        if self.joint_state is not None:
            request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.timeout.sec = 1
        return request


CSV_COLUMNS = [
    "timestamp",
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

        # Sliders and entries are two views of pose_vars. The guard stops the write
        # trace and the slider command from bouncing updates off each other.
        self.pose_scales = {}
        self.syncing = False
        # Joint goal: entries are in degrees for consistency with roll/pitch/yaw,
        # converted to radians only when the goal message is built.
        self.joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        self.joint_vars = {n: tk.StringVar(value="0.00") for n in self.joint_names}
        self.joint_scales = {}
        self.joint_current = tk.StringVar(value="current joints: waiting...")
        self.joints_seeded = False

        self._build_widgets()
        self.root.after(50, self._drain_events)
        self.root.after(200, self._refresh_current_pose)
        self.root.after(250, self._refresh_joints)

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

        joints = ttk.LabelFrame(self.root, text="Joint goal (deg)")
        joints.grid(row=3, column=0, sticky="ew", padx=8, pady=4)
        joints.columnconfigure(3, weight=1)
        for row, name in enumerate(self.joint_names):
            ttk.Label(joints, text=name, width=5).grid(
                row=row, column=0, sticky="w", **padding
            )
            ttk.Entry(joints, textvariable=self.joint_vars[name], width=10).grid(
                row=row, column=1, sticky="w", **padding
            )
            ttk.Label(joints, text="deg", width=4).grid(
                row=row, column=2, sticky="w", **padding
            )
            # Range is replaced once robot_description arrives with the real limits.
            scale = ttk.Scale(
                joints, from_=-180.0, to=180.0, orient="horizontal",
                command=lambda value, n=name: self._on_joint_slider(n, value),
            )
            scale.grid(row=row, column=3, sticky="ew", **padding)
            self.joint_scales[name] = scale
            self.joint_vars[name].trace_add(
                "write", lambda *_, n=name: self._on_joint_typed(n)
            )
        ttk.Label(joints, textvariable=self.joint_current,
                  foreground="#555555").grid(
            row=6, column=0, columnspan=4, sticky="w", padx=6, pady=(2, 2)
        )
        joint_buttons = ttk.Frame(joints)
        joint_buttons.grid(row=7, column=0, columnspan=4, sticky="ew")
        for column in range(3):
            joint_buttons.columnconfigure(column, weight=1)
        self.copy_joints_button = ttk.Button(
            joint_buttons, text="Copy current", command=self._on_copy_joints
        )
        self.copy_joints_button.grid(row=0, column=0, sticky="ew", **padding)
        self.plan_joints_button = ttk.Button(
            joint_buttons, text="Plan joints",
            command=lambda: self._on_move_joints(True),
        )
        self.plan_joints_button.grid(row=0, column=1, sticky="ew", **padding)
        self.execute_joints_button = ttk.Button(
            joint_buttons, text="Plan & Execute joints",
            command=lambda: self._on_move_joints(False),
        )
        self.execute_joints_button.grid(row=0, column=2, sticky="ew", **padding)

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
            self.copy_joints_button,
            self.plan_joints_button,
            self.execute_joints_button,
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
        self.root.after(250, self._refresh_joints)

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

    def _on_joint_slider(self, name, value):
        if self.syncing:
            return
        self.syncing = True
        try:
            self.joint_vars[name].set(f"{float(value):.2f}")
        finally:
            self.syncing = False

    def _on_joint_typed(self, name):
        if self.syncing:
            return
        try:
            value = float(self.joint_vars[name].get())
        except ValueError:
            return  # mid-edit text such as "-" is not an error
        scale = self.joint_scales.get(name)
        if scale is None:
            return
        self.syncing = True
        try:
            scale.set(value)
        finally:
            self.syncing = False

    def _apply_joint_limits(self):
        """Retarget the sliders once the URDF limits are known."""
        limits = self.node.joint_limits
        if not limits:
            return False
        for name, scale in self.joint_scales.items():
            if name in limits:
                low, high = limits[name]
                scale.configure(from_=math.degrees(low), to=math.degrees(high))
        return True

    def _refresh_joints(self):
        state = self.node.joint_state
        if state is not None:
            values = dict(zip(state.name, state.position))
            text = "  ".join(
                f"{n}={math.degrees(values[n]):+7.2f}"
                for n in self.joint_names if n in values
            )
            self.joint_current.set(f"current joints: {text}")
            if not self.joints_seeded and all(n in values for n in self.joint_names):
                # Start from where the arm is, so a stray Execute cannot fling it to
                # an all-zero pose the user never asked for.
                self.joints_seeded = self._apply_joint_limits()
                if self.joints_seeded:
                    self._write_joints(values)
        self.root.after(200, self._refresh_joints)

    def _write_joints(self, values):
        for name in self.joint_names:
            if name in values:
                self.joint_vars[name].set(f"{math.degrees(values[name]):.2f}")

    def _on_copy_joints(self):
        state = self.node.joint_state
        if state is None:
            self._set_status("No joint states received yet.", ok=False)
            return
        self._apply_joint_limits()
        self._write_joints(dict(zip(state.name, state.position)))
        self._set_status("Joint fields set to the current pose.", ok=True)

    def _read_joint_goal(self):
        """Parse the joint entries into radians, checked against the URDF limits."""
        targets = {}
        limits = self.node.joint_limits
        for name in self.joint_names:
            raw = self.joint_vars[name].get()
            try:
                degrees = float(raw)
            except ValueError:
                raise ValueError(f"{name} is not a number: {raw!r}")
            radians = math.radians(degrees)
            if name in limits:
                low, high = limits[name]
                if not low <= radians <= high:
                    raise ValueError(
                        f"{name} = {degrees:.2f} deg is outside its limit "
                        f"[{math.degrees(low):.1f}, {math.degrees(high):.1f}] deg."
                    )
            targets[name] = radians
        return targets

    def _on_move_joints(self, plan_only):
        try:
            goal = self._read_goal()
            targets = self._read_joint_goal()
        except ValueError as error:
            self._set_status(str(error), ok=False)
            return
        if not self.node.move_client.server_is_ready():
            self._set_status("/move_action is not available yet.", ok=False)
            return
        self._set_busy(True)
        self._set_status(
            "Planning joint goal..." if plan_only
            else "Planning and executing joint goal..."
        )
        self.plan_only = plan_only
        future = self.node.move_client.send_goal_async(
            self.node.build_joint_move_goal(goal, targets, plan_only)
        )
        future.add_done_callback(self._on_goal_response)

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

    def _log_row(self, action, **fields):
        """Record one evaluation of the goal fields."""
        row = {"action": action, "tcp_link": self.tcp_var.get().strip()}
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
