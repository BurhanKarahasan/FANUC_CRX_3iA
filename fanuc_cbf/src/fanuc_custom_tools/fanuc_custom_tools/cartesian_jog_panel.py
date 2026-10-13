#!/usr/bin/env python3

"""Hold-to-jog Cartesian velocity panel driving MoveIt Servo.

The robot exposes only position command interfaces, so this is velocity control in
behaviour rather than in mechanism: the panel streams a TwistStamped while a button is
held, Servo integrates it into position setpoints at 100 Hz and feeds the trajectory
controller. Releasing the button stops the stream and Servo halts on its own timeout,
so a lost window or a crashed panel cannot leave the robot moving.
"""

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Codes from moveit_msgs/msg/ServoStatus. Getting these wrong is not cosmetic: the
# panel first subscribed to this topic as std_msgs/Int8, which silently matched
# nothing, so a halted Servo looked identical to an idle one.
SERVO_STATUS = {
    ServoStatus.INVALID: ("INVALID servo state", False),
    ServoStatus.NO_WARNING: ("ready", None),
    ServoStatus.DECELERATE_FOR_APPROACHING_SINGULARITY:
        ("slowing - approaching singularity", False),
    ServoStatus.HALT_FOR_SINGULARITY:
        ("HALTED - singularity. Use 'Move to safe pose'", False),
    ServoStatus.DECELERATE_FOR_LEAVING_SINGULARITY:
        ("slowing - leaving singularity", False),
    ServoStatus.DECELERATE_FOR_COLLISION: ("slowing - collision ahead", False),
    ServoStatus.HALT_FOR_COLLISION: ("HALTED - collision", False),
    ServoStatus.JOINT_BOUND: ("at a joint limit", False),
}
HALTED = {ServoStatus.HALT_FOR_SINGULARITY, ServoStatus.HALT_FOR_COLLISION,
          ServoStatus.INVALID}

AXES = (
    ("X", "linear", 0, "m/s"),
    ("Y", "linear", 1, "m/s"),
    ("Z", "linear", 2, "m/s"),
    ("RX", "angular", 0, "rad/s"),
    ("RY", "angular", 1, "rad/s"),
    ("RZ", "angular", 2, "rad/s"),
)


class CartesianJogNode(Node):
    def __init__(self):
        super().__init__("cartesian_jog_panel")
        self.declare_parameter("servo_node", "/servo_node")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.30)
        self.declare_parameter(
            "controller_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter(
            "joint_names", ["J1", "J2", "J3", "J4", "J5", "J6"]
        )
        # The mock hardware boots every joint at zero, which leaves the wrist straight
        # and J4 colinear with J6 -- a singularity Servo halts on, so jogging is dead
        # on arrival until the arm is moved off it. These are the SRDF 'default' group
        # state values, which bend J5 clear of it.
        self.declare_parameter(
            "safe_pose", [0.0, 0.0, 0.0, 0.0, -1.5708, 0.0]
        )
        self.declare_parameter("safe_pose_seconds", 3.0)
        # Servo has no end-effector parameter: it applies the twist at the planning
        # group's tip link, which the SRDF fixes at "flange". Jogging therefore
        # rotates about the flange no matter what TCP the goal panel is using. Naming
        # the TCP here lets the panel re-reference the twist so rotations pivot about
        # the tool instead. Translations are unaffected -- a rigid body has the same
        # linear velocity everywhere when it is not rotating -- so this only changes
        # RX/RY/RZ, which is exactly the discrepancy that shows up as "jogging works
        # on the flange, not on my TCP".
        self.declare_parameter("tcp_link", "")
        self.declare_parameter("servo_ee_link", "flange")

        servo = self.get_parameter("servo_node").value.rstrip("/")
        self.frame = self.get_parameter("planning_frame").value
        self.tcp_link = self.get_parameter("tcp_link").value.strip()
        self.servo_ee_link = self.get_parameter("servo_ee_link").value.strip()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lever = (0.0, 0.0, 0.0)
        self.lever_known = False
        command_topic = self.get_parameter("command_topic").value
        self.group = ReentrantCallbackGroup()

        self.twist_publisher = self.create_publisher(
            TwistStamped, command_topic or f"{servo}/delta_twist_cmds", 10
        )
        self.joint_publisher = self.create_publisher(
            JointJog, f"{servo}/delta_joint_cmds", 10
        )
        self.switch_client = self.create_client(
            ServoCommandType, f"{servo}/switch_command_type",
            callback_group=self.group,
        )
        self.pause_client = self.create_client(
            SetBool, f"{servo}/pause_servo", callback_group=self.group
        )
        # Servo latches its status, so a late subscriber still sees the current one.
        self.create_subscription(
            ServoStatus, f"{servo}/status", self._on_status,
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.group,
        )

        self.recovery_publisher = self.create_publisher(
            JointTrajectory, self.get_parameter("controller_topic").value, 10
        )
        self.joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self.group,
        )
        self.status_code = ServoStatus.NO_WARNING
        self.status_detail = ""
        self.status_seen = False
        # Written by the Tk thread, read by the timer. Assignment of a tuple is
        # atomic under the GIL, so no lock is needed for this one value.
        self.command = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Joint jog runs through the same stream, but Servo accepts only one command
        # type at a time, so the mode is tracked and switched on demand.
        self.joint_command = {}
        self.mode = None            # "twist" or "joint", whichever Servo is set to
        self.streaming = False
        self.sent = 0

        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.create_timer(period, self._publish, callback_group=self.group)

    def lever_arm(self):
        """Vector from the TCP to Servo's end-effector frame, in the planning frame.

        Returns None when no TCP is configured or TF is not available yet, in which
        case the twist is published unchanged and jogging behaves as it always did.
        """
        if not self.tcp_link or self.tcp_link == self.servo_ee_link:
            return None
        try:
            ee = self.tf_buffer.lookup_transform(
                self.frame, self.servo_ee_link, rclpy.time.Time()
            ).transform.translation
            tcp = self.tf_buffer.lookup_transform(
                self.frame, self.tcp_link, rclpy.time.Time()
            ).transform.translation
        except Exception:  # noqa: BLE001 - TF warms up a moment after startup
            self.lever_known = False
            return None
        self.lever = (ee.x - tcp.x, ee.y - tcp.y, ee.z - tcp.z)
        self.lever_known = True
        return self.lever

    def _on_joint_state(self, message):
        self.joint_state = message

    def positions(self):
        """Current joint angles by name, empty until the first message arrives."""
        if self.joint_state is None:
            return {}
        return dict(zip(self.joint_state.name, self.joint_state.position))

    def _on_status(self, message):
        self.status_code = message.code
        self.status_detail = message.message
        self.status_seen = True

    def send_safe_pose(self):
        """Drive the arm to a non-singular pose so Servo will accept jog commands."""
        message = JointTrajectory()
        message.joint_names = list(self.get_parameter("joint_names").value)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self.get_parameter("safe_pose").value]
        seconds = float(self.get_parameter("safe_pose_seconds").value)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        message.points = [point]
        self.recovery_publisher.publish(message)
        return seconds

    def _publish(self):
        """Stream the active command. Silence is the stop signal, so publish nothing
        when idle and let Servo time out."""
        if not self.streaming:
            return
        if self.mode == "joint":
            self._publish_joint()
            return
        if not any(self.command):
            return
        vx, vy, vz, wx, wy, wz = self.command

        # Re-reference the twist from the TCP to Servo's end-effector frame. For a
        # rigid body, v_ee = v_tcp + w x (p_ee - p_tcp). With no rotation commanded
        # the cross product is zero and this is a no-op, which is why straight
        # translation already tracked the TCP correctly.
        lever = self.lever_arm()
        if lever is not None and (wx or wy or wz):
            lx, ly, lz = lever
            vx += wy * lz - wz * ly
            vy += wz * lx - wx * lz
            vz += wx * ly - wy * lx

        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
         message.twist.angular.x, message.twist.angular.y,
         message.twist.angular.z) = (vx, vy, vz, wx, wy, wz)
        self.twist_publisher.publish(message)
        self.sent += 1

    def _publish_joint(self):
        if not self.joint_command:
            return
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        message.joint_names = list(self.joint_command.keys())
        message.velocities = [float(v) for v in self.joint_command.values()]
        message.duration = 0.0
        self.joint_publisher.publish(message)
        self.sent += 1

    def request_command_type(self, mode):
        """Switch Servo between twist and joint jog; it honours only one at a time."""
        value = (ServoCommandType.Request.JOINT_JOG if mode == "joint"
                 else ServoCommandType.Request.TWIST)
        self.mode = mode
        return self.switch_client.call_async(
            ServoCommandType.Request(command_type=value)
        )

    def request_twist_mode(self):
        return self.switch_client.call_async(
            ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST)
        )

    def set_paused(self, paused):
        return self.pause_client.call_async(SetBool.Request(data=paused))


class CartesianJogPanel:
    def __init__(self, node):
        self.node = node
        self.events = queue.Queue()
        self.held = None
        self.enabled = False

        self.root = tk.Tk()
        self.root.title("CRX-3iA Cartesian Jog")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.columnconfigure(0, weight=1)

        self.linear_var = tk.StringVar(
            value=f"{node.get_parameter('linear_speed').value:g}"
        )
        self.angular_var = tk.StringVar(
            value=f"{node.get_parameter('angular_speed').value:g}"
        )
        self.joint_speed_var = tk.StringVar(value="0.20")
        self.joint_value_vars = {}
        self.enable_var = tk.StringVar(value="DISABLED")
        self.status_var = tk.StringVar(value="Connecting to Servo...")
        self.servo_var = tk.StringVar(value="servo: waiting")

        self._build_widgets()
        self.root.bind("<Escape>", lambda _event: self._stop("Stopped (Esc)."))
        self.root.after(50, self._drain_events)
        self.root.after(300, self._poll_servo)
        self.root.after(200, self._refresh_joint_values)

    def _build_widgets(self):
        pad = {"padx": 4, "pady": 3}

        top = ttk.Frame(self.root)
        top.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 2))
        top.columnconfigure(2, weight=1)
        ttk.Label(top, text="Frame").grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(top, text=self.node.frame, font=("", 9, "bold")).grid(
            row=0, column=1, sticky="w", **pad
        )
        self.enable_button = ttk.Button(
            top, text="Enable jogging", command=self._toggle_enable
        )
        self.enable_button.grid(row=0, column=3, sticky="e", **pad)
        self.enable_label = ttk.Label(
            top, textvariable=self.enable_var, font=("", 9, "bold"),
            foreground="#b3261e",
        )
        self.enable_label.grid(row=0, column=2, sticky="e", **pad)

        jog = ttk.LabelFrame(self.root, text="Hold to jog")
        jog.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        for column in (1, 2):
            jog.columnconfigure(column, weight=1)
        self.jog_buttons = []
        for row, (name, kind, index, unit) in enumerate(AXES):
            ttk.Label(jog, text=name, width=4, font=("", 9, "bold")).grid(
                row=row, column=0, sticky="w", **pad
            )
            for column, (sign, glyph) in enumerate(
                ((-1.0, "◀  −"), (1.0, "+  ▶"))
            ):
                button = ttk.Button(jog, text=glyph)
                button.grid(row=row, column=column + 1, sticky="ew", **pad)
                button.bind(
                    "<ButtonPress-1>",
                    lambda _e, k=kind, i=index, s=sign, n=name: self._press(k, i, s, n),
                )
                button.bind("<ButtonRelease-1>", lambda _e: self._release())
                # Losing the pointer mid-drag must stop motion too.
                button.bind("<Leave>", lambda _e: self._release())
                self.jog_buttons.append(button)
            ttk.Label(jog, text=unit, width=6).grid(
                row=row, column=3, sticky="w", **pad
            )

        joints = ttk.LabelFrame(self.root, text="Hold to jog joints")
        joints.grid(row=2, column=0, sticky="ew", padx=8, pady=4)
        for column in (1, 2):
            joints.columnconfigure(column, weight=1)
        self.joint_names = list(self.node.get_parameter("joint_names").value)
        for row, name in enumerate(self.joint_names):
            ttk.Label(joints, text=name, width=4, font=("", 9, "bold")).grid(
                row=row, column=0, sticky="w", **pad
            )
            for column, (sign, glyph) in enumerate(((-1.0, "◀  −"), (1.0, "+  ▶"))):
                button = ttk.Button(joints, text=glyph)
                button.grid(row=row, column=column + 1, sticky="ew", **pad)
                button.bind(
                    "<ButtonPress-1>",
                    lambda _e, n=name, sg=sign: self._press_joint(n, sg),
                )
                button.bind("<ButtonRelease-1>", lambda _e: self._release())
                button.bind("<Leave>", lambda _e: self._release())
            ttk.Label(joints, text="rad/s", width=6).grid(
                row=row, column=3, sticky="w", **pad
            )
            # Live angle beside the buttons that move it, so the value being changed
            # is read at the point of use rather than in a separate panel.
            self.joint_value_vars[name] = tk.StringVar(value="--")
            ttk.Label(joints, textvariable=self.joint_value_vars[name], width=9,
                      anchor="e", foreground="#1a4f8a").grid(
                row=row, column=4, sticky="e", **pad
            )
        ttk.Label(joints, text="Joint speed").grid(
            row=len(self.joint_names), column=0, sticky="w", **pad
        )
        joint_scale = ttk.Scale(
            joints, from_=0.02, to=1.0, orient="horizontal",
            command=lambda v: self.joint_speed_var.set(f"{float(v):.3f}"),
        )
        joint_scale.set(float(self.joint_speed_var.get()))
        joint_scale.grid(row=len(self.joint_names), column=1, columnspan=2,
                         sticky="ew", **pad)
        ttk.Label(joints, textvariable=self.joint_speed_var, width=7).grid(
            row=len(self.joint_names), column=3, sticky="w", **pad
        )

        speeds = ttk.LabelFrame(self.root, text="Cartesian speed")
        speeds.grid(row=3, column=0, sticky="ew", padx=8, pady=4)
        speeds.columnconfigure(1, weight=1)
        for row, (label, variable, low, high) in enumerate(
            (("Linear (m/s)", self.linear_var, 0.005, 0.25),
             ("Angular (rad/s)", self.angular_var, 0.02, 1.0))
        ):
            ttk.Label(speeds, text=label, width=15).grid(
                row=row, column=0, sticky="w", **pad
            )
            scale = ttk.Scale(
                speeds, from_=low, to=high, orient="horizontal",
                command=lambda value, v=variable: v.set(f"{float(value):.3f}"),
            )
            scale.set(float(variable.get()))
            scale.grid(row=row, column=1, sticky="ew", **pad)
            ttk.Label(speeds, textvariable=variable, width=7).grid(
                row=row, column=2, sticky="e", **pad
            )

        self.safe_button = ttk.Button(
            self.root, text="Move to safe pose (clears singularity)",
            command=self._on_safe_pose,
        )
        self.safe_button.grid(row=4, column=0, sticky="ew", padx=8, pady=(4, 0))

        self.stop_button = tk.Button(
            self.root, text="STOP  (Esc)", command=lambda: self._stop("Stopped."),
            bg="#b3261e", fg="white", activebackground="#8c1d18",
            activeforeground="white", font=("", 11, "bold"), relief="raised", bd=2,
        )
        self.stop_button.grid(row=5, column=0, sticky="ew", padx=8, pady=(6, 4))

        ttk.Label(self.root, textvariable=self.servo_var, foreground="#333333").grid(
            row=6, column=0, sticky="w", padx=12
        )
        self.status_label = ttk.Label(
            self.root, textvariable=self.status_var, wraplength=420, justify="left"
        )
        self.status_label.grid(row=7, column=0, sticky="ew", padx=12, pady=(2, 10))

    # --- helpers -----------------------------------------------------------

    def _set_status(self, text, ok=None):
        self.status_var.set(text)
        self.status_label.configure(
            foreground={True: "#1a7f37", False: "#b3261e", None: "#333333"}[ok]
        )

    def _speeds(self):
        try:
            return float(self.linear_var.get()), float(self.angular_var.get())
        except ValueError:
            return 0.0, 0.0

    def _press(self, kind, index, sign, name):
        if not self.enabled:
            self._set_status("Enable jogging first.", ok=False)
            return
        if self.node.mode != "twist":
            self.node.request_command_type("twist")
        linear, angular = self._speeds()
        command = [0.0] * 6
        command[index if kind == "linear" else index + 3] = sign * (
            linear if kind == "linear" else angular
        )
        self.held = name
        self.node.command = tuple(command)
        self.node.streaming = True
        self._set_status(f"Jogging {'+' if sign > 0 else '-'}{name}.")

    def _press_joint(self, name, sign):
        """Jog one joint. Servo accepts one command type at a time, so this switches
        it into joint mode first; the Cartesian buttons switch it back."""
        if not self.enabled:
            self._set_status("Enable jogging first.", ok=False)
            return
        try:
            speed = float(self.joint_speed_var.get())
        except ValueError:
            self._set_status("Joint speed is not a number.", ok=False)
            return
        if self.node.mode != "joint":
            self.node.request_command_type("joint")
        self.held = name
        self.node.command = (0.0,) * 6
        self.node.joint_command = {name: sign * speed}
        self.node.streaming = True
        self._set_status(f"Jogging {name} {'+' if sign > 0 else '-'}.")

    def _release(self):
        if self.held is None:
            return
        self.held = None
        self.node.command = (0.0,) * 6
        self.node.joint_command = {}
        self.node.streaming = False
        self._set_status("Idle.")

    def _stop(self, message):
        self._release()
        self.node.mode = None
        self.enabled = False
        self.enable_var.set("DISABLED")
        self.enable_label.configure(foreground="#b3261e")
        self.enable_button.configure(text="Enable jogging")
        self.node.set_paused(True)
        self._set_status(message, ok=None)

    def _toggle_enable(self):
        if self.enabled:
            self._stop("Jogging disabled.")
            return
        if not self.node.switch_client.service_is_ready():
            self._set_status(
                "Servo is not running. Launch with launch_servo:=true.", ok=False
            )
            return
        self._set_status("Switching Servo to twist mode...")
        self.node.request_command_type("twist").add_done_callback(self._on_switched)

    def _on_switched(self, future):
        try:
            success = future.result().success
        except Exception as error:  # noqa: BLE001
            self.events.put((self._set_status, (f"Servo call failed: {error}", False)))
            return
        if not success:
            self.events.put(
                (self._set_status, ("Servo refused twist mode.", False))
            )
            return
        self.node.set_paused(False)
        self.events.put((self._enabled_ok, ()))

    def _enabled_ok(self):
        self.enabled = True
        self.enable_var.set("ENABLED")
        self.enable_label.configure(foreground="#1a7f37")
        self.enable_button.configure(text="Disable jogging")
        self._set_status("Ready. Hold a direction button to jog.", ok=True)

    def _on_safe_pose(self):
        """Recover from the singular boot pose by commanding the controller directly.

        This bypasses Servo on purpose: Servo is exactly what refuses to move while
        the arm sits on the singularity, so it cannot be used to escape one.
        """
        self._release()
        seconds = self.node.send_safe_pose()
        self._set_status(
            f"Moving to the safe pose over {seconds:.0f} s. "
            f"Jogging should work once it arrives.", ok=None
        )

    def _refresh_joint_values(self):
        positions = self.node.positions()
        for name, var in self.joint_value_vars.items():
            if name in positions:
                var.set(f"{math.degrees(positions[name]):+8.2f}°")
        self.root.after(150, self._refresh_joint_values)

    def _poll_servo(self):
        if not self.node.switch_client.service_is_ready():
            self.servo_var.set("servo: not running")
        elif not self.node.status_seen:
            self.servo_var.set("servo: connected, no status yet")
        else:
            code = self.node.status_code
            text, ok = SERVO_STATUS.get(code, (f"code {code}", False))
            self.servo_var.set(f"servo: {text} · {self.node.sent} commands sent")
            if code in HALTED:
                detail = f" ({self.node.status_detail})" if self.node.status_detail else ""
                self._set_status(f"Servo halted: {text}{detail}", ok=False)
            elif ok is False and self.held is not None:
                self._set_status(f"Servo says: {text}", ok=False)
        self.root.after(300, self._poll_servo)

    def _drain_events(self):
        while True:
            try:
                callback, args = self.events.get_nowait()
            except queue.Empty:
                break
            callback(*args)
        self.root.after(50, self._drain_events)

    def _on_close(self):
        self._release()
        self.node.set_paused(True)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CartesianJogNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        CartesianJogPanel(node).run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.streaming = False
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
