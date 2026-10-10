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
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import SetBool
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

        servo = self.get_parameter("servo_node").value.rstrip("/")
        self.frame = self.get_parameter("planning_frame").value
        command_topic = self.get_parameter("command_topic").value
        self.group = ReentrantCallbackGroup()

        self.twist_publisher = self.create_publisher(
            TwistStamped, command_topic or f"{servo}/delta_twist_cmds", 10
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
        self.status_code = ServoStatus.NO_WARNING
        self.status_detail = ""
        self.status_seen = False
        # Written by the Tk thread, read by the timer. Assignment of a tuple is
        # atomic under the GIL, so no lock is needed for this one value.
        self.command = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.streaming = False
        self.sent = 0

        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.create_timer(period, self._publish, callback_group=self.group)

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
        """Stream the current twist. Silence is the stop signal, so publish nothing
        when idle and let Servo time out."""
        if not self.streaming:
            return
        if not any(self.command):
            return
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
         message.twist.angular.x, message.twist.angular.y,
         message.twist.angular.z) = self.command
        self.twist_publisher.publish(message)
        self.sent += 1

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
        self.enable_var = tk.StringVar(value="DISABLED")
        self.status_var = tk.StringVar(value="Connecting to Servo...")
        self.servo_var = tk.StringVar(value="servo: waiting")

        self._build_widgets()
        self.root.bind("<Escape>", lambda _event: self._stop("Stopped (Esc)."))
        self.root.after(50, self._drain_events)
        self.root.after(300, self._poll_servo)

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

        speeds = ttk.LabelFrame(self.root, text="Speed")
        speeds.grid(row=2, column=0, sticky="ew", padx=8, pady=4)
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
        self.safe_button.grid(row=3, column=0, sticky="ew", padx=8, pady=(4, 0))

        self.stop_button = tk.Button(
            self.root, text="STOP  (Esc)", command=lambda: self._stop("Stopped."),
            bg="#b3261e", fg="white", activebackground="#8c1d18",
            activeforeground="white", font=("", 11, "bold"), relief="raised", bd=2,
        )
        self.stop_button.grid(row=4, column=0, sticky="ew", padx=8, pady=(6, 4))

        ttk.Label(self.root, textvariable=self.servo_var, foreground="#333333").grid(
            row=5, column=0, sticky="w", padx=12
        )
        self.status_label = ttk.Label(
            self.root, textvariable=self.status_var, wraplength=420, justify="left"
        )
        self.status_label.grid(row=6, column=0, sticky="ew", padx=12, pady=(2, 10))

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
        linear, angular = self._speeds()
        command = [0.0] * 6
        command[index if kind == "linear" else index + 3] = sign * (
            linear if kind == "linear" else angular
        )
        self.held = name
        self.node.command = tuple(command)
        self.node.streaming = True
        self._set_status(f"Jogging {'+' if sign > 0 else '-'}{name}.")

    def _release(self):
        if self.held is None:
            return
        self.held = None
        self.node.command = (0.0,) * 6
        self.node.streaming = False
        self._set_status("Idle.")

    def _stop(self, message):
        self._release()
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
        self.node.request_twist_mode().add_done_callback(self._on_switched)

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
