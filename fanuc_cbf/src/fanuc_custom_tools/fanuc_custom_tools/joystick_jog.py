#!/usr/bin/env python3

"""Direct pygame F710 interface feeding the existing MoveIt Servo safety path."""

import os

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

try:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
except ImportError as error:
    pygame = None
    PYGAME_IMPORT_ERROR = error


class JoystickJog(Node):
    def __init__(self):
        super().__init__("joystick_jog")
        if pygame is None:
            raise RuntimeError(
                "pygame is required for direct F710 input; install python3-pygame"
            ) from PYGAME_IMPORT_ERROR

        self.declare_parameter("servo_node", "/servo_node")
        self.declare_parameter("command_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("device_index", 0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.30)
        self.declare_parameter("deadman_button", 5)

        self.frame = str(self.get_parameter("planning_frame").value)
        self.device_index = int(self.get_parameter("device_index").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.publisher = self.create_publisher(
            TwistStamped, str(self.get_parameter("command_topic").value), 10
        )
        servo = str(self.get_parameter("servo_node").value).rstrip("/")
        self.switch_client = self.create_client(
            ServoCommandType, f"{servo}/switch_command_type"
        )
        self.pause_client = self.create_client(SetBool, f"{servo}/pause_servo")

        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.twist_requested = False
        self.twist_ready = False
        self.last_enabled = False
        self.last_report = 0.0
        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate").value), self._poll
        )
        self.get_logger().info("Waiting for pygame joystick...")

    def _connect(self):
        count = pygame.joystick.get_count()
        if count <= self.device_index:
            self.joystick = None
            return False
        if self.joystick is None:
            self.joystick = pygame.joystick.Joystick(self.device_index)
            self.joystick.init()
            self.get_logger().info(
                f"Connected joystick {self.device_index}: {self.joystick.get_name()} "
                f"({self.joystick.get_numaxes()} axes, "
                f"{self.joystick.get_numbuttons()} buttons, "
                f"{self.joystick.get_numhats()} hats)"
            )
        return True

    def _value(self, value):
        if abs(value) <= self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _axis(self, index):
        if index >= self.joystick.get_numaxes():
            return 0.0
        return self._value(float(self.joystick.get_axis(index)))

    def _publish_twist(self, values):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        ) = values
        self.publisher.publish(message)

    def _request_twist_mode(self):
        if self.twist_requested or not self.switch_client.service_is_ready():
            return
        self.twist_requested = True
        request = ServoCommandType.Request(
            command_type=ServoCommandType.Request.TWIST
        )
        self.switch_client.call_async(request).add_done_callback(
            self._on_twist_mode_response
        )

    def _on_twist_mode_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.twist_requested = False
            self.get_logger().error(f"Failed to switch Servo to twist mode: {error}")
            return
        if not response.success:
            self.twist_requested = False
            self.get_logger().error("Servo refused twist mode.")
            return
        self.twist_ready = True
        if self.pause_client.service_is_ready():
            self.pause_client.call_async(SetBool.Request(data=False))

    def _poll(self):
        pygame.event.pump()
        if not self._connect():
            self._stop()
            return

        deadman = (
            self.deadman_button < self.joystick.get_numbuttons()
            and bool(self.joystick.get_button(self.deadman_button))
        )
        if not deadman:
            self._stop()
            return

        self.last_enabled = True
        self._request_twist_mode()
        if not self.twist_ready:
            self._publish_twist((0.0,) * 6)
            return

        left_x = self._axis(0)
        left_y = self._axis(1)
        right_x = self._axis(2)
        right_y = self._axis(3)
        left_trigger = (self._axis(4) + 1.0) * 0.5
        right_trigger = (self._axis(5) + 1.0) * 0.5
        dpad_x, dpad_y = (0.0, 0.0)
        if self.joystick.get_numhats():
            dpad_x, dpad_y = self.joystick.get_hat(0)

        self._publish_twist(
            (
                -left_y * self.linear_speed,
                -left_x * self.linear_speed,
                -right_y * self.linear_speed,
                (right_trigger - left_trigger) * self.angular_speed,
                dpad_y * self.angular_speed,
                -right_x * self.angular_speed,
            )
        )

    def _stop(self):
        if self.last_enabled:
            self._publish_twist((0.0,) * 6)
        self.last_enabled = False
        self.twist_requested = False
        self.twist_ready = False

    def close(self):
        self._stop()
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickJog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
