#!/usr/bin/env python3

"""Dead-man joystick interface for MoveIt Servo Cartesian jogging."""

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class JoystickJogNode(Node):
    """Convert a joystick stream into a collision-checked Servo twist stream."""

    def __init__(self):
        super().__init__("joystick_jog")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("command_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("servo_node", "/servo_node")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("joy_timeout", 0.25)
        self.declare_parameter("deadman_button", 5)
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.30)
        # Logitech F710 mappings in the default X-input/Linux joy layout. Every
        # index and sign is a parameter because the controller changes its layout
        # between X/D input modes and some joy drivers expose the triggers differently.
        self.declare_parameter("axis_map", [1, 0, 5, 4, 3, 2])
        self.declare_parameter("axis_invert", [True, False, False, False, True, False])

        self.frame = str(self.get_parameter("planning_frame").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.axis_map = [int(value) for value in self.get_parameter("axis_map").value]
        self.axis_invert = [
            bool(value) for value in self.get_parameter("axis_invert").value
        ]
        if len(self.axis_map) != 6 or len(self.axis_invert) != 6:
            raise ValueError("axis_map and axis_invert must each contain six values")

        servo = str(self.get_parameter("servo_node").value).rstrip("/")
        command_topic = str(self.get_parameter("command_topic").value)
        self.publisher = self.create_publisher(TwistStamped, command_topic, 10)
        self.switch_client = self.create_client(
            ServoCommandType, f"{servo}/switch_command_type"
        )
        self.pause_client = self.create_client(SetBool, f"{servo}/pause_servo")
        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._on_joy, 10
        )
        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate").value), self._publish
        )

        self.latest_joy = None
        self.latest_joy_time = None
        self.twist_requested = False
        self.twist_ready = False
        self.was_enabled = False
        self.warned_axis = False

    def _on_joy(self, message):
        self.latest_joy = message
        self.latest_joy_time = self.get_clock().now()

    def _enabled(self):
        if self.latest_joy is None or self.latest_joy_time is None:
            return False
        age = (self.get_clock().now() - self.latest_joy_time).nanoseconds / 1e9
        if age > self.joy_timeout:
            return False
        return (
            0 <= self.deadman_button < len(self.latest_joy.buttons)
            and bool(self.latest_joy.buttons[self.deadman_button])
        )

    def _axis(self, index):
        if index < 0 or index >= len(self.latest_joy.axes):
            if not self.warned_axis:
                self.get_logger().warning(
                    "Joystick message does not contain all configured axes; "
                    "publishing zero until the mapping is corrected."
                )
                self.warned_axis = True
            return 0.0
        return float(self.latest_joy.axes[index])

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

    def _publish(self):
        enabled = self._enabled()
        if not enabled:
            if self.was_enabled:
                self._publish_twist((0.0,) * 6)
            self.was_enabled = False
            self.twist_requested = False
            self.twist_ready = False
            return

        self.was_enabled = True
        self._request_twist_mode()
        if not self.twist_ready:
            self._publish_twist((0.0,) * 6)
            return
        values = [
            self._axis(index) * (-1.0 if invert else 1.0)
            for index, invert in zip(self.axis_map, self.axis_invert)
        ]
        values[:3] = [value * self.linear_speed for value in values[:3]]
        values[3:] = [value * self.angular_speed for value in values[3:]]
        self._publish_twist(values)

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


def main(args=None):
    rclpy.init(args=args)
    node = JoystickJogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_twist((0.0,) * 6)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
