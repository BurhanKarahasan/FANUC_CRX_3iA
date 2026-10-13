#!/usr/bin/env python3

"""Predictive discrete-CBF filter for Cartesian jog commands.

MoveIt exposes collision validity as a Boolean ROS service rather than a signed
distance field. This node therefore uses a discrete barrier: each commanded twist
is rolled out over a short horizon and only the largest collision-free fraction is
forwarded to MoveIt Servo. Servo keeps its native collision checker enabled as an
independent final safety layer.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def quaternion_multiply(first, second):
    first_x, first_y, first_z, first_w = first
    second_x, second_y, second_z, second_w = second
    return (
        first_w * second_x + first_x * second_w + first_y * second_z - first_z * second_y,
        first_w * second_y - first_x * second_z + first_y * second_w + first_z * second_x,
        first_w * second_z + first_x * second_y - first_y * second_x + first_z * second_w,
        first_w * second_w - first_x * second_x - first_y * second_y - first_z * second_z,
    )


def rotation_quaternion(x, y, z, duration):
    angular_speed = math.sqrt(x * x + y * y + z * z)
    angle = angular_speed * duration
    if angle < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    scale = math.sin(angle * 0.5) / angular_speed
    return (x * scale, y * scale, z * scale, math.cos(angle * 0.5))


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(quaternion, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)),
        quaternion_conjugate(quaternion),
    )
    return rotated[:3]


class CbfVelocityFilter(Node):
    """Project requested twists onto short-horizon collision-safe candidates."""

    def __init__(self):
        super().__init__("cbf_velocity_filter")
        self.declare_parameter("input_topic", "/cbf_velocity_filter/input")
        self.declare_parameter("output_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("tcp_link", "fanuc_flange")
        self.declare_parameter("tcp_parent_link", "fanuc_flange")
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("prediction_horizon", 0.20)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("command_timeout", 0.15)
        self.declare_parameter("fractions", [1.0, 0.75, 0.50, 0.25])

        self.planning_frame = self.get_parameter("planning_frame").value
        self.tcp_link = self.get_parameter("tcp_link").value
        self.tcp_parent_link = self.get_parameter("tcp_parent_link").value
        self.tcp_offset = [float(value) for value in self.get_parameter("tcp_offset").value]
        self.group_name = self.get_parameter("group_name").value
        self.horizon = float(self.get_parameter("prediction_horizon").value)
        self.fractions = [float(value) for value in self.get_parameter("fractions").value]
        self.joint_state = None
        self.latest_command = None
        self.latest_command_time = None
        self.candidates = []
        self.candidate_index = 0
        self.request_pending = False

        self.output = self.create_publisher(
            TwistStamped, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            TwistStamped, self.get_parameter("input_topic").value,
            self._on_command, 10,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        period = 1.0 / float(self.get_parameter("control_rate").value)
        self.create_timer(period, self._start_or_stop)

    def _on_joint_state(self, message):
        self.joint_state = message

    def _on_command(self, message):
        self.latest_command = message
        self.latest_command_time = self.get_clock().now()

    def _start_or_stop(self):
        if self.latest_command is None or self.latest_command_time is None:
            return
        age = (self.get_clock().now() - self.latest_command_time).nanoseconds / 1e9
        if age > float(self.get_parameter("command_timeout").value):
            self._publish_fraction(0.0)
            return
        if self.request_pending:
            return
        if self.joint_state is None or not self.ik_client.service_is_ready() or not self.validity_client.service_is_ready():
            self._publish_fraction(0.0)
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.tcp_link, rclpy.time.Time()
            )
        except Exception as error:
            self.get_logger().debug(f"Waiting for {self.planning_frame} -> {self.tcp_link}: {error}")
            self._publish_fraction(0.0)
            return
        self.candidates = self._build_candidates(transform)
        self.candidate_index = 0
        self._check_next_candidate()

    def _build_candidates(self, transform):
        translation = transform.transform.translation
        orientation = transform.transform.rotation
        twist = self.latest_command.twist
        candidates = []
        for fraction in self.fractions:
            duration = self.horizon * fraction
            rotation = rotation_quaternion(
                twist.angular.x, twist.angular.y, twist.angular.z, duration
            )
            candidate_orientation = quaternion_multiply(
                (orientation.x, orientation.y, orientation.z, orientation.w), rotation
            )
            pose = PoseStamped()
            pose.header.frame_id = self.planning_frame
            pose.pose.position.x = translation.x + twist.linear.x * duration
            pose.pose.position.y = translation.y + twist.linear.y * duration
            pose.pose.position.z = translation.z + twist.linear.z * duration
            (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
             pose.pose.orientation.w) = candidate_orientation
            candidates.append((fraction, pose))
        return candidates

    def _check_next_candidate(self):
        if self.candidate_index >= len(self.candidates):
            self._publish_fraction(0.0)
            return
        _, pose = self.candidates[self.candidate_index]
        pose = self._tcp_to_parent_pose(pose)
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.tcp_parent_link
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.timeout.nanosec = 20_000_000
        self.request_pending = True
        self.ik_client.call_async(request).add_done_callback(self._on_ik_response)

    def _tcp_to_parent_pose(self, tcp_pose):
        rotation = (
            tcp_pose.pose.orientation.x, tcp_pose.pose.orientation.y,
            tcp_pose.pose.orientation.z, tcp_pose.pose.orientation.w,
        )
        offset_in_world = rotate_vector(rotation, self.tcp_offset)
        parent_pose = PoseStamped()
        parent_pose.header = tcp_pose.header
        parent_pose.pose.position.x = tcp_pose.pose.position.x - offset_in_world[0]
        parent_pose.pose.position.y = tcp_pose.pose.position.y - offset_in_world[1]
        parent_pose.pose.position.z = tcp_pose.pose.position.z - offset_in_world[2]
        parent_pose.pose.orientation = tcp_pose.pose.orientation
        return parent_pose

    def _on_ik_response(self, future):
        self.request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f"CBF IK request failed: {error}")
            self._publish_fraction(0.0)
            return
        if response.error_code.val != 1:
            self.candidate_index += 1
            self._check_next_candidate()
            return
        request = GetStateValidity.Request()
        request.group_name = self.group_name
        request.robot_state = response.solution
        self.request_pending = True
        self.validity_client.call_async(request).add_done_callback(
            self._on_validity_response
        )

    def _on_validity_response(self, future):
        self.request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f"CBF validity request failed: {error}")
            self._publish_fraction(0.0)
            return
        if response.valid:
            self._publish_fraction(self.candidates[self.candidate_index][0])
            return
        self.candidate_index += 1
        self._check_next_candidate()

    def _publish_fraction(self, fraction):
        if self.latest_command is None:
            return
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.planning_frame
        source = self.latest_command.twist
        message.twist.linear.x = source.linear.x * fraction
        message.twist.linear.y = source.linear.y * fraction
        message.twist.linear.z = source.linear.z * fraction
        message.twist.angular.x = source.angular.x * fraction
        message.twist.angular.y = source.angular.y * fraction
        message.twist.angular.z = source.angular.z * fraction
        self.output.publish(message)


def main():
    rclpy.init()
    node = CbfVelocityFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()