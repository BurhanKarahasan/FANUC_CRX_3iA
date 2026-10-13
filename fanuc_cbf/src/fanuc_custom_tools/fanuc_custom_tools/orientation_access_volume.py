#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
    quaternion_x, quaternion_y, quaternion_z, quaternion_w = quaternion
    return (-quaternion_x, -quaternion_y, -quaternion_z, quaternion_w)


def rotate_vector(quaternion, vector):
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, vector_quaternion), quaternion_conjugate(quaternion)
    )
    return rotated[:3]


def orientation_samples():
    half = math.sqrt(0.5)
    direction_rotations = {
        "+Z": (0.0, 0.0, 0.0, 1.0),
        "-Z": (1.0, 0.0, 0.0, 0.0),
        "+X": (0.0, half, 0.0, half),
        "-X": (0.0, -half, 0.0, half),
        "+Y": (-half, 0.0, 0.0, half),
        "-Y": (half, 0.0, 0.0, half),
    }
    rolls = {"roll 0": (0.0, 0.0, 0.0, 1.0), "roll 180": (0.0, 0.0, 1.0, 0.0)}
    return {
        f"{direction} {roll_label}": quaternion_multiply(direction_rotation, roll)
        for direction, direction_rotation in direction_rotations.items()
        for roll_label, roll in rolls.items()
    }


ORIENTATIONS = orientation_samples()


class OrientationAccessVolume(Node):
    def __init__(self):
        super().__init__("orientation_access_volume")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.joint_state = None
        self.ik_timeout_seconds = 1.0
        self.service_timeout_seconds = 5.0
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)

    def _joint_state_callback(self, message):
        self.joint_state = message

    def wait_for_services(self):
        for client, name in ((self.ik_client, "/compute_ik"), (self.validity_client, "/check_state_validity")):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"MoveIt service unavailable: {name}")
        deadline = self.get_clock().now().nanoseconds + 10_000_000_000
        while self.joint_state is None and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            raise RuntimeError("No /joint_states received within 10 seconds")

    def evaluate(self, x, y, z, quaternion, tcp_offset=(0.0, 0.0, 0.0), tcp_rotation=(0.0, 0.0, 0.0, 1.0)):
        flange_rotation = quaternion_multiply(quaternion, quaternion_conjugate(tcp_rotation))
        offset_in_world = rotate_vector(flange_rotation, tcp_offset)
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x - offset_in_world[0]
        pose.pose.position.y = y - offset_in_world[1]
        pose.pose.position.z = z - offset_in_world[2]
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = flange_rotation
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "manipulator"
        ik_request.ik_request.ik_link_name = "fanuc_flange"
        ik_request.ik_request.pose_stamped = pose
        ik_request.ik_request.robot_state.joint_state = self.joint_state
        ik_request.ik_request.timeout.sec = int(self.ik_timeout_seconds)
        ik_request.ik_request.timeout.nanosec = int((self.ik_timeout_seconds % 1) * 1_000_000_000)
        ik_response = self._call(self.ik_client, ik_request)
        if ik_response is None:
            return False, "ik-timeout"
        if ik_response.error_code.val != 1:
            return False, "ik"
        validity_request = GetStateValidity.Request()
        validity_request.group_name = "manipulator"
        validity_request.robot_state = ik_response.solution
        validity_response = self._call(self.validity_client, validity_request)
        if validity_response is None:
            return False, "validity-timeout"
        return validity_response.valid, "collision" if not validity_response.valid else "safe"

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_seconds)
        if future.result() is None:
            return None
        return future.result()


def values(start, stop, step):
    count = round((stop - start) / step)
    return [round(start + index * step, 6) for index in range(count + 1)]


def project(x, y, z):
    return 480 + (x - y) * 280, 510 - (x + y) * 140 - z * 380


def render_svg(rows, output, step):
    voxels = sorted(rows, key=lambda row: row["x_m"] + row["y_m"] + row["z_m"])
    elements = []
    for row in voxels:
        screen_x, screen_y = project(row["x_m"], row["y_m"], row["z_m"])
        color = "#167a58" if row["all_orientations_safe"] == "True" else "#adb5bd"
        opacity = "0.95" if row["all_orientations_safe"] == "True" else "0.17"
        size = step * 280 * 0.82
        elements.append(f'<rect x="{screen_x - size / 2:.1f}" y="{screen_y - size / 2:.1f}" width="{size:.1f}" height="{size:.1f}" fill="{color}" fill-opacity="{opacity}"/>')
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="960" height="700" viewBox="0 0 960 700">
<style>text {{ font-family: DejaVu Sans, sans-serif; fill: #1f2933; }} .title {{ font-size: 27px; font-weight: bold; }} .note {{ font-size: 15px; }} .axis {{ stroke: #52616b; stroke-width: 2; }}</style>
<rect width="100%" height="100%" fill="#f4f5ef"/>
<text x="45" y="45" class="title">CRX-3iA all-orientation accessibility volume</text>
<text x="45" y="72" class="note">Green voxels: all 12 sampled flange orientations have collision-free IK. Gray: at least one orientation fails.</text>
<line x1="480" y1="510" x2="700" y2="400" class="axis"/><text x="712" y="396" class="note">+X</text>
<line x1="480" y1="510" x2="260" y2="400" class="axis"/><text x="220" y="396" class="note">+Y</text>
<line x1="480" y1="510" x2="480" y2="270" class="axis"/><text x="492" y="275" class="note">+Z</text>
<circle cx="65" cy="640" r="7" fill="#167a58"/><text x="80" y="645" class="note">All 12 orientation samples safe</text>
<rect x="365" y="633" width="14" height="14" fill="#adb5bd" fill-opacity="0.4"/><text x="390" y="645" class="note">Excluded voxel</text>
<text x="45" y="680" class="note">Orientation set: tool axis along +/-X, +/-Y, +/-Z, each at two rolls. Grid spacing: {step:.2f} m. This is a sampled orientation guarantee, not continuous SO(3) proof.</text>
{"".join(elements)}
</svg>'''
    output.write_text(svg)


def main():
    parser = argparse.ArgumentParser(description="Sample a collision-safe all-orientation accessibility volume.")
    parser.add_argument("--step", type=float, default=0.20)
    parser.add_argument("--csv", type=Path, default=Path("orientation_access_volume.csv"))
    parser.add_argument("--svg", type=Path, default=Path("orientation_access_volume.svg"))
    arguments = parser.parse_args()
    rclpy.init()
    node = OrientationAccessVolume()
    try:
        node.wait_for_services()
        rows = []
        for z in values(0.0, 0.6, arguments.step):
            for y in values(-0.6, 0.6, arguments.step):
                for x in values(-0.4, 0.6, arguments.step):
                    failure = ""
                    for label, quaternion in ORIENTATIONS.items():
                        safe, failure_type = node.evaluate(x, y, z, quaternion)
                        if not safe:
                            failure = f"{label} ({failure_type})"
                            break
                    rows.append({"x_m": x, "y_m": y, "z_m": z, "all_orientations_safe": str(not failure), "first_failed_orientation": failure})
            print(f"Completed z = {z:.2f} m")
        with arguments.csv.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        render_svg(rows, arguments.svg, arguments.step)
        safe_count = sum(row["all_orientations_safe"] == "True" for row in rows)
        print(f"{safe_count}/{len(rows)} voxels meet the 12-orientation criterion")
        print(f"Wrote {arguments.csv.resolve()} and {arguments.svg.resolve()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()