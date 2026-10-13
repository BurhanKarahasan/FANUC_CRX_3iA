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


class WorkspaceBoundarySweep(Node):
    def __init__(self):
        super().__init__("workspace_boundary_sweep")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)

    def _joint_state_callback(self, message):
        self.joint_state = message

    def wait_for_services(self):
        for client, name in (
            (self.ik_client, "/compute_ik"),
            (self.validity_client, "/check_state_validity"),
        ):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"MoveIt service unavailable: {name}")

        deadline = self.get_clock().now().nanoseconds + 10_000_000_000
        while self.joint_state is None and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            raise RuntimeError("No /joint_states received within 10 seconds")

    def evaluate(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "manipulator"
        ik_request.ik_request.ik_link_name = "fanuc_flange"
        ik_request.ik_request.pose_stamped = pose
        ik_request.ik_request.robot_state.joint_state = self.joint_state
        ik_request.ik_request.timeout.sec = 1
        ik_response = self._call(self.ik_client, ik_request)
        if ik_response.error_code.val != 1:
            return {"reachable": False, "valid": False, "error_code": ik_response.error_code.val}

        validity_request = GetStateValidity.Request()
        validity_request.group_name = "manipulator"
        validity_request.robot_state = ik_response.solution
        validity_response = self._call(self.validity_client, validity_request)
        return {
            "reachable": True,
            "valid": validity_response.valid,
            "error_code": ik_response.error_code.val,
        }

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            raise RuntimeError("Timed out waiting for MoveIt service response")
        return future.result()


def radial_point(angle, radius, z):
    return radius * math.cos(angle), radius * math.sin(angle), z


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=float, default=0.20)
    parser.add_argument("--output", type=Path, default=Path("workspace_boundary_results.csv"))
    arguments = parser.parse_args()

    rclpy.init()
    node = WorkspaceBoundarySweep()
    try:
        node.wait_for_services()
        rows = []
        for angle_degrees in range(0, 360, 15):
            angle = math.radians(angle_degrees)
            last_safe = None
            first_unsafe = None
            for radius in [index * 0.05 for index in range(1, 25)]:
                x, y, z = radial_point(angle, radius, arguments.height)
                result = node.evaluate(x, y, z)
                safe = result["reachable"] and result["valid"]
                rows.append({"angle_deg": angle_degrees, "radius_m": radius, "x_m": x, "y_m": y, "z_m": z, **result})
                if safe:
                    last_safe = radius
                elif last_safe is not None:
                    first_unsafe = radius
                    break

            if last_safe is not None and first_unsafe is not None:
                low, high = last_safe, first_unsafe
                for _ in range(4):
                    radius = (low + high) / 2
                    x, y, z = radial_point(angle, radius, arguments.height)
                    result = node.evaluate(x, y, z)
                    safe = result["reachable"] and result["valid"]
                    rows.append({"angle_deg": angle_degrees, "radius_m": radius, "x_m": x, "y_m": y, "z_m": z, **result})
                    if safe:
                        low = radius
                    else:
                        high = radius
                print(f"{angle_degrees:3d} deg: safe through {low:.3f} m; boundary before {high:.3f} m")
            else:
                print(f"{angle_degrees:3d} deg: no safe-to-unsafe boundary found in 0.05-1.20 m")

        with arguments.output.open("w", newline="") as result_file:
            writer = csv.DictWriter(result_file, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        print(f"Detailed samples written to {arguments.output.resolve()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()