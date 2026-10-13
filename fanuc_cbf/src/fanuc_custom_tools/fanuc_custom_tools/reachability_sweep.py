#!/usr/bin/env python3

"""Exhaustive reachability and dexterity sweep of the workspace volume.

The workspace mesh defines the region of interest, not an obstacle: a pose inside a
solid collision object is trivially unreachable, so the box is sampled as a volume
while collision checking covers the robot and its attached tool. Remove the mesh from
the planning scene before running (workspace_mesh:="") or every sample collides.

For each grid point inside the volume the sweep asks how many approach directions are
achievable. That ratio is the dexterity index: 1.0 means the tool can arrive from any
sampled direction, a low value means the point is reachable but only from a narrow
cone, which is what "how reliably can we control orientation here" actually means.
"""

import argparse
import csv
import math
import struct
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

REACH = 0.683  # flange reach from the J1 axis, from the URDF link offsets


def read_binary_stl_bounds(path, scale):
    with open(path, "rb") as mesh_file:
        mesh_file.read(80)
        count = struct.unpack("<I", mesh_file.read(4))[0]
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        for _ in range(count):
            values = struct.unpack("<12fH", mesh_file.read(50))
            for offset in (3, 6, 9):
                for axis in range(3):
                    v = values[offset + axis] * scale
                    lo[axis] = min(lo[axis], v)
                    hi[axis] = max(hi[axis], v)
        return lo, hi


def approach_quaternion(theta, phi, roll, bend=0.0):
    """Quaternion whose tool axis points along the (theta, phi) approach direction.

    theta is measured from straight down, so theta=0 is a downward approach and
    theta=90 is horizontal. roll spins the tool about its own axis.

    bend models a tool whose axis is not the flange Z axis but tilted from it by that
    angle, as an elbowed or angled mount would be. The flange is rotated by the
    inverse bend so the TOOL, not the flange, ends up pointing along the approach.
    """
    ax = math.sin(theta) * math.cos(phi)
    ay = math.sin(theta) * math.sin(phi)
    az = -math.cos(theta)

    reference = (0.0, 0.0, 1.0) if abs(az) < 0.95 else (1.0, 0.0, 0.0)
    xx = reference[1] * az - reference[2] * ay
    xy = reference[2] * ax - reference[0] * az
    xz = reference[0] * ay - reference[1] * ax
    norm = math.sqrt(xx * xx + xy * xy + xz * xz)
    xx, xy, xz = xx / norm, xy / norm, xz / norm
    yx = ay * xz - az * xy
    yy = az * xx - ax * xz
    yz = ax * xy - ay * xx

    cos_r, sin_r = math.cos(roll), math.sin(roll)
    rx = (xx * cos_r + yx * sin_r, xy * cos_r + yy * sin_r, xz * cos_r + yz * sin_r)
    ry = (-xx * sin_r + yx * cos_r, -xy * sin_r + yy * cos_r, -xz * sin_r + yz * cos_r)
    rz = (ax, ay, az)

    if bend:
        # Post-multiply by Rx(-bend): the tool axis still lands on the approach
        # direction, but the flange itself is tilted out of it by the bend angle.
        cos_b, sin_b = math.cos(bend), math.sin(bend)
        ry, rz = (
            tuple(ry[i] * cos_b - rz[i] * sin_b for i in range(3)),
            tuple(ry[i] * sin_b + rz[i] * cos_b for i in range(3)),
        )

    trace = rx[0] + ry[1] + rz[2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (ry[2] - rz[1]) / s
        y = (rz[0] - rx[2]) / s
        z = (rx[1] - ry[0]) / s
    elif rx[0] > ry[1] and rx[0] > rz[2]:
        s = math.sqrt(1.0 + rx[0] - ry[1] - rz[2]) * 2
        w = (ry[2] - rz[1]) / s
        x = 0.25 * s
        y = (ry[0] + rx[1]) / s
        z = (rz[0] + rx[2]) / s
    elif ry[1] > rz[2]:
        s = math.sqrt(1.0 + ry[1] - rx[0] - rz[2]) * 2
        w = (rz[0] - rx[2]) / s
        x = (ry[0] + rx[1]) / s
        y = 0.25 * s
        z = (rz[1] + ry[2]) / s
    else:
        s = math.sqrt(1.0 + rz[2] - rx[0] - ry[1]) * 2
        w = (rx[1] - ry[0]) / s
        x = (rz[0] + rx[2]) / s
        y = (rz[1] + ry[2]) / s
        z = 0.25 * s
    return (x, y, z, w), (ax, ay, az)


def build_directions():
    """Approach directions: straight down, then rings tilted away from it."""
    directions = [(0.0, 0.0, "down")]
    for theta_deg in (30, 60, 90):
        for phi_deg in (0, 90, 180, 270):
            directions.append(
                (math.radians(theta_deg), math.radians(phi_deg),
                 f"t{theta_deg}p{phi_deg}")
            )
    return directions


class ReachabilitySweep(Node):
    def __init__(self):
        super().__init__("reachability_sweep")
        self.group = ReentrantCallbackGroup()
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.group
        )
        self.joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self.group,
        )

    def _on_joint_state(self, message):
        self.joint_state = message

    def solve(self, position, quaternion, group_name, tcp, frame, timeout, collide):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion

        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = tcp
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = collide
        if self.joint_state is not None:
            request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.timeout.sec = int(timeout)
        request.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        return self.ik_client.call(request).error_code.val == 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh", default="src/fanuc_custom_tools/assets/KUBTEC_workspace_20cm.stl")
    parser.add_argument("--mesh-scale", type=float, default=0.001)
    parser.add_argument("--workspace-x", type=float, default=-0.20)
    parser.add_argument("--workspace-y", type=float, default=-0.35)
    parser.add_argument("--workspace-z", type=float, default=0.02)
    parser.add_argument("--workspace-yaw", type=float, default=1.5708)
    parser.add_argument("--step", type=float, default=0.05, help="grid pitch (m)")
    parser.add_argument("--tool-length", type=float, default=0.0,
                        help="TCP offset along the tool Z axis (m)")
    parser.add_argument("--tool-bend", type=float, default=0.0,
                        help="angle between the tool axis and the flange Z axis (deg)")
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--tcp", default="fanuc_flange")
    parser.add_argument("--group", default="manipulator")
    parser.add_argument("--frame", default="world")
    parser.add_argument("--no-collide", action="store_true")
    parser.add_argument("--min-z", type=float, default=0.02,
                        help="floor of the sampled volume. Go negative to map the "
                             "reach below the robot's own base plane, which is what "
                             "mounting it on a pedestal buys.")
    parser.add_argument("--bounds", type=float, nargs=6, metavar=("X0", "X1", "Y0", "Y1", "Z0", "Z1"),
                        help="sweep this world-frame box instead of the mesh bounds; "
                             "use to map the robot's whole envelope once, then score "
                             "candidate workspace placements against it offline")
    parser.add_argument("--output", type=Path, default=Path("reachability_sweep.csv"))
    args = parser.parse_args()

    if args.bounds:
        wlo = [args.bounds[0], args.bounds[2], args.bounds[4]]
        whi = [args.bounds[1], args.bounds[3], args.bounds[5]]
    else:
        lo, hi = read_binary_stl_bounds(args.mesh, args.mesh_scale)
        cos_y, sin_y = math.cos(args.workspace_yaw), math.sin(args.workspace_yaw)

        def to_world(x, y, z):
            return (cos_y * x - sin_y * y + args.workspace_x,
                    sin_y * x + cos_y * y + args.workspace_y,
                    z + args.workspace_z)

        corners = [to_world(x, y, z)
                   for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        wlo = [min(c[i] for c in corners) for i in range(3)]
        whi = [max(c[i] for c in corners) for i in range(3)]
    print(f"workspace volume in world: "
          f"x[{wlo[0]:+.3f},{whi[0]:+.3f}] y[{wlo[1]:+.3f},{whi[1]:+.3f}] "
          f"z[{wlo[2]:+.3f},{whi[2]:+.3f}]")

    def frange(a, b, s):
        n = int(math.floor((b - a) / s)) + 1
        return [a + i * s for i in range(n)]

    points = []
    skipped_reach = 0
    for x in frange(wlo[0], whi[0], args.step):
        for y in frange(wlo[1], whi[1], args.step):
            for z in frange(wlo[2], whi[2], args.step):
                if z < args.min_z:
                    continue
                # Cheap pre-filter: the flange cannot leave a sphere of radius REACH
                # about the shoulder, so points outside it need no IK call at all.
                if math.dist((x, y, z - 0.161), (0, 0, 0)) > REACH + args.tool_length:
                    skipped_reach += 1
                    continue
                points.append((x, y, z))

    directions = build_directions()
    print(f"grid step {args.step} m -> {len(points)} candidate points "
          f"({skipped_reach} outside reach, skipped)")
    print(f"{len(directions)} approach directions per point "
          f"-> {len(points) * len(directions)} IK calls")
    print(f"tool length {args.tool_length:.3f} m, bend {args.tool_bend:.0f} deg, "
          f"collision checking {'OFF' if args.no_collide else 'ON'}")

    rclpy.init()
    node = ReachabilitySweep()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    if not node.ik_client.wait_for_service(timeout_sec=20.0):
        raise SystemExit("/compute_ik unavailable")
    deadline = time.time() + 10
    while node.joint_state is None and time.time() < deadline:
        time.sleep(0.1)

    rows = []
    started = time.time()
    for index, (x, y, z) in enumerate(points):
        achieved = []
        for theta, phi, name in directions:
            quaternion, axis = approach_quaternion(
                theta, phi, 0.0, math.radians(args.tool_bend))
            # Place the flange back along the approach axis so the tool TIP lands on
            # the sampled point.
            target = (x - axis[0] * args.tool_length,
                      y - axis[1] * args.tool_length,
                      z - axis[2] * args.tool_length)
            if node.solve(target, quaternion, args.group, args.tcp, args.frame,
                          args.timeout, not args.no_collide):
                achieved.append(name)
        rows.append({
            "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
            "n_reachable": len(achieved),
            "n_directions": len(directions),
            "dexterity": round(len(achieved) / len(directions), 4),
            "down_ok": int("down" in achieved),
            "directions": ";".join(achieved),
        })
        if index % 100 == 0 and index:
            rate = index / (time.time() - started)
            print(f"  {index}/{len(points)} points  {rate:.1f} pt/s  "
                  f"eta {(len(points) - index) / rate:.0f}s", flush=True)

    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    reachable = [r for r in rows if r["n_reachable"] > 0]
    full = [r for r in rows if r["dexterity"] == 1.0]
    cell = args.step ** 3
    print(f"\nelapsed {time.time() - started:.0f}s")
    print(f"points sampled       : {len(rows)}")
    print(f"reachable (any dir)  : {len(reachable)} "
          f"({len(reachable) / len(rows) * 100:.1f}%)  ~{len(reachable) * cell * 1000:.2f} L")
    print(f"fully dexterous      : {len(full)} "
          f"({len(full) / len(rows) * 100:.1f}%)  ~{len(full) * cell * 1000:.2f} L")
    if reachable:
        mean = sum(r["dexterity"] for r in reachable) / len(reachable)
        print(f"mean dexterity index : {mean:.3f} (over reachable points)")
    print(f"wrote {args.output.resolve()}")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
