#!/usr/bin/env python3

import math
import os
import struct

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from shape_msgs.msg import Mesh, MeshTriangle
from visualization_msgs.msg import Marker


def read_binary_stl(path, scale):
    with open(path, "rb") as mesh_file:
        mesh_file.read(80)
        triangle_count = struct.unpack("<I", mesh_file.read(4))[0]
        mesh = Mesh()
        for _ in range(triangle_count):
            values = struct.unpack("<12fH", mesh_file.read(50))
            first_index = len(mesh.vertices)
            for offset in (3, 6, 9):
                mesh.vertices.append(
                    Point(x=values[offset] * scale, y=values[offset + 1] * scale, z=values[offset + 2] * scale)
                )
            mesh.triangles.append(
                MeshTriangle(vertex_indices=[first_index, first_index + 1, first_index + 2])
            )
        return mesh


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


class CollisionScenePublisher(Node):
    def __init__(self):
        super().__init__("collision_scene_publisher")
        self.declare_parameter("workspace_mesh", "")
        self.declare_parameter("stepper_mesh", "")
        self.declare_parameter("mesh_scale", 0.001)
        self.declare_parameter("workspace_x", 0.65)
        self.declare_parameter("workspace_y", 0.0)
        self.declare_parameter("workspace_z", 0.0)
        self.declare_parameter("workspace_roll", 0.0)
        self.declare_parameter("workspace_pitch", 0.0)
        self.declare_parameter("workspace_yaw", 0.0)
        self.declare_parameter("stepper_x", 0.0)
        self.declare_parameter("stepper_y", 0.0)
        self.declare_parameter("stepper_z", 0.0)
        self.declare_parameter("stepper_roll", 0.0)
        self.declare_parameter("stepper_pitch", 0.0)
        self.declare_parameter("stepper_yaw", 0.0)
        self.declare_parameter("stepper_link", "fanuc_flange")
        # Links the stepper is allowed to touch. It is bolted to the flange, so it
        # permanently overlaps J6_link; without these the whole-body collision check
        # reports START_STATE_IN_COLLISION and every plan is rejected. J5_link is
        # deliberately absent so genuine wrist collisions are still caught.
        self.declare_parameter(
            "stepper_touch_links",
            ["fanuc_flange", "flange", "end_effector", "J6_link"],
        )
        self.declare_parameter("frame_id", "world")

        # Republishing the scene forever makes move_group abort every execution with
        # MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE, because each message counts
        # as an environment change. Apply the scene once through the service instead
        # and stop; set keep_republishing to restore the old streaming behaviour.
        self.declare_parameter("keep_republishing", False)

        # The workspace mesh is a solid collision object, so with the box placed over
        # the robot's working region every plan is rejected and Servo halts all
        # jogging. Set workspace_collision false when the box describes a region the
        # robot works INSIDE rather than an obstacle: it is then published as a
        # visualization marker at exactly the configured pose instead of entering the
        # planning scene at all.
        #
        # Excluding it through the allowed-collision matrix was tried first and is not
        # equivalent. move_group honoured it, but Servo keeps its own copy of the
        # scene and went on halting every jog, so the two disagreed about the same
        # box. Keeping it out of the scene entirely is the only way both agree.
        self.declare_parameter("workspace_collision", True)
        self.declare_parameter("marker_topic", "workspace_marker")

        self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        # Latched so a display added after startup still receives the marker.
        self.marker_publisher = self.create_publisher(
            Marker, self.get_parameter("marker_topic").value,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.scene = self.build_scene()
        self.applied = False
        self.pending = None
        self.attempts = 0
        self.dirty = False
        self.timer = self.create_timer(1.0, self.publish_scene)

        # Pose and mesh parameters are settable at runtime so a design sweep can try
        # workspace and tool configurations without relaunching the stack.
        self.reconfigurable = {
            "workspace_mesh", "stepper_mesh", "mesh_scale", "frame_id",
            "stepper_link", "stepper_touch_links",
            "workspace_x", "workspace_y", "workspace_z",
            "workspace_roll", "workspace_pitch", "workspace_yaw",
            "stepper_x", "stepper_y", "stepper_z",
            "stepper_roll", "stepper_pitch", "stepper_yaw",
        }
        self.add_post_set_parameters_callback(self._on_parameters_set)

    def _on_parameters_set(self, parameters):
        # Only flag the change here. Parameters set one at a time would otherwise
        # rebuild the scene once per parameter; the timer coalesces them into one.
        if not any(p.name in self.reconfigurable for p in parameters):
            return
        self.dirty = True
        if self.timer.is_canceled():
            self.timer.reset()

    def rebuild_scene(self):
        # Clear the old objects first. Leaving them to be overwritten by id leaves a
        # stale attached body behind whenever stepper_link changes.
        self.retract_scene()
        self.scene = self.build_scene()
        self.applied = False
        self.pending = None
        self.attempts = 0
        self.get_logger().info("Scene parameters changed; reapplying planning scene.")

    def retract_scene(self):
        """Remove the objects this node owns from move_group's scene."""
        removal = PlanningScene()
        removal.is_diff = True

        workspace = CollisionObject()
        workspace.id = "kubtec_workspace"
        workspace.header.frame_id = self.get_parameter("frame_id").value
        workspace.operation = CollisionObject.REMOVE
        removal.world.collision_objects = [workspace]

        stepper = CollisionObject()
        stepper.id = "stepper_motor"
        stepper.operation = CollisionObject.REMOVE
        detach = AttachedCollisionObject()
        detach.link_name = self.previous_stepper_link
        detach.object = stepper
        removal.robot_state.is_diff = True
        removal.robot_state.attached_collision_objects = [detach]

        if self.apply_client.service_is_ready():
            self.apply_client.call_async(
                ApplyPlanningScene.Request(scene=removal)
            )
        else:
            self.publisher.publish(removal)

    def build_scene(self):
        frame_id = self.get_parameter("frame_id").value
        scale = float(self.get_parameter("mesh_scale").value)
        # Remembered so a later retract detaches from the link actually used, even if
        # stepper_link is what changed.
        self.previous_stepper_link = self.get_parameter("stepper_link").value
        objects = []
        workspace_path = self.get_parameter("workspace_mesh").value
        if not self.get_parameter("workspace_collision").value:
            # Region of interest, not an obstacle: it is drawn as a marker instead so
            # that move_group and Servo both see a scene without it.
            workspace_path = ""
        if workspace_path and os.path.isfile(workspace_path):
            workspace = CollisionObject()
            workspace.id = "kubtec_workspace"
            workspace.header.frame_id = frame_id
            workspace.meshes = [read_binary_stl(workspace_path, scale)]
            workspace_pose = Pose()
            workspace_pose.position.x = float(self.get_parameter("workspace_x").value)
            workspace_pose.position.y = float(self.get_parameter("workspace_y").value)
            workspace_pose.position.z = float(self.get_parameter("workspace_z").value)
            workspace_pose.orientation.x, workspace_pose.orientation.y, workspace_pose.orientation.z, workspace_pose.orientation.w = quaternion_from_rpy(
                float(self.get_parameter("workspace_roll").value),
                float(self.get_parameter("workspace_pitch").value),
                float(self.get_parameter("workspace_yaw").value),
            )
            workspace.mesh_poses = [workspace_pose]
            workspace.operation = CollisionObject.ADD
            objects.append(workspace)
            self.get_logger().info(f"Added fixed kubtec_workspace in {frame_id} from {workspace_path}")
        else:
            self.get_logger().warning(f"Skipping kubtec_workspace; mesh not found: {workspace_path}")

        attached = None
        stepper_path = self.get_parameter("stepper_mesh").value
        if stepper_path and os.path.isfile(stepper_path):
            stepper = CollisionObject()
            stepper.id = "stepper_motor"
            stepper.header.frame_id = self.get_parameter("stepper_link").value
            stepper.meshes = [read_binary_stl(stepper_path, scale)]
            stepper_pose = Pose()
            stepper_pose.position.x = float(self.get_parameter("stepper_x").value)
            stepper_pose.position.y = float(self.get_parameter("stepper_y").value)
            stepper_pose.position.z = float(self.get_parameter("stepper_z").value)
            stepper_pose.orientation.x, stepper_pose.orientation.y, stepper_pose.orientation.z, stepper_pose.orientation.w = quaternion_from_rpy(
                float(self.get_parameter("stepper_roll").value),
                float(self.get_parameter("stepper_pitch").value),
                float(self.get_parameter("stepper_yaw").value),
            )
            stepper.mesh_poses = [stepper_pose]
            stepper.operation = CollisionObject.ADD
            attached = AttachedCollisionObject()
            attached.link_name = self.get_parameter("stepper_link").value
            attached.object = stepper
            touch_links = list(self.get_parameter("stepper_touch_links").value)
            if attached.link_name not in touch_links:
                touch_links.append(attached.link_name)
            attached.touch_links = touch_links
            self.get_logger().info(
                f"Attached stepper_motor to {attached.link_name} from {stepper_path} "
                f"(touch_links: {', '.join(touch_links)})"
            )
        else:
            self.get_logger().warning(f"Skipping stepper_motor; mesh not found: {stepper_path}")

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects = objects
        if attached is not None:
            planning_scene.robot_state.is_diff = True
            planning_scene.robot_state.attached_collision_objects = [attached]
        return planning_scene

    def publish_scene(self):
        if self.dirty:
            # Give the removal a tick to land before adding the new geometry.
            self.dirty = False
            self.rebuild_scene()
            return
        if self.get_parameter("keep_republishing").value:
            self.publisher.publish(self.scene)
            return
        if self.applied or self.pending is not None:
            return

        self.attempts += 1
        if self.apply_client.service_is_ready():
            self.pending = self.apply_client.call_async(
                ApplyPlanningScene.Request(scene=self.scene)
            )
            self.pending.add_done_callback(self._on_applied)
            return

        # move_group is not up yet. Keep nudging the topic so the scene still lands
        # if the service never appears, but warn once so the cause is visible.
        self.publisher.publish(self.scene)
        if self.attempts == 15:
            self.get_logger().warning(
                "/apply_planning_scene still unavailable; falling back to publishing "
                "on /planning_scene, which can invalidate motions during execution."
            )

    def _on_applied(self, future):
        try:
            success = future.result().success
        except Exception as error:  # noqa: BLE001 - report and retry on next tick
            self.get_logger().warning(f"apply_planning_scene call failed: {error}")
            self.pending = None
            return
        self.pending = None
        if not success:
            self.get_logger().warning("apply_planning_scene reported failure; retrying")
            return
        self.applied = True
        self.timer.cancel()
        self.get_logger().info(
            "Planning scene applied once via /apply_planning_scene; no longer republishing."
        )

    def publish_workspace_marker(self):
        """Show the workspace box in RViz without putting it in the planning scene."""
        path = self.get_parameter("workspace_mesh").value
        if not (path and os.path.isfile(path)):
            return
        scale = float(self.get_parameter("mesh_scale").value)
        marker = Marker()
        marker.header.frame_id = self.get_parameter("frame_id").value
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = f"file://{path}"
        marker.pose.position.x = float(self.get_parameter("workspace_x").value)
        marker.pose.position.y = float(self.get_parameter("workspace_y").value)
        marker.pose.position.z = float(self.get_parameter("workspace_z").value)
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = quaternion_from_rpy(
            float(self.get_parameter("workspace_roll").value),
            float(self.get_parameter("workspace_pitch").value),
            float(self.get_parameter("workspace_yaw").value),
        )
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.91, 0.38, 0.16, 0.35
        )
        self.marker_publisher.publish(marker)
        self.get_logger().info(
            f"workspace_collision is false: kubtec_workspace published as a marker on "
            f"'{self.get_parameter('marker_topic').value}', not as a collision object."
        )

    def publish_scene(self):
        if self.dirty:
            # Give the removal a tick to land before adding the new geometry.
            self.dirty = False
            self.rebuild_scene()
            return
        if self.get_parameter("keep_republishing").value:
            self.publisher.publish(self.scene)
            return
        if self.applied or self.pending is not None:
            return

        self.attempts += 1
        if self.apply_client.service_is_ready():
            self.pending = self.apply_client.call_async(
                ApplyPlanningScene.Request(scene=self.scene)
            )
            self.pending.add_done_callback(self._on_applied)
            return

        # move_group is not up yet. Keep nudging the topic so the scene still lands
        # if the service never appears, but warn once so the cause is visible.
        self.publisher.publish(self.scene)
        if self.attempts == 15:
            self.get_logger().warning(
                "/apply_planning_scene still unavailable; falling back to publishing "
                "on /planning_scene, which can invalidate motions during execution."
            )

    def _on_applied(self, future):
        try:
            success = future.result().success
        except Exception as error:  # noqa: BLE001 - report and retry on next tick
            self.get_logger().warning(f"apply_planning_scene call failed: {error}")
            self.pending = None
            return
        self.pending = None
        if not success:
            self.get_logger().warning("apply_planning_scene reported failure; retrying")
            return
        self.applied = True
        self.timer.cancel()
        self.get_logger().info(
            "Planning scene applied once via /apply_planning_scene; no longer republishing."
        )
        if not self.get_parameter("workspace_collision").value:
            self.publish_workspace_marker()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionScenePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()