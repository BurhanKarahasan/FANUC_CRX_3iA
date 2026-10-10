import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def tcp_offset_param():
    """The TCP offset as a real DOUBLE_ARRAY parameter.

    A plain Python list of substitutions is NOT passed through as three values: launch
    concatenates them into a single string, so tcp_x=0.0 tcp_y=0.11 tcp_z=0.0 arrived
    as "0.00.110.0" and the node died with InvalidParameterTypeException. Building a
    list literal and declaring the type keeps it a list of floats.
    """
    return ParameterValue(
        PythonExpression([
            "[", LaunchConfiguration("tcp_x"), ",",
            LaunchConfiguration("tcp_y"), ",",
            LaunchConfiguration("tcp_z"), "]",
        ]),
        value_type=List[float],
    )


def servo_setup(context, *args, **kwargs):
    """Start MoveIt Servo plus the jog panel.

    Servo needs the robot description, its semantics and the kinematics solver, which
    only exist as a built MoveIt config, so it is assembled here rather than declared
    statically. Imports are local so the rest of the launch file still works when
    moveit_servo is not installed.
    """
    if context.perform_substitution(LaunchConfiguration("launch_servo")) != "true":
        return []

    import xacro
    import yaml

    # MoveItConfigsBuilder is deliberately not used here. Its to_moveit_configs()
    # always loads config/pilz_cartesian_limits.yaml, which fanuc_moveit_config does
    # not ship, so it raises before Servo ever starts. Servo needs only these three
    # parameters, and reading them directly avoids the whole dependency.
    moveit_share = get_package_share_directory("fanuc_moveit_config")
    urdf = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot", "crx3ia.urdf.xacro",
    )
    robot_description = xacro.process_file(
        urdf, mappings={"use_mock": "true", "robot_ip": "127.0.0.1"}
    ).toxml()
    with open(os.path.join(moveit_share, "srdf", "crx3ia.srdf")) as handle:
        semantic = handle.read()
    with open(os.path.join(moveit_share, "config", "kinematics.yaml")) as handle:
        kinematics = yaml.safe_load(handle)
    with open(os.path.join(
        get_package_share_directory("fanuc_scene_tools"),
        "config", "servo_crx3ia.yaml",
    )) as handle:
        servo_settings = yaml.safe_load(handle)

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            {"moveit_servo": servo_settings},
            {"robot_description": robot_description},
            {"robot_description_semantic": semantic},
            {"robot_description_kinematics": kinematics},
        ],
        output="screen",
    )
    cbf_filter = Node(
        package="fanuc_scene_tools",
        executable="cbf_velocity_filter",
        parameters=[
            {
                "input_topic": "/cbf_velocity_filter/input",
                "output_topic": "/servo_node/delta_twist_cmds",
                "planning_frame": LaunchConfiguration("goal_frame"),
                "tcp_link": LaunchConfiguration("goal_tcp_link"),
                "tcp_parent_link": LaunchConfiguration("tcp_parent_link"),
                "tcp_offset": tcp_offset_param(),
                "prediction_horizon": LaunchConfiguration("cbf_prediction_horizon"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_cbf")),
        output="screen",
    )
    jog_panel = Node(
        package="fanuc_scene_tools",
        executable="cartesian_jog_panel",
        parameters=[
            {
                "servo_node": "/servo_node",
                "planning_frame": LaunchConfiguration("goal_frame"),
                "command_topic": PythonExpression([
                    "'/cbf_velocity_filter/input' if '",
                    LaunchConfiguration("launch_cbf"),
                    "' == 'true' else '/servo_node/delta_twist_cmds'",
                ]),
                # Servo always applies the twist at the planning group's tip link, so
                # the panel re-references rotations onto this TCP itself.
                "tcp_link": LaunchConfiguration("goal_tcp_link"),
                "servo_ee_link": LaunchConfiguration("servo_ee_link"),
            }
        ],
        output="screen",
    )
    return [servo_node, cbf_filter, jog_panel]


def ortho_views_setup(context, *args, **kwargs):
    """Three simultaneous orthographic views, one per principal plane.

    RViz2's only orthographic view controller, TopDownOrtho, always looks down the
    -Z axis of its target frame. Rather than settle for one top view, two helper
    frames are published whose -Z axes point along -Y and -X, so the same controller
    yields true XZ and YZ orthographic views. Each view is its own RViz process
    because a single RViz renders one camera at a time.
    """
    if context.perform_substitution(
        LaunchConfiguration("launch_ortho_views")
    ) != "true":
        return []

    share = get_package_share_directory("fanuc_scene_tools")
    nodes = []
    # view_xz: Z along -Y_world, so the screen shows world X right and Z up.
    # view_yz: Z along +X_world, so the screen shows world Y right and Z up.
    for name, rpy in (("view_xz", ("1.5708", "0", "0")),
                      ("view_yz", ("1.5708", "0", "1.5708"))):
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{name}_frame",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
                       "--frame-id", "world", "--child-frame-id", name],
            output="log",
        ))
    for config in ("ortho_xy", "ortho_xz", "ortho_yz"):
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name=f"rviz_{config}",
            arguments=["--display-config", os.path.join(share, "rviz", f"{config}.rviz")],
            output="log",
        ))
    return nodes


def generate_launch_description():
    share = FindPackageShare("fanuc_scene_tools")
    workspace_mesh = PathJoinSubstitution([share, "assets", "KUBTEC_workspace_10cm.stl"])
    stepper_mesh = PythonExpression([
        "'", PathJoinSubstitution([share, "assets", "stepper_motor.stl"]),
        "' if '", LaunchConfiguration("load_stepper"), "' == 'true' else ''",
    ])
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fanuc_moveit_config"),
                "launch",
                "fanuc_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "robot_model": "crx3ia",
            "use_mock": "true",
            "launch_rviz": "true",
        }.items(),
    )
    scene_publisher = Node(
        package="fanuc_scene_tools",
        executable="collision_scene_publisher",
        parameters=[
            {
                "workspace_mesh": workspace_mesh,
                "stepper_mesh": stepper_mesh,
                "mesh_scale": LaunchConfiguration("mesh_scale"),
                "workspace_x": LaunchConfiguration("workspace_x"),
                "workspace_y": LaunchConfiguration("workspace_y"),
                "workspace_z": LaunchConfiguration("workspace_z"),
                "workspace_roll": LaunchConfiguration("workspace_roll"),
                "workspace_pitch": LaunchConfiguration("workspace_pitch"),
                "workspace_yaw": LaunchConfiguration("workspace_yaw"),
                "stepper_x": LaunchConfiguration("stepper_x"),
                "stepper_y": LaunchConfiguration("stepper_y"),
                "stepper_z": LaunchConfiguration("stepper_z"),
                "stepper_roll": LaunchConfiguration("stepper_roll"),
                "stepper_pitch": LaunchConfiguration("stepper_pitch"),
                "stepper_yaw": LaunchConfiguration("stepper_yaw"),
                "stepper_link": LaunchConfiguration("stepper_link"),
                "workspace_collision": LaunchConfiguration("workspace_collision"),
            }
        ],
        output="screen",
    )
    tool_tcp_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", LaunchConfiguration("tcp_x"),
            "--y", LaunchConfiguration("tcp_y"),
            "--z", LaunchConfiguration("tcp_z"),
            "--roll", LaunchConfiguration("tcp_roll"),
            "--pitch", LaunchConfiguration("tcp_pitch"),
            "--yaw", LaunchConfiguration("tcp_yaw"),
            "--frame-id", LaunchConfiguration("tcp_parent_link"),
            "--child-frame-id", LaunchConfiguration("goal_tcp_link"),
        ],
        output="screen",
    )
    goal_panel = Node(
        package="fanuc_scene_tools",
        executable="cartesian_goal_panel",
        parameters=[
            {
                "group_name": LaunchConfiguration("goal_group"),
                "tcp_link": LaunchConfiguration("goal_tcp_link"),
                "tcp_parent_link": LaunchConfiguration("tcp_parent_link"),
                "tcp_offset": tcp_offset_param(),
                "reference_frame": LaunchConfiguration("goal_frame"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_goal_panel")),
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("mesh_scale", default_value="0.001"),
        DeclareLaunchArgument("workspace_x", default_value="0.65"),
        DeclareLaunchArgument("workspace_y", default_value="0.0"),
        DeclareLaunchArgument("workspace_z", default_value="0.0"),
        DeclareLaunchArgument("workspace_roll", default_value="0.0"),
        DeclareLaunchArgument("workspace_pitch", default_value="0.0"),
        DeclareLaunchArgument("workspace_yaw", default_value="0.0"),
        DeclareLaunchArgument("stepper_x", default_value="0.0"),
        DeclareLaunchArgument("stepper_y", default_value="0.0"),
        DeclareLaunchArgument("stepper_z", default_value="0.0"),
        DeclareLaunchArgument("stepper_roll", default_value="0.0"),
        DeclareLaunchArgument("stepper_pitch", default_value="0.0"),
        DeclareLaunchArgument("stepper_yaw", default_value="0.0"),
        DeclareLaunchArgument("stepper_link", default_value="fanuc_flange"),
        DeclareLaunchArgument("load_stepper", default_value="true"),
        DeclareLaunchArgument("workspace_collision", default_value="true"),
        DeclareLaunchArgument("launch_goal_panel", default_value="false"),
        DeclareLaunchArgument("goal_group", default_value="manipulator"),
        DeclareLaunchArgument("goal_tcp_link", default_value="tool_tcp"),
        DeclareLaunchArgument("tcp_parent_link", default_value="fanuc_flange"),
        # The SRDF fixes the manipulator group's tip at "flange"; that is the frame
        # MoveIt Servo applies twists at, and there is no Servo parameter to change it.
        DeclareLaunchArgument("servo_ee_link", default_value="flange"),
        DeclareLaunchArgument("tcp_x", default_value="0.0"),
        DeclareLaunchArgument("tcp_y", default_value="0.0"),
        DeclareLaunchArgument("tcp_z", default_value="0.0"),
        DeclareLaunchArgument("tcp_roll", default_value="0.0"),
        DeclareLaunchArgument("tcp_pitch", default_value="0.0"),
        DeclareLaunchArgument("tcp_yaw", default_value="0.0"),
        DeclareLaunchArgument("goal_frame", default_value="world"),
        DeclareLaunchArgument("launch_servo", default_value="false"),
        DeclareLaunchArgument("launch_ortho_views", default_value="false"),
        DeclareLaunchArgument("launch_cbf", default_value="true"),
        DeclareLaunchArgument("cbf_prediction_horizon", default_value="0.20"),
        moveit_launch,
        scene_publisher,
        tool_tcp_frame,
        goal_panel,
        OpaqueFunction(function=servo_setup),
        OpaqueFunction(function=ortho_views_setup),
    ])