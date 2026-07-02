"""Dry-run launch: test_ball_node (publish_frame=base_link) + live_catch_node.

No robot command is emitted. ``/joint_states`` must come from somewhere — bring up
``use_fake_hardware:=true`` or URSim separately, or remap to a stub publisher.
Inspect with:  ros2 topic echo /ball_state   and   ros2 topic echo /catch_telemetry

Override params on the CLI, e.g. publish in the camera frame for the §12 parity test:
    ros2 launch ur3e_live_catch test_dry_run.launch.py publish_frame:=camera_optical
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("ur3e_live_catch"), "config", "live_catch.yaml"]
    )
    publish_frame = LaunchConfiguration("publish_frame")

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_frame", default_value="base_link",
            description="Frame the test ball is published in: base_link | <camera_frame>",
        ),
        Node(
            package="ur3e_live_catch",
            executable="test_ball_node",
            name="test_ball_node",
            parameters=[config, {"publish_frame": publish_frame}],
            output="screen",
        ),
        Node(
            package="ur3e_live_catch",
            executable="live_catch_node",
            name="live_catch_node",
            parameters=[config],
            output="screen",
        ),
    ])
