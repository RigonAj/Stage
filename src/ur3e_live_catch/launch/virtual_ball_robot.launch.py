"""Real-robot virtual-ball bring-up.

This is the one-command entry point for the web UI Test tab:

    ros2 launch ur3e_live_catch virtual_ball_robot.launch.py

It starts the UR3e driver, the live-catch dry-run chain with ``test_ball_node``
in trigger mode, and the web UI. By default ``enable_command`` is false, so the
policy/safety telemetry and ghost target are visible but no robot command is
emitted until the UI explicitly enables command mode.

For real-perception sessions swap the ball source INSIDE this stack instead of
starting a second live_catch launch next to it (that creates two
live_catch_node instances and two ball_state producers — 2026-07-09 incident):

    ros2 launch ur3e_live_catch virtual_ball_robot.launch.py \
        use_test_ball:=false use_tracker:=true use_ball_regression:=true
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_calibration_file() -> str:
    path = os.path.expanduser("~/ur3e_calibration.yaml")
    if os.path.isfile(path):
        return path
    return os.path.join(
        get_package_share_directory("ur_description"),
        "config",
        "ur3e",
        "default_kinematics.yaml",
    )


def _launch_driver(context, *_, **__):
    launch_args = {
        "ur_type": "ur3e",
        "robot_ip": LaunchConfiguration("robot_ip").perform(context),
        "reverse_ip": LaunchConfiguration("reverse_ip").perform(context),
        "use_mock_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
        "headless_mode": LaunchConfiguration("headless_mode").perform(context),
        "initial_joint_controller": "scaled_joint_trajectory_controller",
        "launch_rviz": "false",
    }
    kinematics = LaunchConfiguration("kinematics_params_file").perform(context).strip()
    if kinematics:
        launch_args["kinematics_params_file"] = kinematics

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
                )
            ),
            launch_arguments=launch_args.items(),
        )
    ]


def _split_vector(raw: str, expected: int, name: str) -> list[str]:
    parts = raw.replace(",", " ").split()
    if len(parts) != expected:
        raise RuntimeError(f"{name} must contain {expected} numeric values, got {raw!r}")
    for part in parts:
        float(part)
    return parts


# Isaac-matched hoop geometry per racket hold side (seen from in front of the
# robot). "left" is the right-hand setup rotated 180 deg about wrist_3 Z:
# translation x flips sign and the frame rotation becomes 180 deg about Y
# (Rz(pi) * Rx(pi)); both map hoop +Z to the Isaac disk normal -Z.
HOOP_SIDE_DEFAULTS = {
    "right": ("-0.5 0.0 0.0", "1.0 0.0 0.0 0.0"),
    "left": ("0.5 0.0 0.0", "0.0 1.0 0.0 0.0"),
}


def _launch_hoop_tf(context, *_, **__):
    enabled = LaunchConfiguration("publish_hoop_tf").perform(context).lower()
    if enabled not in ("1", "true", "yes", "on"):
        return []

    side = LaunchConfiguration("hold_side").perform(context).strip().lower()
    if side not in HOOP_SIDE_DEFAULTS:
        raise RuntimeError(f"hold_side must be 'right' or 'left', got {side!r}")
    default_xyz, default_quat = HOOP_SIDE_DEFAULTS[side]
    raw_xyz = LaunchConfiguration("hoop_xyz").perform(context).strip() or default_xyz
    raw_quat = LaunchConfiguration("hoop_quat").perform(context).strip() or default_quat
    xyz = _split_vector(raw_xyz, 3, "hoop_xyz")
    quat = _split_vector(raw_quat, 4, "hoop_quat")
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="hoop_center_static_tf",
            arguments=xyz + quat + ["wrist_3_link", "hoop_center"],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    live_catch_launch = PathJoinSubstitution(
        [FindPackageShare("ur3e_live_catch"), "launch", "live_catch.launch.py"]
    )
    moveit_launch = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.0.5",
                description="UR3e robot IP address.",
            ),
            DeclareLaunchArgument(
                "reverse_ip",
                default_value="192.168.0.3",
                description="ROS PC IP used by the robot reverse interface.",
            ),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=_default_calibration_file(),
                description=(
                    "UR calibration YAML. Defaults to ~/ur3e_calibration.yaml when "
                    "present, otherwise ur_description's UR3e default kinematics."
                ),
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="true => start the UR driver with fake hardware.",
            ),
            DeclareLaunchArgument(
                "headless_mode",
                default_value="false",
                description="UR driver headless mode for External Control.",
            ),
            DeclareLaunchArgument(
                "launch_moveit",
                default_value="true",
                description="Start MoveIt move_group for the UI TCP Target tab.",
            ),
            DeclareLaunchArgument(
                "launch_ui",
                default_value="true",
                description="Start ur3e_web_ui.",
            ),
            DeclareLaunchArgument(
                "ui_host",
                default_value="127.0.0.1",
                description="Web UI bind address.",
            ),
            DeclareLaunchArgument(
                "ui_port",
                default_value="8080",
                description="Web UI port.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Policy export; empty => ur3e_live_catch default lookup.",
            ),
            DeclareLaunchArgument(
                "action_mode",
                default_value="faithful",
                description="Action mapping mode: faithful resolves model metadata; safe is manual.",
            ),
            DeclareLaunchArgument(
                "enable_command",
                default_value="false",
                description="false => dry-run; true => live_catch may stream robot commands.",
            ),
            DeclareLaunchArgument(
                "publish_frame",
                default_value="base_link",
                description="Frame used by test_ball_node: base_link | <camera_frame>.",
            ),
            DeclareLaunchArgument(
                "trigger_mode",
                default_value="true",
                description="true => virtual ball flies only after /test_ball_node/throw.",
            ),
            # Exactly ONE ball source may publish toward the live loop: keep
            # use_test_ball and use_tracker mutually exclusive. Running the
            # stack's test_ball next to a real tracker interleaves valid=false
            # heartbeats with the flight and stalls the robot (2026-07-09).
            DeclareLaunchArgument(
                "use_test_ball",
                default_value="true",
                description=(
                    "Start the simulated test_ball_node (virtual ball). Set to false "
                    "when using the real tracker (use_tracker:=true)."
                ),
            ),
            DeclareLaunchArgument(
                "use_tracker",
                default_value="false",
                description=(
                    "Start ball_tracking_cpp (real DVXplorer Trace perception) as the "
                    "ball source instead of the virtual ball. Combine with "
                    "use_test_ball:=false and preferably use_ball_regression:=true."
                ),
            ),
            DeclareLaunchArgument(
                "camera_calibration_file",
                default_value=(
                    "recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml"
                ),
                description="OpenCV XML intrinsics for ball_tracking_cpp (use_tracker).",
            ),
            DeclareLaunchArgument(
                "ball_radius_mm",
                default_value="20.0",
                description=(
                    "Physical ball radius in millimetres for ball_tracking_cpp Trace "
                    "depth (use_tracker). Depth scales directly with this value."
                ),
            ),
            DeclareLaunchArgument(
                "use_ball_regression",
                default_value="false",
                description=(
                    "Insert the ballistic-regression ball publisher between the "
                    "raw ball source and live_catch (Isaac pop parity)."
                ),
            ),
            DeclareLaunchArgument(
                "launch_latency_report",
                default_value="false",
                description="Start ur3e_live_catch latency_report.",
            ),
            DeclareLaunchArgument(
                "publish_hoop_tf",
                default_value="true",
                description=(
                    "Publish wrist_3_link -> hoop_center static TF using the "
                    "Isaac-matched hoop geometry unless overridden."
                ),
            ),
            DeclareLaunchArgument(
                "hold_side",
                default_value="right",
                description=(
                    "Racket hold side seen from in front of the robot: right "
                    "(historical, hoop at -0.5 m on wrist_3 X) or left "
                    "(180 deg about wrist_3 Z, hoop at +0.5 m). Must match "
                    "the physical mount and the loaded model's hold_side."
                ),
            ),
            DeclareLaunchArgument(
                "hoop_xyz",
                default_value="",
                description=(
                    "Static hoop translation x y z in wrist_3_link, metres. "
                    "Empty => derived from hold_side."
                ),
            ),
            DeclareLaunchArgument(
                "hoop_quat",
                default_value="",
                description=(
                    "Static hoop quaternion qx qy qz qw in wrist_3_link; maps "
                    "+Z to Isaac normal -Z. Empty => derived from hold_side."
                ),
            ),
            OpaqueFunction(function=_launch_driver),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "ur_type": "ur3e",
                    "launch_rviz": "false",
                    "launch_servo": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_moveit")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(live_catch_launch),
                launch_arguments={
                    "use_test_ball": LaunchConfiguration("use_test_ball"),
                    "use_tracker": LaunchConfiguration("use_tracker"),
                    "camera_calibration_file": LaunchConfiguration("camera_calibration_file"),
                    "ball_radius_mm": LaunchConfiguration("ball_radius_mm"),
                    "use_adapter": "false",
                    "trigger_mode": LaunchConfiguration("trigger_mode"),
                    "publish_frame": LaunchConfiguration("publish_frame"),
                    "use_ball_regression": LaunchConfiguration("use_ball_regression"),
                    "enable_command": LaunchConfiguration("enable_command"),
                    "action_mode": LaunchConfiguration("action_mode"),
                    "model_path": LaunchConfiguration("model_path"),
                }.items(),
            ),
            OpaqueFunction(function=_launch_hoop_tf),
            Node(
                package="ur3e_live_catch",
                executable="latency_report",
                name="latency_report",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_latency_report")),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ur3e_web_ui",
                    "ur3e_web_ui",
                    "--host",
                    LaunchConfiguration("ui_host"),
                    "--port",
                    LaunchConfiguration("ui_port"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_ui")),
            ),
        ]
    )
