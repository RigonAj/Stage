"""Live catch launch — the command-capable loop (archi steps 6-9).

Single entry point for staged bring-up (archi §11.9). The ball source is
selectable so the SAME node config is used from dry-run sim all the way to the
real robot:

    # 1. Dry-run with the simulated ball (nothing moves; inspect telemetry):
    ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=false

    # 2. Command on fake hardware / URSim with the simulated ball:
    ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=true

    # 3. Real perception (C++ tracker -> native BallState) + command (KEEP THE E-STOP):
    ros2 launch ur3e_live_catch live_catch.launch.py use_tracker:=true enable_command:=true

``enable_command:=false`` (the default) keeps it a pure DRY-RUN: the full
perception->policy->safety pipeline runs and publishes CatchTelemetry, but no
robot command is emitted. Bring ``/joint_states`` up separately (use_fake_hardware
/ URSim / real driver).

The policy needs either onnxruntime or torch. ``_detect_policy_python()`` only
selects a venv if it can import ROS 2 ``rclpy`` and one inference backend;
otherwise the node stays on the ROS Python interpreter. Override with the
``policy_python`` arg or ``LIVE_CATCH_PYTHON``.
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _python_can_run_policy(path: str) -> bool:
    probe = (
        "import importlib.util; import rclpy; "
        "raise SystemExit(0 if "
        "importlib.util.find_spec('onnxruntime') or importlib.util.find_spec('torch') "
        "else 1)"
    )
    result = subprocess.run(
        [path, "-c", probe],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _detect_policy_python() -> str:
    """Locate a compatible project ``.venv`` interpreter."""
    override = os.environ.get("LIVE_CATCH_PYTHON", "")
    if override and os.path.isfile(override) and _python_can_run_policy(override):
        return override
    candidates: list[str] = []
    root = os.environ.get("DV_ROSWS_ROOT", "")
    if root:
        candidates.append(os.path.join(root, ".venv", "bin", "python"))
    # Walk up from this launch file (realpath: --symlink-install points back to src).
    here = os.path.dirname(os.path.realpath(__file__))
    while True:
        candidates.append(os.path.join(here, ".venv", "bin", "python"))
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    for cand in candidates:
        if os.path.isfile(cand) and _python_can_run_policy(cand):
            return cand
    return ""


def generate_launch_description() -> LaunchDescription:
    policy_python_default = _detect_policy_python()
    interpreter_note = (
        f"live_catch_node interpreter: {policy_python_default} (policy backend should load)"
        if policy_python_default
        else "live_catch_node interpreter: system python"
    )
    config = PathJoinSubstitution(
        [FindPackageShare("ur3e_live_catch"), "config", "live_catch.yaml"]
    )
    enable_command = LaunchConfiguration("enable_command")
    action_mode = LaunchConfiguration("action_mode")
    model_path = LaunchConfiguration("model_path")
    use_tracker = LaunchConfiguration("use_tracker")
    use_adapter = LaunchConfiguration("use_adapter")
    use_test_ball = LaunchConfiguration("use_test_ball")
    publish_frame = LaunchConfiguration("publish_frame")
    trigger_mode = LaunchConfiguration("trigger_mode")
    use_ball_regression = LaunchConfiguration("use_ball_regression")
    ball_radius_mm = LaunchConfiguration("ball_radius_mm")
    camera_calibration_file = LaunchConfiguration("camera_calibration_file")
    # With the regression node enabled, every raw source is re-pointed to
    # ball_state_raw and the regression republishes the fitted BallState on
    # ball_state (live_catch_node.ball_topic stays untouched). With the arg
    # false this expression resolves to "ball_state": byte-identical wiring.
    raw_ball_topic = PythonExpression(
        ["'ball_state_raw' if '", use_ball_regression,
         "'.lower() in ('true', '1') else 'ball_state'"]
    )
    # Measurement purity: when the regression consumes the tracker output, the
    # tracker must publish measurements only — its lead/coast prediction is
    # pinned to 0 regardless of the requested values (prediction belongs to the
    # estimator; a lead > 0 publishes extrapolated points at confidence 1.0
    # that the regression cannot distinguish from measurements).
    tracker_lead_ms = PythonExpression(
        ["0.0 if '", use_ball_regression, "'.lower() in ('true', '1') else ",
         LaunchConfiguration("trace_lead_ms")]
    )
    tracker_hold_ms = PythonExpression(
        ["0.0 if '", use_ball_regression, "'.lower() in ('true', '1') else ",
         LaunchConfiguration("trace_hold_ms")]
    )

    overrides = {
        "enable_command": ParameterValue(enable_command, value_type=bool),
        "action_mode": ParameterValue(action_mode, value_type=str),
        "model_path": ParameterValue(model_path, value_type=str),
    }

    return LaunchDescription([
        DeclareLaunchArgument("enable_command", default_value="false",
                              description="true => stream commands to the robot; false => dry-run"),
        DeclareLaunchArgument("action_mode", default_value="faithful",
                              description="faithful resolves model metadata; safe is manual (archi §4.3.4)"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="policy export; empty => data/models then dated fallback"),
        DeclareLaunchArgument("use_tracker", default_value="false",
                              description="start ball_tracking_cpp native BallState publisher"),
        DeclareLaunchArgument("use_adapter", default_value="false",
                              description="start the legacy Float32MultiArray->BallState adapter; "
                                          "do not combine with use_tracker"),
        DeclareLaunchArgument("use_test_ball", default_value="false",
                              description="start the simulated test_ball_node instead of a real source"),
        DeclareLaunchArgument("publish_frame", default_value="base_link",
                              description="test_ball_node frame: base_link | <camera_frame> (archi §12)"),
        DeclareLaunchArgument("trigger_mode", default_value="false",
                              description="test_ball_node: true => throw on demand (~/throw service, web UI)"),
        DeclareLaunchArgument("use_ball_regression", default_value="false",
                              description="insert the ballistic-regression ball publisher: raw sources "
                                          "publish ball_state_raw, the fitted BallState lands on ball_state"),
        DeclareLaunchArgument("ball_radius_mm", default_value="20.0",
                              description="physical ball radius in millimetres for ball_tracking_cpp Trace depth"),
        DeclareLaunchArgument("trace_lead_ms", default_value="0.0",
                              description="tracker lead prediction (ms); forced to 0 when "
                                          "use_ball_regression is true (measurements only)"),
        DeclareLaunchArgument("trace_hold_ms", default_value="0.0",
                              description="tracker coast duration (ms); forced to 0 when "
                                          "use_ball_regression is true (measurements only)"),
        DeclareLaunchArgument("camera_calibration_file",
                              default_value="recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml",
                              description="OpenCV XML intrinsics consumed by ball_tracking_cpp"),
        DeclareLaunchArgument("use_reader", default_value="false",
                              description="tracker input: false = live DVXplorer camera, "
                                          "true = File/reader playback"),
        DeclareLaunchArgument("reader_file", default_value="",
                              description="recordings/ H5 file to replay through the tracker "
                                          "(forces reader mode, autoplay); empty = no replay"),
        DeclareLaunchArgument("policy_python", default_value=policy_python_default,
                              description="python interpreter for live_catch_node; empty => ROS Python"),
        LogInfo(msg=interpreter_note),
        Node(
            package="ur3e_live_catch",
            executable="live_catch_node",
            name="live_catch_node",
            parameters=[config, overrides],
            prefix=LaunchConfiguration("policy_python"),
            output="screen",
        ),
        Node(
            package="ball_tracking_cpp",
            executable="talker",
            name="ball_tracking_cpp",
            parameters=[
                config,
                {"ball_state_topic": raw_ball_topic},
                {"ball_radius_mm": ParameterValue(ball_radius_mm, value_type=float)},
                {"camera_calibration_file": ParameterValue(camera_calibration_file, value_type=str)},
                {"trace_lead_ms": ParameterValue(tracker_lead_ms, value_type=float)},
                {"trace_hold_ms": ParameterValue(tracker_hold_ms, value_type=float)},
                {"use_reader": ParameterValue(LaunchConfiguration("use_reader"), value_type=bool)},
                {"reader_file": ParameterValue(LaunchConfiguration("reader_file"), value_type=str)},
            ],
            output="screen",
            condition=IfCondition(use_tracker),
        ),
        Node(
            package="ur3e_live_catch",
            executable="float32_adapter",
            name="ball_float32_adapter",
            parameters=[config, {"output_topic": raw_ball_topic}],
            output="screen",
            condition=IfCondition(use_adapter),
        ),
        Node(
            package="ur3e_live_catch",
            executable="test_ball_node",
            name="test_ball_node",
            parameters=[
                config,
                {"output_topic": raw_ball_topic},
                {"publish_frame": publish_frame},
                {"trigger_mode": ParameterValue(trigger_mode, value_type=bool)},
            ],
            output="screen",
            condition=IfCondition(use_test_ball),
        ),
        Node(
            package="ur3e_live_catch",
            executable="ball_regression_node",
            name="ball_regression_node",
            parameters=[config],
            output="screen",
            condition=IfCondition(use_ball_regression),
        ),
    ])
