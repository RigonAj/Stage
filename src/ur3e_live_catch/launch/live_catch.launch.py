"""Live catch launch — the command-capable loop (archi steps 6-9).

Single entry point for staged bring-up (archi §11.9). The ball source is
selectable so the SAME node config is used from dry-run sim all the way to the
real robot:

    # 1. Dry-run with the simulated ball (nothing moves; inspect telemetry):
    ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=false

    # 2. Command on fake hardware / URSim with the simulated ball:
    ros2 launch ur3e_live_catch live_catch.launch.py use_test_ball:=true enable_command:=true

    # 3. Real perception (C++ tracker -> adapter) + command (KEEP THE E-STOP):
    ros2 launch ur3e_live_catch live_catch.launch.py use_adapter:=true enable_command:=true

``enable_command:=false`` (the default) keeps it a pure DRY-RUN: the full
perception->policy->safety pipeline runs and publishes CatchTelemetry, but no
robot command is emitted. Bring ``/joint_states`` up separately (use_fake_hardware
/ URSim / real driver).

The policy needs torch, which lives in the project ``.venv`` (NOT in the system
python that ``ros2 launch`` uses). Without it ``_make_policy()`` loads None and the
action is all-zeros, so the policy ghost never moves. ``_detect_policy_python()``
finds the venv interpreter and runs ``live_catch_node`` under it so inference
actually runs; override with the ``policy_python`` arg or ``LIVE_CATCH_PYTHON``.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _detect_policy_python() -> str:
    """Locate the project ``.venv`` interpreter that has torch.

    ``ros2 launch`` runs nodes with the system python, which has no torch here, so
    the policy silently loads as None and the action is all-zeros (the ghost never
    moves). Running ``live_catch_node`` under the venv interpreter fixes inference.
    Returns ``""`` when no venv is found (keeps the default interpreter).
    """
    override = os.environ.get("LIVE_CATCH_PYTHON", "")
    if override and os.path.isfile(override):
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
        if os.path.isfile(cand):
            return cand
    return ""


def generate_launch_description() -> LaunchDescription:
    policy_python_default = _detect_policy_python()
    interpreter_note = (
        f"live_catch_node interpreter: {policy_python_default} (torch should load)"
        if policy_python_default
        else "live_catch_node interpreter: system python -- WARNING: no .venv found, "
        "torch likely missing, the policy will infer zeros and the ghost will not move"
    )
    config = PathJoinSubstitution(
        [FindPackageShare("ur3e_live_catch"), "config", "live_catch.yaml"]
    )
    enable_command = LaunchConfiguration("enable_command")
    action_mode = LaunchConfiguration("action_mode")
    model_path = LaunchConfiguration("model_path")
    use_adapter = LaunchConfiguration("use_adapter")
    use_test_ball = LaunchConfiguration("use_test_ball")
    publish_frame = LaunchConfiguration("publish_frame")
    trigger_mode = LaunchConfiguration("trigger_mode")

    overrides = {
        "enable_command": ParameterValue(enable_command, value_type=bool),
        "action_mode": ParameterValue(action_mode, value_type=str),
        "model_path": ParameterValue(model_path, value_type=str),
    }

    return LaunchDescription([
        DeclareLaunchArgument("enable_command", default_value="false",
                              description="true => stream commands to the robot; false => dry-run"),
        DeclareLaunchArgument("action_mode", default_value="faithful",
                              description="faithful | safe (archi §4.3.4)"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="policy export; empty => data/models then dated fallback"),
        DeclareLaunchArgument("use_adapter", default_value="false",
                              description="start the Float32MultiArray->BallState adapter (real tracker)"),
        DeclareLaunchArgument("use_test_ball", default_value="false",
                              description="start the simulated test_ball_node instead of a real source"),
        DeclareLaunchArgument("publish_frame", default_value="base",
                              description="test_ball_node frame: base | <camera_frame> (archi §12)"),
        DeclareLaunchArgument("trigger_mode", default_value="false",
                              description="test_ball_node: true => throw on demand (~/throw service, web UI)"),
        DeclareLaunchArgument("policy_python", default_value=policy_python_default,
                              description="python interpreter for live_catch_node so torch (.venv) "
                                          "loads; empty => default interpreter (policy infers zeros)"),
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
            package="ur3e_live_catch",
            executable="float32_adapter",
            name="ball_float32_adapter",
            parameters=[config],
            output="screen",
            condition=IfCondition(use_adapter),
        ),
        Node(
            package="ur3e_live_catch",
            executable="test_ball_node",
            name="test_ball_node",
            parameters=[
                config,
                {"publish_frame": publish_frame},
                {"trigger_mode": ParameterValue(trigger_mode, value_type=bool)},
            ],
            output="screen",
            condition=IfCondition(use_test_ball),
        ),
    ])
