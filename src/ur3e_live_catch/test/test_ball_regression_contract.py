"""Wiring contract for the ballistic-regression ball publisher.

Source-text asserts (style of test_heartbeat_contract.py): config block, launch
insertion, packaging, and the live-node velocity contract.
"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parents[0]
CONFIG = PACKAGE_ROOT / "config" / "live_catch.yaml"
SETUP = PACKAGE_ROOT / "setup.py"
LIVE_NODE = PACKAGE_ROOT / "ur3e_live_catch" / "live_catch_node.py"
REGRESSION_NODE = PACKAGE_ROOT / "ur3e_live_catch" / "ball_regression_node.py"
LIVE_LAUNCH = PACKAGE_ROOT / "launch" / "live_catch.launch.py"
VIRTUAL_LAUNCH = PACKAGE_ROOT / "launch" / "virtual_ball_robot.launch.py"
TRACKER_NODE = WORKSPACE_SRC / "Ball_Tracking_Cpp" / "src" / "publisher_member_function.cpp"


def test_config_declares_regression_node_block():
    config = CONFIG.read_text(encoding="utf-8")

    assert "ball_regression_node:" in config
    assert 'input_topic: "ball_state_raw"' in config
    assert 'output_topic: "ball_state"' in config
    assert "rate_hz: 60.0" in config
    assert "ground_z_m: 0.05" in config  # Isaac ball_on_ground parity
    assert "use_ball_state_velocity: true" in config


def test_setup_exposes_regression_entry_point():
    setup = SETUP.read_text(encoding="utf-8")

    assert "ball_regression_node = ur3e_live_catch.ball_regression_node:main" in setup


def test_launch_repoints_raw_sources_when_regression_enabled():
    launch = LIVE_LAUNCH.read_text(encoding="utf-8")

    assert '"use_ball_regression", default_value="false"' in launch
    assert "'ball_state_raw' if '" in launch
    assert '{"ball_state_topic": raw_ball_topic}' in launch  # tracker
    assert '{"output_topic": raw_ball_topic}' in launch      # adapter + test ball
    assert 'executable="ball_regression_node"' in launch
    assert "IfCondition(use_ball_regression)" in launch


def test_virtual_ball_launch_forwards_regression_arg():
    launch = VIRTUAL_LAUNCH.read_text(encoding="utf-8")

    assert '"use_ball_regression"' in launch
    assert '"use_ball_regression": LaunchConfiguration("use_ball_regression")' in launch


def test_live_node_trusts_producer_velocity_and_resets_filter():
    source = LIVE_NODE.read_text(encoding="utf-8")

    assert 'self.declare_parameter("use_ball_state_velocity", True)' in source
    assert "producer_velocity(" in source
    assert "self._ball_frame.reset_velocity()" in source


def test_live_node_bounds_producer_inputs():
    source = LIVE_NODE.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")

    assert 'self.declare_parameter("max_ball_speed_m_s", 12.0)' in source
    assert 'self.declare_parameter("max_ball_distance_m", 4.0)' in source
    assert '"ball_out_of_range"' in source
    assert "max_ball_speed_m_s: 12.0" in config
    assert "max_ball_distance_m: 4.0" in config


def test_regression_robustness_gates_configured():
    config = CONFIG.read_text(encoding="utf-8")

    assert "min_pop_distance_m: 0.6" in config
    assert "ballistic_check_span_s: 0.15" in config
    assert "min_sample_interval_s: 0.003" in config
    # Current-state baseline: the fit already evaluates measurements at now.
    # A future horizon must never return as an unmeasured bring-up default.
    assert "lead_time_s: 0.0" in config
    assert "lead_time_s: 0.2" not in config


def test_tracker_reanchors_event_clock_after_gaps():
    tracker = TRACKER_NODE.read_text(encoding="utf-8")

    assert "REANCHOR_AFTER_GAP_S" in tracker
    assert "last_stamp_conversion_ros_time_" in tracker


def test_regression_node_guards_against_feedback_loop():
    source = REGRESSION_NODE.read_text(encoding="utf-8")

    assert "input_topic == output_topic" in source
    assert 'msg.header.frame_id = self._base_frame' in source


def test_tracker_pose_source_is_configurable_and_set_to_trace():
    config = CONFIG.read_text(encoding="utf-8")
    tracker = TRACKER_NODE.read_text(encoding="utf-8")

    assert 'pose_source: "trace"' in config
    assert 'declare_parameter<std::string>("pose_source", "circle")' in tracker
    assert "publishTracePose" in tracker
