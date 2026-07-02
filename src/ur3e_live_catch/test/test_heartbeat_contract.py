from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
CONFIG = PACKAGE_ROOT / "config" / "live_catch.yaml"
LIVE_NODE = PACKAGE_ROOT / "ur3e_live_catch" / "live_catch_node.py"
TEST_BALL_NODE = PACKAGE_ROOT / "ur3e_live_catch" / "test_ball_node.py"
CATCH_TELEMETRY_MSG = WORKSPACE_ROOT / "src" / "ur3e_catch_msgs" / "msg" / "CatchTelemetry.msg"


def test_catch_telemetry_declares_idle_heartbeat_flag():
    source = CATCH_TELEMETRY_MSG.read_text(encoding="utf-8")

    assert "bool                 command_enabled" in source
    assert "bool                 ball_valid" in source
    assert "false =>" in source


def test_live_node_publishes_idle_heartbeat_as_invalid_ball():
    source = LIVE_NODE.read_text(encoding="utf-8")

    assert "def _publish_heartbeat_telemetry" in source
    assert "obs=[]" in source
    assert "raw_action=[]" in source
    assert "ball_valid=False" in source


def test_virtual_ball_ground_termination_is_configured():
    config = CONFIG.read_text(encoding="utf-8")
    node = TEST_BALL_NODE.read_text(encoding="utf-8")

    assert "ground_z_m: 0.05" in config
    assert 'self.declare_parameter("ground_z_m", 0.05)' in node
    assert 'if base_pt[2] < float(self.get_parameter("ground_z_m").value):' in node
    assert "return self._to_output_frame(base_pt)" in node
