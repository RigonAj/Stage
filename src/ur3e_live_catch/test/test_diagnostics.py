"""Exclusive-producer conflict detection (diagnostics.py).

The 2026-07-09 real Trace command test ran a manual live_catch launch next to
the still-running virtual-ball stack: two live_catch_node instances and two
ball_state producers. These tests pin the detection rules and the fail-closed
command gate condition.
"""

from ur3e_live_catch.diagnostics import ball_producer_conflict, producer_conflict_warnings


def _warnings(**overrides):
    kwargs = dict(
        ball_topic="ball_state",
        ball_publisher_count=1,
        telemetry_topic="catch_telemetry",
        telemetry_publisher_count=1,
        own_node_name="live_catch_node",
        node_names=["live_catch_node", "ball_tracking_cpp", "ur3e_web_ui"],
    )
    kwargs.update(overrides)
    return producer_conflict_warnings(**kwargs)


def test_single_producers_are_healthy():
    assert _warnings() == []


def test_extra_ball_producer_warns_and_fails_closed():
    warnings = _warnings(ball_publisher_count=2)
    assert len(warnings) == 1
    assert "ball topic 'ball_state'" in warnings[0]
    assert "test_ball_node" in warnings[0]  # points at the usual culprit
    assert ball_producer_conflict(warnings, "ball_state")


def test_duplicate_live_node_via_telemetry_publishers():
    warnings = _warnings(telemetry_publisher_count=2)
    assert len(warnings) == 1
    assert "telemetry topic 'catch_telemetry'" in warnings[0]
    # A telemetry conflict alone must NOT trip the ball fail-closed gate: the
    # OTHER node may be the commanding one; blocking here is a separate call.
    assert not ball_producer_conflict(warnings, "ball_state")


def test_duplicate_node_names_warn():
    warnings = _warnings(node_names=["live_catch_node", "live_catch_node"])
    assert len(warnings) == 1
    assert "nodes named 'live_catch_node'" in warnings[0]


def test_node_names_unavailable_skips_duplicate_check():
    assert _warnings(node_names=None) == []


def test_combined_conflicts_report_everything():
    warnings = _warnings(
        ball_publisher_count=2,
        telemetry_publisher_count=2,
        node_names=["live_catch_node", "live_catch_node"],
    )
    assert len(warnings) == 3
    assert ball_producer_conflict(warnings, "ball_state")


def test_zero_publishers_is_not_a_conflict():
    # Before the ball source starts there is nothing to warn about; staleness
    # is the watchdog's job, not the producer check's.
    assert _warnings(ball_publisher_count=0) == []
