"""Exclusive-producer sanity checks (single ball source, single live node).

The live loop assumes exactly ONE producer per contract topic:

- ``ball_state``: one ball source (tracker, regression node, test ball or
  legacy adapter). A second producer interleaves its own messages (e.g. the
  virtual-ball stack's ``test_ball_node`` idle ``valid=false`` heartbeats)
  with the real flight samples; every interleaved invalid message triggers a
  controlled stop plus a policy-state reset, so the robot only ever twitches
  (2026-07-09 real Trace command test incident).
- ``catch_telemetry``: one ``live_catch_node``. A duplicate node (virtual-ball
  stack still running while a second live_catch launch is started) alternates
  ``command_enabled`` true/false in the Web UI and both nodes fight over
  controller switching.

These helpers are pure (no rclpy) so the trigger conditions stay
unit-testable without a ROS graph.
"""

from __future__ import annotations

from typing import Optional, Sequence


def producer_conflict_warnings(
    *,
    ball_topic: str,
    ball_publisher_count: int,
    telemetry_topic: str,
    telemetry_publisher_count: int,
    own_node_name: str,
    node_names: Optional[Sequence[str]] = None,
) -> list[str]:
    """Return human-readable conflict descriptions (empty list = healthy).

    ``node_names`` may be None when the graph query failed; the duplicate-name
    check is then skipped (the telemetry publisher count already covers the
    duplicate live node case on RMWs that collapse same-named graph entries).
    """
    warnings: list[str] = []
    if ball_publisher_count > 1:
        warnings.append(
            f"{ball_publisher_count} publishers on ball topic '{ball_topic}': only ONE "
            "ball producer may feed the live loop. Is the virtual-ball stack's "
            "test_ball_node still running next to the tracker/regression? Its idle "
            "valid=false heartbeats interleave with the real samples and reset the "
            "policy every tick, so the robot barely moves."
        )
    if telemetry_publisher_count > 1:
        warnings.append(
            f"{telemetry_publisher_count} publishers on telemetry topic "
            f"'{telemetry_topic}': another live_catch_node appears to be running "
            "(virtual-ball stack + manual live_catch launch?). The Web UI command "
            "state will flicker and both nodes may fight over controller switching. "
            "Stop one of them."
        )
    if node_names is not None:
        duplicates = sum(1 for name in node_names if name == own_node_name)
        if duplicates > 1:
            warnings.append(
                f"{duplicates} nodes named '{own_node_name}' discovered on the ROS "
                "graph: stop the duplicate before enabling command mode."
            )
    return warnings


def ball_producer_conflict(warnings: Sequence[str], ball_topic: str) -> bool:
    """True when the warning list contains a ball-topic producer conflict.

    Command emission fails closed on this specific conflict: with two ball
    producers the safe target alternates with hold/reset states, which reads
    as a twitching robot and can mask the real flight entirely.
    """
    needle = f"publishers on ball topic '{ball_topic}'"
    return any(needle in warning for warning in warnings)
