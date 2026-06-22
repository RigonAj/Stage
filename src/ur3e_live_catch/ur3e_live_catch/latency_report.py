"""Latency report node (archi §10, step 8).

Subscribes to ``CatchTelemetry`` and accumulates the two timing fields the live
node stamps each tick:
  - ``perception_age_s`` — now - BallState.stamp (how stale the ball estimate is);
  - ``loop_compute_s``   — hot-path compute time (obs -> policy -> safety).

Prints a rolling summary (count / mean / p50 / p95 / p99 / max, in ms) on a timer
and a final summary on shutdown. Compare the perception-age budget against the
latency modelled at training (sim-to-real §5.6): if it exceeds it, widen the
latency randomisation and retrain. Pure stats live in :mod:`latency` (tested).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from ur3e_catch_msgs.msg import CatchTelemetry
from ur3e_live_catch.latency import LatencyStats


class LatencyReportNode(Node):
    def __init__(self) -> None:
        super().__init__("latency_report")
        self.declare_parameter("telemetry_topic", "catch_telemetry")
        self.declare_parameter("report_period_s", 5.0)

        self._perception = LatencyStats()
        self._compute = LatencyStats()

        self.create_subscription(
            CatchTelemetry, str(self.get_parameter("telemetry_topic").value), self._on_telemetry, 10
        )
        self.create_timer(float(self.get_parameter("report_period_s").value), self._report)
        self.get_logger().info("latency_report listening on catch_telemetry")

    def _on_telemetry(self, msg: CatchTelemetry) -> None:
        # Fields exist once ur3e_catch_msgs is rebuilt; guard for older messages.
        if hasattr(msg, "perception_age_s"):
            self._perception.add(msg.perception_age_s)
        if hasattr(msg, "loop_compute_s"):
            self._compute.add(msg.loop_compute_s)

    def _report(self) -> None:
        self.get_logger().info(self._perception.format_ms("perception_age"))
        self.get_logger().info(self._compute.format_ms("loop_compute"))

    def destroy_node(self) -> bool:
        self.get_logger().info("FINAL " + self._perception.format_ms("perception_age"))
        self.get_logger().info("FINAL " + self._compute.format_ms("loop_compute"))
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LatencyReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
