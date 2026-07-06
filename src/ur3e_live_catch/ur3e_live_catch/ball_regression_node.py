"""Ballistic-regression ball publisher (Isaac "pop" parity, archi §4.2/§7).

Sits between the raw ball source and ``live_catch_node``:

    tracker / test_ball_node  --ball_state_raw-->  THIS NODE  --ball_state-->  live_catch

Subscribes to raw ``BallState`` detections, transforms each sample to
``base_link`` at sample time (TF or identity), feeds ``BallRegression``
(``ball_regression.py``), and publishes a smoothed/predicted ``BallState`` on a
fixed timer: ``valid=False`` heartbeats while support is insufficient (same
idle contract as trigger-mode ``test_ball_node``), then position AND velocity
evaluated on the fit from the first valid tick.

Timestamp semantics: published ``header.stamp`` is the EVALUATION time (node
clock, now + lead_time_s) — deliberate latency compensation. Downstream
``perception_age_s`` therefore measures regression liveness (~0), not camera
latency; a dead camera mid-flight ends the flight through the coast timeout
instead of the staleness watchdog.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from ur3e_catch_msgs.msg import BallState
from ur3e_live_catch.ball_frame import BallFrameTransformer, FrameError, RigidTransform
from ur3e_live_catch.ball_regression import BallRegression, RegressionConfig


class BallRegressionNode(Node):
    def __init__(self) -> None:
        super().__init__("ball_regression_node")
        self.declare_parameter("input_topic", "ball_state_raw")
        self.declare_parameter("output_topic", "ball_state")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("units", "m")
        self.declare_parameter("rate_hz", 60.0)
        # Guard against grossly skewed producer stamps (event-clock drift,
        # use_sim_time mismatch): samples this far from the node clock are dropped.
        self.declare_parameter("max_stamp_age_s", 0.5)
        cfg = RegressionConfig()
        self.declare_parameter("gravity_m_s2", cfg.gravity_m_s2)
        self.declare_parameter("max_samples", cfg.max_samples)
        self.declare_parameter("irls_iterations", cfg.irls_iterations)
        self.declare_parameter("sigma_floor_m", cfg.sigma_floor_m)
        self.declare_parameter("recency_lambda", cfg.recency_lambda)
        self.declare_parameter("gate_floor_m", cfg.gate_floor_m)
        self.declare_parameter("gate_k", cfg.gate_k)
        self.declare_parameter("reorder_tolerance_s", cfg.reorder_tolerance_s)
        self.declare_parameter("min_sample_interval_s", cfg.min_sample_interval_s)
        self.declare_parameter("min_samples", cfg.min_samples)
        self.declare_parameter("min_span_s", cfg.min_span_s)
        self.declare_parameter("max_rms_m", cfg.max_rms_m)
        self.declare_parameter("min_speed_m_s", cfg.min_speed_m_s)
        self.declare_parameter("max_speed_m_s", cfg.max_speed_m_s)
        self.declare_parameter("require_approach", cfg.require_approach)
        self.declare_parameter("ground_z_m", cfg.ground_z_m)
        self.declare_parameter("min_pop_distance_m", cfg.min_pop_distance_m)
        self.declare_parameter("ballistic_check_span_s", cfg.ballistic_check_span_s)
        self.declare_parameter("ballistic_rms_ratio", cfg.ballistic_rms_ratio)
        self.declare_parameter("ballistic_rms_floor_m", cfg.ballistic_rms_floor_m)
        self.declare_parameter("collect_timeout_s", cfg.collect_timeout_s)
        self.declare_parameter("max_collect_span_s", cfg.max_collect_span_s)
        self.declare_parameter("coast_after_s", cfg.coast_after_s)
        self.declare_parameter("freeze_distance_m", cfg.freeze_distance_m)
        self.declare_parameter("reject_streak_n", cfg.reject_streak_n)
        self.declare_parameter("max_coast_s", cfg.max_coast_s)
        self.declare_parameter("coast_conf_tau_s", cfg.coast_conf_tau_s)
        self.declare_parameter("max_flight_s", cfg.max_flight_s)
        self.declare_parameter("refractory_s", cfg.refractory_s)
        self.declare_parameter("lead_time_s", cfg.lead_time_s)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if input_topic == output_topic:
            raise ValueError(
                f"input_topic == output_topic ({input_topic!r}): the regression "
                "node would feed itself; point the raw source elsewhere"
            )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._max_stamp_age = float(self.get_parameter("max_stamp_age_s").value)
        self._frame = BallFrameTransformer(
            self._base_frame, units=str(self.get_parameter("units").value)
        )
        self._logic = BallRegression(self._build_config())
        self._last_state = self._logic.state
        self._last_flights_ended = 0
        self._dropped_stamp = 0
        self._dropped_tf = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # Depth 50: the real tracker emits detections in bursts (event-driven,
        # up to ~1 kHz spurts) between our 60 Hz timer wakeups.
        self._sub = self.create_subscription(BallState, input_topic, self._on_raw, 50)
        self._pub = self.create_publisher(BallState, output_topic, 10)
        rate = float(self.get_parameter("rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"ball_regression_node: {input_topic!r} -> {output_topic!r}, "
            f"base_frame={self._base_frame!r}, rate={rate} Hz"
        )

    def _build_config(self) -> RegressionConfig:
        def p(name: str):
            return self.get_parameter(name).value

        return RegressionConfig(
            gravity_m_s2=float(p("gravity_m_s2")),
            max_samples=int(p("max_samples")),
            irls_iterations=int(p("irls_iterations")),
            sigma_floor_m=float(p("sigma_floor_m")),
            recency_lambda=float(p("recency_lambda")),
            gate_floor_m=float(p("gate_floor_m")),
            gate_k=float(p("gate_k")),
            reorder_tolerance_s=float(p("reorder_tolerance_s")),
            min_sample_interval_s=float(p("min_sample_interval_s")),
            min_samples=int(p("min_samples")),
            min_span_s=float(p("min_span_s")),
            max_rms_m=float(p("max_rms_m")),
            min_speed_m_s=float(p("min_speed_m_s")),
            max_speed_m_s=float(p("max_speed_m_s")),
            require_approach=bool(p("require_approach")),
            ground_z_m=float(p("ground_z_m")),
            min_pop_distance_m=float(p("min_pop_distance_m")),
            ballistic_check_span_s=float(p("ballistic_check_span_s")),
            ballistic_rms_ratio=float(p("ballistic_rms_ratio")),
            ballistic_rms_floor_m=float(p("ballistic_rms_floor_m")),
            collect_timeout_s=float(p("collect_timeout_s")),
            max_collect_span_s=float(p("max_collect_span_s")),
            coast_after_s=float(p("coast_after_s")),
            freeze_distance_m=float(p("freeze_distance_m")),
            reject_streak_n=int(p("reject_streak_n")),
            max_coast_s=float(p("max_coast_s")),
            coast_conf_tau_s=float(p("coast_conf_tau_s")),
            max_flight_s=float(p("max_flight_s")),
            refractory_s=float(p("refractory_s")),
            lead_time_s=float(p("lead_time_s")),
        )

    # --- raw sample intake ------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_raw(self, msg: BallState) -> None:
        if not msg.valid:
            return  # raw idle/dropped samples never feed the fit
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if abs(self._now_s() - stamp_s) > self._max_stamp_age:
            self._dropped_stamp += 1
            self.get_logger().warn(
                f"dropping raw sample with skewed stamp (|age|>{self._max_stamp_age}s)",
                throttle_duration_sec=2.0,
            )
            return
        transform = self._lookup_transform(msg.header.frame_id)
        try:
            pos_base = self._frame.to_base(
                (msg.position.x, msg.position.y, msg.position.z),
                msg.header.frame_id, transform,
            )
        except FrameError as exc:
            self.get_logger().warn(f"raw ball rejected: {exc}", throttle_duration_sec=2.0)
            return
        self._logic.add_sample(stamp_s, pos_base)
        self._log_state_change()

    def _lookup_transform(self, frame_id: str):
        if not frame_id or frame_id == self._base_frame:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(self._base_frame, frame_id, rclpy.time.Time())
        except Exception as exc:  # tf2 raises several lookup error types
            self._dropped_tf += 1
            self.get_logger().warn(
                f"no TF {frame_id!r} -> {self._base_frame!r}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return RigidTransform(translation=(t.x, t.y, t.z), quaternion=(q.x, q.y, q.z, q.w))

    # --- fixed-rate output --------------------------------------------------------

    def _tick(self) -> None:
        now_s = self._now_s()
        est = self._logic.step(now_s)
        self._log_state_change()
        msg = BallState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        if est.valid:
            msg.position.x, msg.position.y, msg.position.z = est.position
            msg.velocity.x, msg.velocity.y, msg.velocity.z = est.velocity
        msg.valid = est.valid
        msg.confidence = est.confidence
        self._pub.publish(msg)

    def _log_state_change(self) -> None:
        state = self._logic.state
        if state != self._last_state:
            fit = self._logic.fit
            detail = ""
            if fit is not None:
                detail = (f" (n={fit.n}, span={fit.span:.3f}s, rms={fit.rms:.3f}m,"
                          f" v0=({fit.v0[0]:.2f},{fit.v0[1]:.2f},{fit.v0[2]:.2f}))")
            self.get_logger().info(f"regression state: {self._last_state} -> {state}{detail}")
            self._last_state = state
        if self._logic.flights_ended != self._last_flights_ended:
            self._last_flights_ended = self._logic.flights_ended
            self._log_flight_summary()

    def _log_flight_summary(self) -> None:
        """One line per ended flight: the runbook's tuning/monitoring datum."""
        s = self._logic.last_flight_summary
        if s is None:
            return
        pop = s["pop_position"]
        pop_txt = "no-pop" if pop is None else f"({pop[0]:.2f},{pop[1]:.2f},{pop[2]:.2f})"
        lat = s["pop_latency_s"]
        lat_txt = "-" if lat is None else f"{lat * 1000:.0f}ms"
        rms = s["fit_rms_m"]
        rms_txt = "-" if rms is None else f"{rms:.3f}m"
        self.get_logger().info(
            f"flight summary: end={s['reason']} pop_latency={lat_txt} pop_pos={pop_txt} "
            f"accepted={s['n_accepted']} rejected={s['n_rejected']} rms={rms_txt} "
            f"dropped_tf={self._dropped_tf} dropped_stamp={self._dropped_stamp}"
        )
        self._dropped_tf = 0
        self._dropped_stamp = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallRegressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
