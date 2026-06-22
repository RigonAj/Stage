"""End-to-end command-pipeline logic, composed exactly as live_catch_node does.

ActionMapper -> SafetyLimiter -> CommandStreamer, off-robot (archi §4.3.4-§4.3.6,
step 6). The point of these tests is the safety contract: an aggressive policy in
``faithful`` mode emits a large ABSOLUTE target, yet the streamed command must
never jump — SafetyLimiter caps each step to v_safe*dt and a_safe*dt, independent
of the policy (defense in depth, archi §9).
"""

import math

import pytest

from ur3e_live_catch.action import ActionMapper
from ur3e_live_catch.limits import build_joint_bounds, nominal_ur3e_limits, v_safe_vector
from ur3e_live_catch.safety import SafetyLimiter
from ur3e_live_catch.streaming import CommandStreamer

DT = 1.0 / 60.0


def _pipeline(mode="faithful"):
    bounds = build_joint_bounds(nominal_ur3e_limits(), v_safe_factor=0.5, a_safe=1e9)
    mapper = ActionMapper(mode=mode, v_safe=v_safe_vector(bounds), dt=DT)
    safety = SafetyLimiter(bounds, DT)
    streamer = CommandStreamer()
    return bounds, mapper, safety, streamer


def test_faithful_absolute_target_is_rate_limited_no_jump():
    bounds, mapper, safety, streamer = _pipeline("faithful")
    q = [0.0] * 6
    raw = [10.0] * 6  # absurd absolute target = action*0.5 = 5 rad, way past one step
    target = mapper.map(raw, q)
    assert target == pytest.approx([5.0] * 6)  # faithful: absolute, unclipped
    report = safety.limit(target, q)
    cmd = streamer.format(report.target)
    # each joint moved at most v_safe*dt from q this tick
    for i, b in enumerate(bounds):
        assert abs(cmd[i] - q[i]) <= b.v_safe * DT + 1e-9
        assert report.rate_limited[i]
    # comp-9 feedback is the RAW action (training fidelity)
    assert mapper.prev_action == pytest.approx(raw)


def test_faithful_converges_to_absolute_target_over_time():
    bounds, mapper, safety, streamer = _pipeline("faithful")
    q = [0.0] * 6
    raw = [1.0] * 6           # target = 0.5 rad
    goal = [0.5] * 6
    for _ in range(2000):     # plenty of ticks to ramp in
        target = mapper.map(raw, q)
        report = safety.limit(target, q)
        q = streamer.format(report.target)  # robot perfectly tracks the command
    assert q == pytest.approx(goal, abs=1e-3)


def test_position_clip_keeps_command_in_range():
    bounds = build_joint_bounds(nominal_ur3e_limits(), v_safe_factor=0.5, a_safe=1e9)
    safety = SafetyLimiter(bounds, DT)
    q = [b.max_position - 1e-4 for b in bounds]  # right at the upper limit
    # huge push outward, slow enough not to be rate-limited first
    report = safety.limit([b.max_position + 100.0 for b in bounds], q)
    for i, b in enumerate(bounds):
        assert report.target[i] <= b.max_position + 1e-9


def test_safe_mode_is_clipped_increment():
    bounds, mapper, safety, streamer = _pipeline("safe")
    q = [0.1] * 6
    raw = [50.0] * 6  # clipped to +1
    target = mapper.map(raw, q)
    v_safe = v_safe_vector(bounds)
    assert target == pytest.approx([q[i] + 1.0 * v_safe[i] * DT for i in range(6)])
    # comp-9 feedback is the CLIPPED action in safe mode
    assert mapper.prev_action == pytest.approx([1.0] * 6)
