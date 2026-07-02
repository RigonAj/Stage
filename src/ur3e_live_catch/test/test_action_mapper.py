"""ActionMapper faithful vs safe (archi §4.3.4; user decision: flag, default faithful)."""

import json
import math

import pytest

from conftest import repo_root
from ur3e_live_catch.action import ACTION_SCALE, ActionMapper

ROLLOUTS = (
    "data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/rollouts_10_episodes.json"
)
LATEST_METADATA = "data/models/latest/policy_metadata.json"


def _clip(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


def _isaac_incremental_step(action, reference_q, prev_cmd_vel, *, v_safe, a_safe, lower, upper, dt):
    clipped_action = [_clip(float(a), -1.0, 1.0) for a in action]
    target = [0.0] * 6
    next_cmd_vel = [0.0] * 6
    for i in range(6):
        desired_delta_q = _clip(clipped_action[i] * v_safe[i] * dt, -v_safe[i] * dt, v_safe[i] * dt)
        desired_cmd_vel = desired_delta_q / dt
        distance_to_lower = max(reference_q[i] - lower[i], 0.0)
        distance_to_upper = max(upper[i] - reference_q[i], 0.0)
        accel_step = a_safe[i] * dt
        max_negative_stop_vel = max(
            -accel_step + math.sqrt(accel_step * accel_step + 2.0 * a_safe[i] * distance_to_lower),
            0.0,
        )
        max_positive_stop_vel = max(
            -accel_step + math.sqrt(accel_step * accel_step + 2.0 * a_safe[i] * distance_to_upper),
            0.0,
        )
        desired_cmd_vel = _clip(desired_cmd_vel, -max_negative_stop_vel, max_positive_stop_vel)
        cmd_vel_delta = _clip(desired_cmd_vel - prev_cmd_vel[i], -accel_step, accel_step)
        cmd_vel = _clip(prev_cmd_vel[i] + cmd_vel_delta, -v_safe[i], v_safe[i])
        target[i] = _clip(reference_q[i] + cmd_vel * dt, lower[i], upper[i])
        next_cmd_vel[i] = (target[i] - reference_q[i]) / dt
    return target, next_cmd_vel, clipped_action


def test_faithful_is_absolute_unclipped():
    m = ActionMapper(mode="faithful")
    action = [2.0, -3.0, 0.1, -1.5, 0.0, 4.2]
    q = [0.5] * 6
    target = m.map(action, q)
    assert target == pytest.approx([a * ACTION_SCALE for a in action])
    # comp-9 feedback is the RAW action
    assert m.prev_action == pytest.approx(action)


def test_safe_is_incremental_and_clipped():
    v_safe = [1.571, 1.571, 1.571, 3.142, 3.142, 3.142]
    dt = 1.0 / 60.0
    m = ActionMapper(mode="safe", v_safe=v_safe, dt=dt)
    action = [2.0, -2.0, 0.5, 0.0, -0.25, 10.0]  # some beyond [-1, 1]
    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    target = m.map(action, q)
    clipped = [1.0, -1.0, 0.5, 0.0, -0.25, 1.0]
    expect = [q[i] + clipped[i] * v_safe[i] * dt for i in range(6)]
    assert target == pytest.approx(expect)
    # comp-9 feedback is the CLIPPED action in safe mode
    assert m.prev_action == pytest.approx(clipped)


def test_safe_requires_v_safe():
    with pytest.raises(ValueError):
        ActionMapper(mode="safe")


def test_incremental_mirrors_current_isaac_integrator():
    dt = 1.0 / 60.0
    v_safe = [3.0] * 6
    a_safe = [12.0] * 6
    lower = [-1.0] * 6
    upper = [1.0] * 6
    m = ActionMapper(
        mode="incremental",
        v_safe=v_safe,
        a_safe=a_safe,
        position_lower=lower,
        position_upper=upper,
        dt=dt,
    )

    first = m.map([2.0] * 6, [0.0] * 6)
    second = m.map([2.0] * 6, first)

    # action is clipped to +1, then acceleration-limited:
    # tick 1 cmd_vel = 12 * dt = 0.2 rad/s -> step = 0.2 * dt
    # tick 2 cmd_vel = 0.4 rad/s -> cumulative target = first + 0.4 * dt
    assert first == pytest.approx([0.2 * dt] * 6)
    assert second == pytest.approx([(0.2 + 0.4) * dt] * 6)
    assert m.prev_action == pytest.approx([1.0] * 6)


def test_incremental_matches_isaac_formula_over_action_sequence():
    dt = 1.0 / 60.0
    v_safe = [3.1416, 3.1416, 3.1416, 6.2832, 6.2832, 6.2832]
    a_safe = [12.5664, 12.5664, 12.5664, 25.1328, 25.1328, 25.1328]
    lower = [-2.0 * math.pi, -2.0 * math.pi, -math.pi, -2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi]
    upper = [2.0 * math.pi, 2.0 * math.pi, math.pi, 2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi]
    actions = [
        [1.5, -2.0, 0.25, 0.0, -0.5, 2.0],
        [0.1, 0.2, -1.2, 1.0, -1.0, 0.0],
        [-1.0, 1.0, 1.0, -0.75, 0.4, -0.2],
    ]
    mapper = ActionMapper(
        mode="incremental",
        v_safe=v_safe,
        a_safe=a_safe,
        position_lower=lower,
        position_upper=upper,
        dt=dt,
    )
    reference_q = [0.0] * 6
    prev_cmd_vel = [0.0] * 6
    for action in actions:
        expected, prev_cmd_vel, clipped = _isaac_incremental_step(
            action,
            reference_q,
            prev_cmd_vel,
            v_safe=v_safe,
            a_safe=a_safe,
            lower=lower,
            upper=upper,
            dt=dt,
        )
        assert mapper.map(action, reference_q) == pytest.approx(expected)
        assert mapper.prev_action == pytest.approx(clipped)
        reference_q = expected


def test_incremental_reset_restarts_from_measured_pose():
    m = ActionMapper(mode="incremental", v_safe=[1.0] * 6, dt=0.1)
    assert m.map([1.0] * 6, [0.0] * 6) == pytest.approx([0.1] * 6)
    m.reset()
    assert m.map([-1.0] * 6, [0.5] * 6) == pytest.approx([0.4] * 6)


def test_incremental_reset_with_reference_discards_stale_target_and_velocity():
    m = ActionMapper(
        mode="incremental",
        v_safe=[3.0] * 6,
        a_safe=[12.0] * 6,
        position_lower=[-2.0] * 6,
        position_upper=[2.0] * 6,
        dt=0.1,
    )
    assert m.map([1.0] * 6, [0.0] * 6) == pytest.approx([0.12] * 6)

    m.reset(reference_q=[0.5] * 6)

    # Acceleration memory is reset, and the next integration reference is the
    # supplied measured/held pose, not the stale 0.12 target.
    assert m.map([1.0] * 6, [9.9] * 6) == pytest.approx([0.62] * 6)


def test_latest_metadata_drives_current_incremental_contract():
    meta = json.loads((repo_root() / LATEST_METADATA).read_text())
    assert meta["observation_space"] == 33
    assert meta["action_space"] == 6
    assert meta["action_clip"] == [-1.0, 1.0]
    assert meta["observation_frame"] == "base_link"
    assert meta["ball_position_frame"] == "base_link"
    assert meta["disk_position_frame"] == "base_link"
    assert meta["ball_velocity_frame"] == "world"
    assert meta["disk_offset_wrist_3_link_m"] == pytest.approx([-0.5, 0.0, 0.0])
    assert meta["disk_normal_wrist_3_link"] == pytest.approx([0.0, 0.0, -1.0])
    assert meta["disk_radius_m"] == pytest.approx(0.05)
    assert meta["ball_spawn_ranges_m"] == {
        "x": [-0.6, -0.2],
        "y": [1.2, 2.1],
        "z": [0.5, 1.2],
    }
    assert meta["ball_velocity_ranges_m_s"] == {
        "x": [-0.7, 0.6],
        "y": [-5.0, -3.5],
        "z": [-0.1, 1.5],
    }
    assert meta["ball_position_noise_std_m"] == pytest.approx(0.01)

    dt = float(meta["dt_s"])
    m = ActionMapper(
        mode="incremental",
        v_safe=meta["joint_velocity_safe_rad_s"],
        a_safe=meta["joint_acceleration_safe_rad_s2"],
        position_lower=meta["joint_position_lower_rad"],
        position_upper=meta["joint_position_upper_rad"],
        dt=dt,
    )
    first = m.map([2.0] * 6, [0.0] * 6)
    expect = [float(a) * dt * dt for a in meta["joint_acceleration_safe_rad_s2"]]
    assert first == pytest.approx(expect)
    assert m.prev_action == pytest.approx([1.0] * 6)


def test_faithful_matches_recorded_targets():
    """faithful target must reproduce joint_position_target_rad from the rollouts."""
    data = json.loads((repo_root() / ROLLOUTS).read_text())
    m = ActionMapper(mode="faithful")
    sample = data["episodes"][0]["samples"][3]
    target = m.map(sample["action_normalized"], [0.0] * 6)
    assert target == pytest.approx(sample["joint_position_target_rad"], abs=1e-4)
