"""ActionMapper faithful vs safe (archi §4.3.4; user decision: flag, default faithful)."""

import json

import pytest

from conftest import repo_root
from ur3e_live_catch.action import ACTION_SCALE, ActionMapper

ROLLOUTS = (
    "data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/rollouts_10_episodes.json"
)
LATEST_METADATA = "data/models/latest/policy_metadata.json"


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


def test_incremental_reset_restarts_from_measured_pose():
    m = ActionMapper(mode="incremental", v_safe=[1.0] * 6, dt=0.1)
    assert m.map([1.0] * 6, [0.0] * 6) == pytest.approx([0.1] * 6)
    m.reset()
    assert m.map([-1.0] * 6, [0.5] * 6) == pytest.approx([0.4] * 6)


def test_latest_metadata_drives_current_incremental_contract():
    meta = json.loads((repo_root() / LATEST_METADATA).read_text())
    assert meta["observation_space"] == 33
    assert meta["action_space"] == 6
    assert meta["action_clip"] == [-1.0, 1.0]
    assert meta["disk_radius_m"] == pytest.approx(0.1)

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
