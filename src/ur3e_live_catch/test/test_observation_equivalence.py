"""Offline observation-equivalence test (archi §12, roadmap step 4).

Reconstruct the 33-D observation and compare it, component by component, to the
``observation`` recorded in the Isaac rollouts. No Isaac source / torch / numpy
needed: the recorded observation IS the ground truth.

What is validated exactly here:
  - comp 1 joint_pos == recorded, fed from the INDEPENDENT joint_position_before_rad
  - comp 2 joint_vel == recorded, fed from joint_velocity_before_rad_s
  - comp 5 direction == ball - disk   (derivation)
  - comp 6 distance  == ||direction|| (derivation)
  - comp 9 actions   == previous sample's action_normalized (raw, unclipped)
  - structure: length 33 and slot placement

comps 3/8/10 (disk geometry, pass-through) are injected from the recorded obs and
will get their own bit-exact check once the Isaac geometry lands.
"""

import json

import pytest

from conftest import repo_root
from ur3e_live_catch.observation import (
    IDX_ACTIONS, IDX_BALL_POS, IDX_BALL_VEL, IDX_DIRECTION, IDX_DISK_POS,
    IDX_DISTANCE, IDX_JOINT_POS, IDX_JOINT_VEL, IDX_PASS_THROUGH, OBS_DIM,
    ObservationBuilder,
)

ROLLOUTS = (
    "data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports/rollouts_10_episodes.json"
)
TOL = 2e-4  # recorded values are float32


def _load_samples():
    path = repo_root() / ROLLOUTS
    data = json.loads(path.read_text())
    return data["episodes"]


def _close(a, b, tol=TOL):
    return all(abs(x - y) <= tol + tol * abs(y) for x, y in zip(a, b))


@pytest.fixture(scope="module")
def episodes():
    return _load_samples()


def test_dataset_present(episodes):
    assert episodes, "no episodes in rollouts file"
    assert len(episodes[0]["samples"]) > 1


def _blank_obs(builder, ball_pos):
    return builder.build(
        joint_pos=[0.0] * 6,
        joint_vel=[0.0] * 6,
        disk_pos=[0.0, 0.0, 0.0],
        disk_normal=[0.0, 0.0, 1.0],
        ball_pos=ball_pos,
        ball_vel=[0.0, 0.0, 0.0],
        prev_action=[0.0] * 6,
    )


def test_pass_through_matches_isaac_crossing_and_radius_gate():
    builder = ObservationBuilder(disk_radius=0.1)

    _blank_obs(builder, [0.0, 0.0, 0.05])
    obs = _blank_obs(builder, [0.09, 0.0, -0.05])
    assert obs[IDX_PASS_THROUGH] == 0.0  # count is emitted before this tick's update
    assert builder.pass_through_count == 1
    obs = _blank_obs(builder, [0.09, 0.0, -0.10])
    assert obs[IDX_PASS_THROUGH] == 1.0

    builder = ObservationBuilder(disk_radius=0.1)
    _blank_obs(builder, [0.0, 0.0, -0.05])
    _blank_obs(builder, [0.09, 0.0, 0.05])
    assert builder.pass_through_count == 1

    builder = ObservationBuilder(disk_radius=0.1)
    _blank_obs(builder, [0.0, 0.0, 0.05])
    _blank_obs(builder, [0.11, 0.0, -0.05])
    assert builder.pass_through_count == 0


def test_observation_equivalence(episodes):
    checked = 0
    for ep in episodes:
        builder = ObservationBuilder()
        prev_action = [0.0] * 6  # comp 9 at step 0 is zeros
        for sample in ep["samples"]:
            rec = sample["observation"]
            assert len(rec) == OBS_DIM

            disk = rec[IDX_DISK_POS:IDX_DISK_POS + 3]
            ball = rec[IDX_BALL_POS:IDX_BALL_POS + 3]
            ball_vel = rec[IDX_BALL_VEL:IDX_BALL_VEL + 3]

            obs = builder.build(
                joint_pos=sample["joint_position_before_rad"],
                joint_vel=sample["joint_velocity_before_rad_s"],
                disk_pos=disk,
                disk_normal=(0.0, 0.0, 1.0),
                ball_pos=ball,
                ball_vel=ball_vel,
                prev_action=prev_action,
            )

            assert len(obs) == OBS_DIM
            # comp 1 / 2: independent fields must equal the recorded slots
            assert _close(obs[IDX_JOINT_POS:IDX_JOINT_POS + 6], rec[IDX_JOINT_POS:IDX_JOINT_POS + 6])
            assert _close(obs[IDX_JOINT_VEL:IDX_JOINT_VEL + 6], rec[IDX_JOINT_VEL:IDX_JOINT_VEL + 6])
            # comp 5 / 6: derivations
            expect_dir = [ball[i] - disk[i] for i in range(3)]
            assert _close(obs[IDX_DIRECTION:IDX_DIRECTION + 3], expect_dir)
            assert abs(obs[IDX_DISTANCE] - (sum(d * d for d in expect_dir)) ** 0.5) <= TOL
            # comp 9: raw previous action
            assert _close(obs[IDX_ACTIONS:IDX_ACTIONS + 6], rec[IDX_ACTIONS:IDX_ACTIONS + 6])

            prev_action = sample["action_normalized"]
            checked += 1
    assert checked > 50, f"only {checked} samples checked"
