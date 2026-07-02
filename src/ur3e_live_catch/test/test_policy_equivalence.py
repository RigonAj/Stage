"""Policy-equivalence test — decides the SCALER question (archi §4.3.3, §12).

Feed each recorded ``observation`` to the exported policy and compare the output
to the recorded ``action_normalized``. If they match, the export already embeds
(or does not need) the SKRL ``RunningStandardScaler``. If they diverge, the
scaler must be applied before the net — wire an :class:`ObsScaler` and re-run.

Skipped automatically where torch (or the model) is unavailable — e.g. this
cloud environment. Run it on the ROS Humble machine.
"""

import json

import pytest

from conftest import repo_root
from ur3e_live_catch.policy_runtime import PolicyRunner, load_metadata, policy_model_candidates

torch = pytest.importorskip("torch", reason="torch not installed (run on the ROS machine)")

EXPORTS = "data/ur3e_rollouts/2026-05-26_17-13-29_ppo_torch/exports"


@pytest.fixture(scope="module")
def runner():
    model = repo_root() / EXPORTS / "policy_deterministic.ts"
    if not model.exists():
        pytest.skip(f"model not found: {model}")
    return PolicyRunner(model).load()


@pytest.fixture(scope="module")
def rollouts():
    return json.loads((repo_root() / EXPORTS / "rollouts_10_episodes.json").read_text())


def test_metadata_dims():
    meta = load_metadata(repo_root() / EXPORTS / "policy_metadata.json")
    assert meta["observation_space"] == 33
    assert meta["action_space"] == 6
    assert meta["action_scale"] == pytest.approx(0.5)


def test_explicit_onnx_model_falls_back_to_torchscript_sibling():
    assert policy_model_candidates("/tmp/model/policy_deterministic.onnx", []) == [
        "/tmp/model/policy_deterministic.onnx",
        "/tmp/model/policy_deterministic.ts",
    ]


def test_policy_reproduces_recorded_actions(runner, rollouts):
    max_abs = 0.0
    n = 0
    for ep in rollouts["episodes"]:
        for sample in ep["samples"][:5]:
            out = runner.infer(sample["observation"])
            assert len(out) == 6
            for a, b in zip(out, sample["action_normalized"]):
                max_abs = max(max_abs, abs(a - b))
            n += 1
    assert n > 0
    # A loose-but-meaningful bound: large divergence => the observation scaler is
    # NOT being applied (see module docstring) — wire ObsScaler and re-run.
    assert max_abs < 1e-2, (
        f"policy output diverges from recorded actions (max |delta|={max_abs:.4g}); "
        "the SKRL RunningStandardScaler is most likely missing from the export — "
        "provide policy_scaler.json and pass ObsScaler to PolicyRunner."
    )
