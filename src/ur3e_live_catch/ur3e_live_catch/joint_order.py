"""Canonical UR3e joint order, shared by every module on the live path.

This MUST match three independent sources or the policy receives a scrambled
observation:
  - the training export ``policy_metadata.json`` ``joint_names``;
  - the replay package ``ur3e_rollout_replay/replay_core.py`` ``DEFAULT_JOINT_NAMES``;
  - the order assumed by ``firsttraining_env`` when it builds the observation.

``/joint_states`` does NOT guarantee this order, so callers reorder by name with
:func:`reorder_by_name` before doing anything else (archi §4.3, §6 comp 1/2).
"""

from __future__ import annotations

from typing import Mapping, Sequence

JOINT_ORDER: tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class JointOrderError(KeyError):
    """A required joint is missing from the source message."""


def reorder_by_name(
    names: Sequence[str],
    values: Sequence[float],
    target: Sequence[str] = JOINT_ORDER,
) -> list[float]:
    """Return ``values`` re-indexed from ``names`` into ``target`` order.

    Raises :class:`JointOrderError` if a target joint is absent — we never
    silently drop or zero a joint, which would corrupt the observation.
    """
    if len(names) != len(values):
        raise ValueError(f"names ({len(names)}) and values ({len(values)}) length mismatch")
    index: Mapping[str, int] = {name: i for i, name in enumerate(names)}
    out: list[float] = []
    for joint in target:
        if joint not in index:
            raise JointOrderError(f"joint {joint!r} not present in {list(names)!r}")
        out.append(float(values[index[joint]]))
    return out
