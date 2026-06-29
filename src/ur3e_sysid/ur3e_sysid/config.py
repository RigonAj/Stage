"""Shared constants for the UR3e system-id (doc §6.1, reference §1).

The canonical joint order is imported from :mod:`ur3e_live_catch.joint_order` so
sweeps, fits and the live path agree on indexing — a scrambled order would send
each joint the wrong angle / fit the wrong gain.
"""

from __future__ import annotations

import math

from ur3e_live_catch.joint_order import JOINT_ORDER

# Canonical UR3e joint order (shoulder_pan ... wrist_3).
JOINTS: tuple[str, ...] = tuple(JOINT_ORDER)

# Effort limits (Nm), ur_description joint_limits.yaml (reference §1). NOT the User
# Manual; the project historically used 9 Nm wrists vs 12 here — to reconfirm
# against the joint_limits.yaml actually shipped on the ROS machine (doc §6.1).
EFFORT_LIMIT: tuple[float, ...] = (56.0, 56.0, 28.0, 12.0, 12.0, 12.0)

# Velocity limits (rad/s), UR3e nominal (reference §1): 180 deg/s shoulder/elbow,
# 360 deg/s wrists.
VELOCITY_LIMIT: tuple[float, ...] = (
    math.pi, math.pi, math.pi, 2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi,
)

# Identification pose: mid-range, away from joint stops/singularities (doc §6.1).
# elbow at +pi/2 (well inside the +-pi software limit).
ID_POSE: tuple[float, ...] = (
    0.0, -math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0, -math.pi / 2.0, 0.0,
)

SIGNALS: tuple[str, ...] = ("step", "chirp", "ramp")


# short aliases ("elbow" -> "elbow_joint") for ergonomic CLI use (doc §6.1 examples).
_JOINT_ALIASES: dict[str, str] = {name.removesuffix("_joint"): name for name in JOINTS}


def resolve_joint(name: str) -> str:
    """Canonical joint name from a full or short name ('elbow' -> 'elbow_joint')."""
    if name in JOINTS:
        return name
    if name in _JOINT_ALIASES:
        return _JOINT_ALIASES[name]
    candidate = f"{name}_joint"
    if candidate in JOINTS:
        return candidate
    raise KeyError(f"unknown joint {name!r}; expected one of {list(JOINTS)} (or short form)")


def joint_index(joint: str) -> int:
    """Index of ``joint`` (full or short name) in the canonical order."""
    return JOINTS.index(resolve_joint(joint))
