"""Joint limits -> SafetyLimiter bounds (archi §4.3.5, §9, sim-to-real §2.1).

``v_safe`` is the per-joint rate cap enforced robot-side. Current Isaac exports
provide the exact velocity/acceleration bounds in ``policy_metadata.json`` and
``live_catch_node`` uses those metadata bounds for action mapping and safety. The
URDF/config path below is a fallback for legacy models or manual modes.

``a_safe`` (acceleration cap) is not carried in the URDF; it is a tuning knob
supplied by config (conservative by default).

Split in two:
  - :func:`build_joint_bounds` — PURE: turns a name->limit mapping into the ordered
    list of :class:`safety.JointBound`. Stdlib only, unit-tested off-robot.
  - :func:`load_ur3e_joint_limits` — reads ``ur_description/.../joint_limits.yaml``
    (PyYAML, imported lazily); falls back to the documented nominal limits when the
    file is absent so the package is self-sufficient.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence

from ur3e_live_catch.joint_order import JOINT_ORDER
from ur3e_live_catch.safety import JointBound


def _default_joint_limits_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("ur_description")) / "config" / "ur3e" / "joint_limits.yaml")
    except Exception:
        for distro in ("jazzy", "humble"):
            path = Path(f"/opt/ros/{distro}/share/ur_description/config/ur3e/joint_limits.yaml")
            if path.is_file():
                return str(path)
        return "/opt/ros/jazzy/share/ur_description/config/ur3e/joint_limits.yaml"


UR3E_JOINT_LIMITS_PATH = _default_joint_limits_path()

# Documented UR3e nominal limits (archi §4.3.5, §9). Used when the URDF yaml is
# unavailable. max_velocity in rad/s; position range ±2π (UR3e joints).
_NOMINAL_VEL = {
    "shoulder_pan_joint": math.pi,      # 180 deg/s
    "shoulder_lift_joint": math.pi,
    "elbow_joint": math.pi,
    "wrist_1_joint": 2.0 * math.pi,     # 360 deg/s
    "wrist_2_joint": 2.0 * math.pi,
    "wrist_3_joint": 2.0 * math.pi,
}
_NOMINAL_POS = 2.0 * math.pi


@dataclass(frozen=True)
class JointLimit:
    min_position: float
    max_position: float
    max_velocity: float


def build_joint_bounds(
    limits_by_name: Mapping[str, JointLimit],
    *,
    joint_order: Sequence[str] = JOINT_ORDER,
    v_safe_factor: float = 0.5,
    a_safe: float = 10.0,
) -> list[JointBound]:
    """Return ordered :class:`safety.JointBound` (one per joint, in ``joint_order``).

    ``v_safe = max_velocity * v_safe_factor`` (sim-to-real §2.1). Raises
    ``KeyError`` if a required joint is missing — never silently substitute a
    bound, which would let a joint run uncapped.
    """
    if not (0.0 < v_safe_factor <= 1.0):
        raise ValueError(f"v_safe_factor must be in (0, 1], got {v_safe_factor}")
    if a_safe <= 0.0:
        raise ValueError(f"a_safe must be > 0, got {a_safe}")
    bounds: list[JointBound] = []
    for joint in joint_order:
        if joint not in limits_by_name:
            raise KeyError(f"no joint limit for {joint!r}")
        lim = limits_by_name[joint]
        if not math.isfinite(lim.max_velocity) or lim.max_velocity <= 0.0:
            raise ValueError(f"joint {joint!r} has non-finite/zero max_velocity")
        bounds.append(
            JointBound(
                min_position=float(lim.min_position),
                max_position=float(lim.max_position),
                v_safe=float(lim.max_velocity) * float(v_safe_factor),
                a_safe=float(a_safe),
            )
        )
    return bounds


def v_safe_vector(
    bounds: Sequence[JointBound],
) -> list[float]:
    """Per-joint ``v_safe`` list (for ActionMapper 'safe' mode), from built bounds."""
    return [b.v_safe for b in bounds]


def scale_bounds(bounds: Sequence[JointBound], scale: float) -> list[JointBound]:
    """Scale ``v_safe``/``a_safe`` of metadata bounds by a test factor.

    Position limits are untouched. The ActionMapper integrator and SafetyLimiter
    share the returned bounds. ``scale=1`` is the trained metadata contract;
    values below 1 are bring-up slowdowns, and values above 1 are deliberate
    lab-test overdrives that can exceed metadata/UR nominal velocity limits.
    """
    if not (0.0 < scale <= 4.0):
        raise ValueError(f"scale must be in (0, 4], got {scale}")
    if scale == 1.0:
        return list(bounds)
    return [
        JointBound(
            min_position=b.min_position,
            max_position=b.max_position,
            v_safe=b.v_safe * scale,
            a_safe=b.a_safe * scale,
        )
        for b in bounds
    ]


def nominal_ur3e_limits() -> dict[str, JointLimit]:
    """The documented UR3e nominal limits, used as a fallback."""
    return {
        name: JointLimit(min_position=-_NOMINAL_POS, max_position=_NOMINAL_POS, max_velocity=vel)
        for name, vel in _NOMINAL_VEL.items()
    }


def load_ur3e_joint_limits(path: Path | str = UR3E_JOINT_LIMITS_PATH) -> dict[str, JointLimit]:
    """Load ``ur_description`` joint_limits.yaml; fall back to nominal if missing.

    PyYAML is imported lazily so importing this module never requires it (the pure
    :func:`build_joint_bounds` path and the off-robot tests stay dependency-free).
    The ``!degrees`` tag used by ur_description is converted to radians.
    """
    p = Path(path)
    if not p.is_file():
        default_path = Path(UR3E_JOINT_LIMITS_PATH)
        if default_path.is_file():
            p = default_path
    if not p.is_file():
        return nominal_ur3e_limits()
    try:
        import yaml  # lazy: only needed on the ROS machine
    except ImportError:
        # No PyYAML (e.g. a minimal venv): fall back to the documented nominal
        # UR3e limits rather than crashing. The nominal max_velocity equals the
        # URDF value; only fine-grained position limits would differ.
        return nominal_ur3e_limits()

    class _DegreesLoader(yaml.SafeLoader):
        pass

    _DegreesLoader.add_constructor(
        "!degrees",
        lambda loader, node: math.radians(float(loader.construct_scalar(node))),
    )

    data = yaml.load(p.read_text(), Loader=_DegreesLoader)
    out: dict[str, JointLimit] = {}
    for name, entry in data.get("joint_limits", {}).items():
        if entry.get("has_position_limits", False):
            lo = float(entry["min_position"])
            hi = float(entry["max_position"])
        else:
            lo, hi = -_NOMINAL_POS, _NOMINAL_POS
        vmax = float(entry["max_velocity"]) if entry.get("has_velocity_limits", False) else math.inf
        out[name] = JointLimit(min_position=lo, max_position=hi, max_velocity=vmax)
    # Backfill any joint absent from the file with the nominal value.
    for name, lim in nominal_ur3e_limits().items():
        out.setdefault(name, lim)
    return out
