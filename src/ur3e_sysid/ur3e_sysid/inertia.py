"""Effective per-joint inertia ``I_eff`` at the id pose (doc §4.5) — PURE (numpy/yaml).

We need ``I_eff[j]`` to turn the identified ``(wn, zeta)`` into the gains the sim
injects: ``K = I_eff*wn^2``, ``D = 2*zeta*wn*I_eff`` (doc §4.1). Pinocchio/KDL are not
installed, so this computes the **diagonal of the joint-space mass matrix** ``M(q)[j,j]``
directly from ``ur_description`` (forward kinematics from ``default_kinematics.yaml`` +
link masses/COM/inertia tensors from ``physical_parameters.yaml``):

    M[j,j] = sum over links i outboard of joint j of
             a_j . I_i(base) . a_j  +  m_i * || a_j x (c_i - o_j) ||^2

with ``a_j`` the joint axis (unit) and ``o_j`` its origin at the pose, ``c_i`` the link
COM and ``I_i(base)`` its inertia about the COM, both expressed in base. This is the
*exact* diagonal of M(q); it omits only the off-diagonal coupling (the standard
scalar "effective inertia" approximation). Config-dependent -> the pose is recorded in
the output. A full CRBA via Pinocchio is the precise upgrade (doc §4.5a, links).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

import numpy as np

DEFAULT_PHYSICAL_PARAMS = "/opt/ros/humble/share/ur_description/config/ur3e/physical_parameters.yaml"
DEFAULT_KINEMATICS = "/opt/ros/humble/share/ur_description/config/ur3e/default_kinematics.yaml"

# Canonical order (mirrors ur3e_live_catch.joint_order.JOINT_ORDER / config.JOINTS);
# kept local so this math module imports neither rclpy nor ur3e_live_catch.
JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
# ur_description keys for the 6 moving joints/links, proximal -> distal.
_CHAIN_KEYS = ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis rotation ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _rotz(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _tensor_matrix(t: dict) -> np.ndarray:
    return np.array(
        [
            [t["ixx"], t["ixy"], t["ixz"]],
            [t["ixy"], t["iyy"], t["iyz"]],
            [t["ixz"], t["iyz"], t["izz"]],
        ],
        dtype=float,
    )


def _load_yaml(path: Path | str) -> dict:
    import yaml  # lazy: only when computing inertia on the ROS machine

    with Path(path).open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def effective_inertia(
    pose: Sequence[float],
    *,
    physical_params_path: Path | str = DEFAULT_PHYSICAL_PARAMS,
    kinematics_path: Path | str = DEFAULT_KINEMATICS,
) -> dict[str, float]:
    """Return ``{joint_name: I_eff}`` (kg*m^2) at ``pose`` (canonical joint order)."""
    if len(pose) != 6:
        raise ValueError(f"pose must have 6 values, got {len(pose)}")
    kin = _load_yaml(kinematics_path)["kinematics"]
    phys = _load_yaml(physical_params_path)["inertia_parameters"]
    com = phys["center_of_mass"]
    rot = phys["rotation"]
    tens = phys["tensor"]

    # Forward kinematics: link/joint frame i = base * prod_{k<=i}(Origin_k * Rz(theta_k)).
    frames_R: list[np.ndarray] = []   # link frame orientation in base
    frames_p: list[np.ndarray] = []   # link frame origin in base
    masses: list[float] = []
    com_base: list[np.ndarray] = []   # link COM in base
    inertia_base: list[np.ndarray] = []  # link inertia about COM, in base

    R = np.eye(3)
    p = np.zeros(3)
    for i, key in enumerate(_CHAIN_KEYS):
        o = kin[key]
        Ro = _rpy(float(o["roll"]), float(o["pitch"]), float(o["yaw"]))
        po = np.array([float(o["x"]), float(o["y"]), float(o["z"])], dtype=float)
        p = p + R @ po
        R = R @ Ro
        # joint rotation about local Z (preserves Z axis and origin -> use this frame
        # for both the joint axis/origin and the attached link frame).
        R = R @ _rotz(float(pose[i]))
        frames_R.append(R.copy())
        frames_p.append(p.copy())

        cog = com[f"{key}_cog"]
        com_local = np.array([float(cog["x"]), float(cog["y"]), float(cog["z"])], dtype=float)
        com_base.append(p + R @ com_local)
        masses.append(float(phys[f"{key}_mass"]))

        rr = rot[key]
        R_rot = _rpy(float(rr["roll"]), float(rr["pitch"]), float(rr["yaw"]))
        I_local = R_rot @ _tensor_matrix(tens[key]) @ R_rot.T   # about COM, in link frame
        inertia_base.append(R @ I_local @ R.T)                  # about COM, in base

    out: dict[str, float] = {}
    for j in range(6):
        a_j = frames_R[j][:, 2]            # joint axis (unit) in base
        o_j = frames_p[j]                  # joint origin in base
        total = 0.0
        for i in range(j, 6):              # links outboard of joint j (incl. link j)
            total += float(a_j @ inertia_base[i] @ a_j)
            lever = np.cross(a_j, com_base[i] - o_j)
            total += masses[i] * float(lever @ lever)
        out[JOINT_NAMES[j]] = total
    return out
