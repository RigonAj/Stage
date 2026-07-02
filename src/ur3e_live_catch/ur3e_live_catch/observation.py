"""33-D observation reconstruction — mirror of ``firsttraining_env`` (archi §6).

Exact order (sum = 6+6+3+3+3+1+3+1+6+1 = 33), with slice indices:

    [ 0: 6]  1  joint_pos        (rad)
    [ 6:12]  2  joint_vel        (rad/s)
    [12:15]  3  disk_pos_local   (m)   hoop centre in base_link
    [15:18]  4  ball_pos_local   (m)   ball in base_link
    [18:21]  5  direction        (m)   ball - disk
    [21:22]  6  distance         (m)   ||direction||
    [22:25]  7  ball_vel_w       (m/s) filtered
    [25:26]  8  prev_signed_dist > 0   (0/1)               <- needs Isaac sign
    [26:32]  9  actions          previous policy action per model contract
    [32:33] 10  pass_through_count                          <- needs Isaac logic

Units are strict: rad, rad/s, m, m/s — no degrees anywhere (archi §6).

Components 3, 8 and 10 depend on the disk/ball geometry of the trained task.
They are parameterised by the same disk pose, normal and trigger radius used by
Isaac's ``_read_disk_pose_in_body_frame`` / ``detect_pass_through``.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

Vec3 = tuple[float, float, float]

OBS_DIM = 33

# Slice layout (start indices), kept as named constants for tests and telemetry.
IDX_JOINT_POS = 0
IDX_JOINT_VEL = 6
IDX_DISK_POS = 12
IDX_BALL_POS = 15
IDX_DIRECTION = 18
IDX_DISTANCE = 21
IDX_BALL_VEL = 22
IDX_SIGNED_FLAG = 25
IDX_ACTIONS = 26
IDX_PASS_THROUGH = 32


def _sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _mul(v: Sequence[float], scalar: float) -> Vec3:
    return (v[0] * scalar, v[1] * scalar, v[2] * scalar)


def _unit(v: Sequence[float]) -> Vec3:
    n = _norm(v)
    if n <= 1.0e-12:
        raise ValueError("disk_normal must be non-zero")
    return (float(v[0]) / n, float(v[1]) / n, float(v[2]) / n)


class ObservationBuilder:
    """Stateful builder: keeps the signed-distance sign and pass-through count.

    ``prev_action`` (component 9) is supplied per tick by the caller (the last
    action value according to the model contract). Legacy absolute exports feed
    back the raw action; current incremental Isaac exports feed back Isaac's
    clipped ``self.actions`` tensor.
    """

    def __init__(self, disk_radius: float = 0.05) -> None:
        # disk_radius gates pass-through to crossings *inside* the hoop. The
        # current FirstTraining config sets this to 0.05 m; the mesh radius is
        # larger, but firsttraining_env uses cfg.disk_radius when it is > 0.
        self.disk_radius = float(disk_radius)
        self._prev_signed_dist: float = 0.0
        self._pass_through_count: int = 0

    def reset(self) -> None:
        self._prev_signed_dist = 0.0
        self._pass_through_count = 0

    @property
    def pass_through_count(self) -> int:
        return self._pass_through_count

    # --- geometry-dependent pieces (to finalise from the Isaac source) --------

    @staticmethod
    def disk_pos_local(disk_pos_base: Sequence[float]) -> Vec3:
        """Component 3. In sim, "local" = world - env_origin = base_link, so the
        hoop centre expressed in base_link IS disk_pos_local."""
        return (float(disk_pos_base[0]), float(disk_pos_base[1]), float(disk_pos_base[2]))

    @staticmethod
    def signed_distance(ball_base: Sequence[float], disk_base: Sequence[float],
                        disk_normal: Sequence[float]) -> float:
        """Signed distance of the ball to the disk plane along its normal.

        ``disk_normal`` is normalized by :meth:`build`, matching Isaac's
        ``_read_disk_pose_in_body_frame`` output.
        """
        return _dot(_sub(ball_base, disk_base), disk_normal)

    def _update_pass_through(self, signed_dist: float, ball_base: Sequence[float],
                             disk_base: Sequence[float], disk_normal: Sequence[float]) -> None:
        """Component 10. Mirror Isaac ``detect_pass_through``."""
        crossed = self._prev_signed_dist * signed_dist < 0.0
        if crossed and self.disk_radius > 0.0:
            ball_relative = _sub(ball_base, disk_base)
            along_normal = _mul(disk_normal, signed_dist)
            radial = _sub(ball_relative, along_normal)
            if _norm(radial) >= self.disk_radius:
                crossed = False
        if crossed:
            self._pass_through_count += 1

    # --- full observation -----------------------------------------------------

    def build(
        self,
        *,
        joint_pos: Sequence[float],
        joint_vel: Sequence[float],
        disk_pos: Sequence[float],
        disk_normal: Sequence[float],
        ball_pos: Sequence[float],
        ball_vel: Sequence[float],
        prev_action: Sequence[float],
    ) -> list[float]:
        if len(joint_pos) != 6 or len(joint_vel) != 6 or len(prev_action) != 6:
            raise ValueError("joint_pos, joint_vel and prev_action must each have 6 elements")

        disk_local = self.disk_pos_local(disk_pos)
        disk_normal_unit = _unit(disk_normal)
        ball_local = (float(ball_pos[0]), float(ball_pos[1]), float(ball_pos[2]))
        direction = _sub(ball_local, disk_local)
        distance = _norm(direction)

        # Isaac updates prev_disk_signed_dist/pass_through_count in _get_dones()
        # before the next observation is emitted. Mirror that order here.
        current_signed = self.signed_distance(ball_local, disk_local, disk_normal_unit)
        self._update_pass_through(current_signed, ball_local, disk_local, disk_normal_unit)
        signed_flag = 1.0 if current_signed > 0.0 else 0.0

        obs: list[float] = []
        obs.extend(float(v) for v in joint_pos)          # 1
        obs.extend(float(v) for v in joint_vel)          # 2
        obs.extend(disk_local)                            # 3
        obs.extend(ball_local)                            # 4
        obs.extend(direction)                             # 5
        obs.append(distance)                              # 6
        obs.extend(float(v) for v in ball_vel)           # 7
        obs.append(signed_flag)                           # 8
        obs.extend(float(v) for v in prev_action)        # 9 (contract-specific action feedback)
        obs.append(float(self._pass_through_count))       # 10

        # Store the signed distance for the next crossing test.
        self._prev_signed_dist = current_signed

        if len(obs) != OBS_DIM:
            raise AssertionError(f"observation length {len(obs)} != {OBS_DIM}")
        return obs
