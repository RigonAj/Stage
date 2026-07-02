"""Frame-aware ball transform + velocity filtering (archi §4.3.1).

Pure logic, no rclpy / no numpy: the rigid-body transform is passed *as data*
(translation + quaternion, exactly what a tf2 lookup yields), so the whole module
is unit-testable off-robot. The live node supplies the transform via tf2.

Design rule (archi §4.3.1, §7): the consumer is **frame-aware and never assumes
the camera**. It transforms ``header.frame_id -> base_link`` by default:
  - ``frame_id == base_frame``     -> identity (ball already in the policy frame);
  - any other frame                -> apply the supplied transform (e.g. hand-eye);
  - empty / unknown frame          -> caller rejects (raise), never guess.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class RigidTransform:
    """``P^target = R(quaternion) * P^source + translation``.

    Matches tf2 ``lookup_transform(target, source)``: applying this to a point
    expressed in ``source`` yields the point expressed in ``target``.
    """

    translation: Vec3 = (0.0, 0.0, 0.0)
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def apply(self, point: Sequence[float]) -> Vec3:
        rx, ry, rz = _quat_rotate(self.quaternion, point)
        tx, ty, tz = self.translation
        return (rx + tx, ry + ty, rz + tz)


def _quat_rotate(q: Sequence[float], v: Sequence[float]) -> Vec3:
    """Rotate vector ``v`` by quaternion ``q`` (x, y, z, w): v' = q * v * q^-1."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w * t + cross(q_xyz, t)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class BallVelocityFilter:
    """Filtered ball velocity from finite differences of policy-frame positions.

    No direct velocity measurement exists — this is the noisy link (sim-to-real
    §5.2). Because Isaac local = world - constant env origin, the finite
    difference of base_link positions IS the world velocity ``ball_vel_w``.
    """

    def __init__(self, ema_alpha: float = 0.5, max_dt: float = 0.5) -> None:
        self._ema_alpha = float(ema_alpha)
        self._max_dt = float(max_dt)
        self._prev_pos: Optional[Vec3] = None
        self._prev_stamp: Optional[float] = None
        self._vel: Vec3 = (0.0, 0.0, 0.0)

    def reset(self) -> None:
        self._prev_pos = None
        self._prev_stamp = None
        self._vel = (0.0, 0.0, 0.0)

    def update(self, pos_base: Sequence[float], stamp_s: float) -> Vec3:
        pos = (float(pos_base[0]), float(pos_base[1]), float(pos_base[2]))
        if self._prev_pos is None or self._prev_stamp is None:
            self._prev_pos, self._prev_stamp = pos, float(stamp_s)
            return self._vel
        dt = float(stamp_s) - self._prev_stamp
        if dt <= 0.0 or dt > self._max_dt:
            # Out-of-order or too large a gap: keep last estimate, re-anchor.
            self._prev_pos, self._prev_stamp = pos, float(stamp_s)
            return self._vel
        raw = ((pos[0] - self._prev_pos[0]) / dt,
               (pos[1] - self._prev_pos[1]) / dt,
               (pos[2] - self._prev_pos[2]) / dt)
        a = self._ema_alpha
        self._vel = (a * raw[0] + (1 - a) * self._vel[0],
                     a * raw[1] + (1 - a) * self._vel[1],
                     a * raw[2] + (1 - a) * self._vel[2])
        self._prev_pos, self._prev_stamp = pos, float(stamp_s)
        return self._vel


class FrameError(ValueError):
    """The declared frame_id is empty or no transform is available for it."""


class BallFrameTransformer:
    """Transform a declared ball position into the policy frame and filter velocity."""

    def __init__(
        self,
        base_frame: str = "base_link",
        *,
        units: str = "m",
        ema_alpha: float = 0.5,
        stale_after_s: float = 0.1,
    ) -> None:
        if units not in ("m", "mm"):
            raise ValueError(f"units must be 'm' or 'mm', got {units!r}")
        self.base_frame = base_frame
        self._scale = 1e-3 if units == "mm" else 1.0
        self._vel_filter = BallVelocityFilter(ema_alpha=ema_alpha)
        self.stale_after_s = float(stale_after_s)
        self._last_stamp: Optional[float] = None

    def to_base(
        self,
        position: Sequence[float],
        frame_id: str,
        transform: Optional[RigidTransform] = None,
    ) -> Vec3:
        """Return the ball position in the configured policy frame.

        ``transform`` is the ``frame_id -> base_frame`` rigid transform (tf2 lookup).
        It is ignored for the identity case (``frame_id == base_frame``) and
        required otherwise.
        """
        if not frame_id:
            raise FrameError("empty frame_id: refusing to assume a frame")
        p = (position[0] * self._scale, position[1] * self._scale, position[2] * self._scale)
        if frame_id == self.base_frame:
            return (float(p[0]), float(p[1]), float(p[2]))
        if transform is None:
            raise FrameError(f"no transform available for frame_id {frame_id!r} -> {self.base_frame!r}")
        return transform.apply(p)

    def process(
        self,
        position: Sequence[float],
        frame_id: str,
        stamp_s: float,
        transform: Optional[RigidTransform] = None,
    ) -> tuple[Vec3, Vec3]:
        """Return ``(pos_base, vel_base)``, updating the velocity filter."""
        pos_base = self.to_base(position, frame_id, transform)
        vel_base = self._vel_filter.update(pos_base, stamp_s)
        self._last_stamp = float(stamp_s)
        return pos_base, vel_base

    def is_stale(self, now_s: float) -> bool:
        """True if no fresh detection within ``stale_after_s`` (archi §9 watchdog)."""
        if self._last_stamp is None:
            return True
        return (now_s - self._last_stamp) > self.stale_after_s
