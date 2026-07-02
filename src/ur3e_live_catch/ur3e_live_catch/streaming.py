"""Command streaming to the forward_position_controller (archi §4.3.6), wired at step 6.

The live loop runs at 60 Hz (sim ``dt = 1/60``); the UR ``forward_position_controller``
(`position_controllers/JointGroupPositionController`) accepts a 6-value
``std_msgs/Float64MultiArray`` on ``/forward_position_controller/commands`` and the
driver/hardware interpolates each command to the 500 Hz servo loop. So by default
the node simply *formats* the safe joint target in the canonical joint order and
publishes it once per tick — :meth:`CommandStreamer.format`.

For cases where we want to publish faster than 60 Hz (a higher-rate timer feeding
the controller smoother set-points), :meth:`CommandStreamer.stream` upsamples by
linear interpolation between the previous command and the new target. Both paths
share one rule: the command is ALWAYS 6 values in :data:`joint_order.JOINT_ORDER`
— a scrambled order would send each joint the wrong angle.

Pure logic (no rclpy / no numpy): the node builds the ``Float64MultiArray`` from
the returned plain ``list[float]`` so this is unit-testable off-robot.
"""

from __future__ import annotations

from typing import Optional, Sequence

from ur3e_live_catch.joint_order import JOINT_ORDER


def step_toward(
    current: Sequence[float],
    goal: Sequence[float],
    max_deltas: Sequence[float],
) -> list[float]:
    """One interpolation step from ``current`` toward ``goal``, per-joint capped.

    Used by the high-rate command timer: the UR driver validates consecutive
    servoj set-points every 2 ms against the joint velocity limits, so the
    per-message delta must stay <= v_safe * command period — publishing the 60 Hz
    safe target directly would be rejected as a velocity violation.
    """
    if not (len(current) == len(goal) == len(max_deltas)):
        raise ValueError("current, goal and max_deltas must have the same length")
    out: list[float] = []
    for cur, tgt, max_delta in zip(current, goal, max_deltas):
        if max_delta < 0.0:
            raise ValueError(f"max_deltas must be >= 0, got {max_delta}")
        delta = float(tgt) - float(cur)
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        out.append(float(cur) + delta)
    return out


class CommandStreamer:
    """Format (and optionally upsample) safe joint targets into controller commands.

    ``substeps`` linearly interpolates from the previously streamed command to the
    new target, yielding ``substeps`` commands (the last one equals the target).
    ``substeps == 1`` (default) just forwards the target — the hardware does the
    60 Hz -> 500 Hz interpolation. ``hold`` repeats the last command, used for the
    watchdog controlled-stop (archi §9): a position controller holds its last
    set-point, so re-publishing it keeps the robot still.
    """

    def __init__(self, joint_order: Sequence[str] = JOINT_ORDER, substeps: int = 1) -> None:
        if substeps < 1:
            raise ValueError(f"substeps must be >= 1, got {substeps}")
        self.joint_order = tuple(joint_order)
        self.ndof = len(self.joint_order)
        self.substeps = int(substeps)
        self._last: Optional[list[float]] = None

    def reset(self) -> None:
        self._last = None

    @property
    def last_command(self) -> Optional[list[float]]:
        return list(self._last) if self._last is not None else None

    def format(self, target: Sequence[float]) -> list[float]:
        """Validate and return the 6-value command, recording it as the last command."""
        if len(target) != self.ndof:
            raise ValueError(f"target has {len(target)} values, expected {self.ndof}")
        cmd = [float(v) for v in target]
        self._last = cmd
        return list(cmd)

    def stream(self, target: Sequence[float]) -> list[list[float]]:
        """Return ``substeps`` commands interpolating last_command -> target.

        On the first call (no previous command) returns a single command equal to
        the target — never ramp from an unknown start.
        """
        if len(target) != self.ndof:
            raise ValueError(f"target has {len(target)} values, expected {self.ndof}")
        goal = [float(v) for v in target]
        if self._last is None or self.substeps == 1:
            self._last = goal
            return [list(goal)]
        start = self._last
        commands: list[list[float]] = []
        for k in range(1, self.substeps + 1):
            frac = k / self.substeps
            commands.append([start[i] + (goal[i] - start[i]) * frac for i in range(self.ndof)])
        self._last = goal
        return commands

    def hold(self, fallback: Optional[Sequence[float]] = None) -> list[float]:
        """Repeat the last command (controlled stop). If none yet, use ``fallback``.

        ``fallback`` is normally the current measured ``q`` so the very first
        watchdog stop still produces a valid hold-in-place command.
        """
        if self._last is not None:
            return list(self._last)
        if fallback is None:
            raise ValueError("no previous command and no fallback to hold")
        if len(fallback) != self.ndof:
            raise ValueError(f"fallback has {len(fallback)} values, expected {self.ndof}")
        cmd = [float(v) for v in fallback]
        self._last = cmd
        return list(cmd)
