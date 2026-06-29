"""Timestamped recording for one sweep (doc §6.1) — no rclpy (numpy only).

``run_sweep`` feeds it the excited joint's measured state (from ``/joint_states``) and
the commanded set-point, each with its own monotonic timestamp. ``save_csv`` emits the
doc's table ``t,q_cmd,q,qd,effort,f0,f1`` using the state timestamps as the master grid
and linearly interpolating the command stream onto them (command and state arrive on
different clocks/rates). ``fs`` is later measured from ``t`` (never assumed 500 Hz, §5).
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

CSV_FIELDS = ("t", "q_cmd", "q", "qd", "effort", "f0", "f1")


class Recorder:
    """Accumulate (t, q, qd, effort) state samples and (t, q_cmd) command samples."""

    def __init__(self, joint: str) -> None:
        self.joint = joint
        self._states: list[tuple[float, float, float, float]] = []
        self._cmds: list[tuple[float, float]] = []

    def add_state(self, t: float, q: float, qd: float, effort: float) -> None:
        self._states.append((float(t), float(q), float(qd), float(effort)))

    def add_command(self, t: float, q_cmd: float) -> None:
        self._cmds.append((float(t), float(q_cmd)))

    @property
    def num_states(self) -> int:
        return len(self._states)

    @property
    def num_commands(self) -> int:
        return len(self._cmds)

    def _table(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not self._states:
            raise ValueError("no state samples recorded")
        states = np.asarray(self._states, dtype=float)
        t, q, qd, effort = states[:, 0], states[:, 1], states[:, 2], states[:, 3]
        if self._cmds:
            cmds = np.asarray(self._cmds, dtype=float)
            order = np.argsort(cmds[:, 0])
            q_cmd = np.interp(t, cmds[order, 0], cmds[order, 1])
        else:
            q_cmd = np.full_like(t, np.nan)
        return t, q_cmd, q, qd, effort

    def save_csv(self, path: Path | str, *, f0: float = 0.0, f1: float = 0.0) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        t, q_cmd, q, qd, effort = self._table()
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(CSV_FIELDS)
            for i in range(t.size):
                writer.writerow(
                    [
                        f"{t[i]:.9f}",
                        f"{q_cmd[i]:.9f}",
                        f"{q[i]:.9f}",
                        f"{qd[i]:.9f}",
                        f"{effort[i]:.9f}",
                        f"{f0:.6f}",
                        f"{f1:.6f}",
                    ]
                )
        return path

    def save_meta(self, path: Path | str, meta: dict[str, Any]) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(meta, indent=2, sort_keys=True), encoding="utf-8")
        return path


def load_csv(path: Path | str) -> dict[str, np.ndarray]:
    """Read a sweep CSV back into column arrays (used by ``fit_gains``)."""
    path = Path(path)
    with path.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty sweep CSV: {path}")
    cols: dict[str, np.ndarray] = {}
    for field in CSV_FIELDS:
        cols[field] = np.array([float(row[field]) for row in rows], dtype=float)
    return cols
