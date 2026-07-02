"""Policy runtime: load the exported PPO policy and run inference (archi §4.3.3).

Backends are imported lazily so this module imports fine with neither torch nor
onnxruntime installed (the pure-logic tests and the dry-run wiring can be
exercised without them). The live node calls :meth:`PolicyRunner.infer`.

THE SCALER (critical, archi §4.3.3): SKRL PPO wraps observations in a
``RunningStandardScaler``. If the export does NOT embed it, we must apply the
saved ``mean`` / ``var`` BEFORE the network — feeding the raw net unnormalised
observations is a major sim-to-real divergence. ``policy_metadata.json`` here
carries no mean/var, so either the ``.ts`` embeds the scaler or a sidecar
``policy_scaler.json`` must provide it. The policy-equivalence test
(``test/test_policy_equivalence.py``) decides this empirically on the ROS machine.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional, Sequence


class ObsScaler:
    """``RunningStandardScaler``: ``(obs - mean) / sqrt(var + eps)``."""

    def __init__(self, mean: Sequence[float], var: Sequence[float], eps: float = 1e-8) -> None:
        if len(mean) != len(var):
            raise ValueError("mean and var length mismatch")
        self.mean = [float(m) for m in mean]
        self.std = [math.sqrt(float(v) + eps) for v in var]

    def __call__(self, obs: Sequence[float]) -> list[float]:
        if len(obs) != len(self.mean):
            raise ValueError(f"obs dim {len(obs)} != scaler dim {len(self.mean)}")
        return [(float(o) - m) / s for o, m, s in zip(obs, self.mean, self.std)]

    @classmethod
    def from_json(cls, path: Path | str) -> "ObsScaler":
        data = json.loads(Path(path).read_text())
        return cls(data["mean"], data["var"], float(data.get("eps", 1e-8)))


class PolicyRunner:
    """Load ``policy_deterministic.ts`` (torch) or ``.onnx`` and infer 6 actions."""

    def __init__(
        self,
        model_path: Path | str,
        *,
        backend: str = "auto",
        scaler: Optional[ObsScaler] = None,
    ) -> None:
        self.model_path = Path(model_path)
        self.scaler = scaler
        self._backend = self._resolve_backend(backend)
        self._model = None  # loaded lazily on first infer / explicit load()

    def _resolve_backend(self, backend: str) -> str:
        if backend != "auto":
            return backend
        suffix = self.model_path.suffix.lower()
        if suffix == ".onnx":
            return "onnx"
        return "torch"  # .ts / .pt default

    def load(self) -> "PolicyRunner":
        if self._model is not None:
            return self
        if not self.model_path.exists():
            raise FileNotFoundError(f"policy not found: {self.model_path}")
        if self._backend == "torch":
            import torch  # lazy

            self._torch = torch
            self._model = torch.jit.load(str(self.model_path), map_location="cpu").eval()
        elif self._backend == "onnx":
            import onnxruntime  # lazy

            self._model = onnxruntime.InferenceSession(
                str(self.model_path), providers=["CPUExecutionProvider"]
            )
            self._onnx_input = self._model.get_inputs()[0].name
        else:
            raise ValueError(f"unknown backend {self._backend!r}")
        return self

    def infer(self, obs: Sequence[float]) -> list[float]:
        if self._model is None:
            self.load()
        scaled = self.scaler(obs) if self.scaler is not None else [float(o) for o in obs]
        if self._backend == "torch":
            torch = self._torch
            with torch.no_grad():
                tensor = torch.tensor([scaled], dtype=torch.float32)
                out = self._model(tensor)
                if isinstance(out, (tuple, list)):
                    out = out[0]
                return [float(x) for x in out.reshape(-1).tolist()]
        else:
            import numpy as np  # onnxruntime already pulls numpy

            inp = np.asarray([scaled], dtype=np.float32)
            out = self._model.run(None, {self._onnx_input: inp})[0]
            return [float(x) for x in out.reshape(-1).tolist()]


def load_metadata(path: Path | str) -> dict:
    """Read ``policy_metadata.json`` (obs/action dims, action_scale, dt, joints)."""
    return json.loads(Path(path).read_text())


def policy_model_candidates(configured: str, default_candidates: Sequence[str]) -> list[str]:
    """Return model load attempts, including ONNX/TorchScript siblings for explicit paths."""
    if not configured:
        return [candidate for candidate in default_candidates if candidate]

    path = Path(configured)
    candidates = [configured]
    suffix = path.suffix.lower()
    if suffix == ".onnx":
        candidates.append(str(path.with_suffix(".ts")))
    elif suffix in (".ts", ".pt"):
        candidates.append(str(path.with_suffix(".onnx")))
    return list(dict.fromkeys(candidate for candidate in candidates if candidate))
