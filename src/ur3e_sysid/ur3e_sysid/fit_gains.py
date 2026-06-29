"""Offline fit stage (doc §6.1, §4, §11): sweep CSVs -> ur3e_actuator_identified.yaml.

Loads the per-joint step/chirp/ramp CSVs written by ``run_sweep``, fits (wn, zeta, L)
with :mod:`estimator`, turns them into the sim gains with ``I_eff`` from :mod:`inertia`
(``K = I*wn^2``, ``D = 2*zeta*wn*I``, doc §4.1), and writes the YAML the Isaac config
consumes. The closed-loop gains are an INITIALISATION (doc §12) to be refined by sim
replay; we therefore also store the inertia-independent (wn, zeta, L).

Run under the ROS python:  ros2 run ur3e_sysid fit_gains --in-dir recordings/sysid
"""

from __future__ import annotations

import argparse
import datetime as _dt
import math
import sys
from pathlib import Path

import numpy as np

from .config import EFFORT_LIMIT, ID_POSE, JOINTS, VELOCITY_LIMIT, joint_index, resolve_joint
from . import estimator
from .inertia import effective_inertia
from .recorder import load_csv


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Fit UR3e actuator gains from system-id sweep CSVs.")
    p.add_argument("--in-dir", default="recordings/sysid", help="Directory holding {joint}_{signal}.csv")
    p.add_argument("--out", default="ur3e_actuator_identified.yaml", help="Output YAML path")
    p.add_argument("--joints", help="Comma-separated subset of joints to fit (default: all present)")
    p.add_argument("--id-pose", help="Comma-separated 6-joint id pose (rad); default from meta/config")
    p.add_argument("--robot-serial", default="", help="Robot serial for the YAML meta")
    p.add_argument("--payload-kg", type=float, default=0.0)
    p.add_argument("--r2-min", type=float, default=0.95, help="Chirp R^2 acceptance threshold (doc §12)")
    p.add_argument("--dr-pct", type=float, default=0.25, help="Domain-randomization fraction for K/D")
    return p


def _prep_step(cols: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Extract the post-onset, normalised (0->1) step response from a sweep CSV."""
    t, q, qc = cols["t"], cols["q"], cols["q_cmd"]
    q0 = float(qc[0])
    amp = float(qc[-1] - q0)
    if abs(amp) < 1e-9:
        raise ValueError("step command has zero amplitude")
    onset = int(np.argmax(np.abs(qc - q0) > 0.5 * abs(amp)))
    if np.abs(qc[onset] - q0) <= 0.5 * abs(amp):
        raise ValueError("could not locate the step onset in q_cmd")
    q_pre = float(np.mean(q[:onset])) if onset > 0 else float(q[0])
    y = (q[onset:] - q_pre) / amp
    t_rel = t[onset:] - t[onset]
    return t_rel, y


def _friction_point(cols: dict[str, np.ndarray]) -> tuple[float, float]:
    """One (signed speed, gravity-removed torque) sample from a ramp CSV (doc §4.6)."""
    qd, effort = cols["qd"], cols["effort"]
    peak = float(np.max(np.abs(qd))) if qd.size else 0.0
    if peak < 1e-6:
        raise ValueError("ramp has no motion")
    moving = np.abs(qd) > 0.5 * peak
    dwell = np.abs(qd) < 0.05 * peak
    grav = float(np.median(effort[dwell])) if np.any(dwell) else 0.0
    return float(np.median(qd[moving])), float(np.median(effort[moving]) - grav)


def _fit_joint(joint: str, in_dir: Path, pose, r2_min: float) -> dict:
    k = joint_index(joint)
    wn_s = zeta_s = r2_s = None
    wn_c = zeta_c = L = r2_c = None

    step_csv = in_dir / f"{joint}_step.csv"
    if step_csv.is_file():
        try:
            wn_s, zeta_s, r2_s = estimator.fit_step(*_prep_step(load_csv(step_csv)))
        except Exception as exc:  # noqa: BLE001 - report and continue
            print(f"  [{joint}] step fit failed: {exc}", file=sys.stderr)

    chirp_csv = in_dir / f"{joint}_chirp.csv"
    if chirp_csv.is_file():
        cols = load_csv(chirp_csv)
        f0 = float(cols["f0"][0]) if cols["f0"].size else 0.1
        f1 = float(cols["f1"][0]) if cols["f1"].size else 3.0
        try:
            wn_c, zeta_c, L, r2_c = estimator.fit_chirp(cols["t"], cols["q_cmd"], cols["q"], f0, f1)
        except Exception as exc:  # noqa: BLE001
            print(f"  [{joint}] chirp fit failed: {exc}", file=sys.stderr)

    # consolidate: chirp priority, else step.
    if wn_c is not None and wn_s is not None:
        wn, zeta, warns = estimator.reconcile(wn_c, zeta_c, wn_s, zeta_s)
        for w in warns:
            print(f"  [{joint}] reconcile: {w}", file=sys.stderr)
    elif wn_c is not None:
        wn, zeta = wn_c, zeta_c
    elif wn_s is not None:
        wn, zeta = wn_s, zeta_s
    else:
        raise ValueError(f"no usable step or chirp fit for {joint}")

    inertia = effective_inertia(pose)[joint]
    K = inertia * wn * wn
    D = 2.0 * zeta * wn * inertia

    # friction (best-effort, doc §4.6): all {joint}_ramp*.csv -> one point each.
    fc = fv = 0.0
    pts = []
    for ramp_csv in sorted(in_dir.glob(f"{joint}_ramp*.csv")):
        try:
            pts.append(_friction_point(load_csv(ramp_csv)))
        except Exception as exc:  # noqa: BLE001
            print(f"  [{joint}] friction point from {ramp_csv.name} skipped: {exc}", file=sys.stderr)
    if len(pts) >= 2:
        v = [p[0] for p in pts]
        tau = [p[1] for p in pts]
        fc, fv, _ = estimator.fit_friction(v, tau)
    elif pts:
        print(f"  [{joint}] only {len(pts)} ramp point(s); need >=2 for friction — left at 0", file=sys.stderr)

    if r2_c is not None and r2_c < r2_min:
        print(f"  [{joint}] WARNING chirp R^2={r2_c:.3f} < {r2_min} (doc §12)", file=sys.stderr)

    return {
        "stiffness": round(K, 6),
        "damping": round(D, 6),
        "latency_s": round(L, 6) if L is not None else 0.0,
        "inertia_eff": round(inertia, 8),
        "wn": round(wn, 6),
        "zeta": round(zeta, 6),
        "friction_coulomb_Nm": round(fc, 6),
        "friction_viscous_Nms": round(fv, 6),
        "effort_limit": EFFORT_LIMIT[k],
        "velocity_limit": round(VELOCITY_LIMIT[k], 4),
        "fit_r2_step": round(r2_s, 4) if r2_s is not None else None,
        "fit_r2_chirp": round(r2_c, 4) if r2_c is not None else None,
    }


def _load_pose(in_dir: Path, override: str | None):
    if override:
        parts = tuple(float(x.strip()) for x in override.split(","))
        if len(parts) != 6:
            raise ValueError("--id-pose must contain 6 values")
        return parts
    for meta in in_dir.glob("*.meta.json"):
        try:
            import json

            data = json.loads(meta.read_text(encoding="utf-8"))
            pose = data.get("id_pose_rad")
            if isinstance(pose, list) and len(pose) == 6:
                return tuple(float(x) for x in pose)
        except Exception:  # noqa: BLE001
            continue
    return tuple(ID_POSE)


def main(argv: list[str] | None = None) -> int:
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover
        print(f"ERROR: PyYAML required: {exc}", file=sys.stderr)
        return 2

    args = build_parser().parse_args(argv)
    in_dir = Path(args.in_dir)
    if not in_dir.is_dir():
        print(f"ERROR: input directory not found: {in_dir}", file=sys.stderr)
        return 2

    pose = _load_pose(in_dir, args.id_pose)
    requested = [resolve_joint(j.strip()) for j in args.joints.split(",")] if args.joints else list(JOINTS)
    present = [j for j in requested if (in_dir / f"{j}_step.csv").is_file() or (in_dir / f"{j}_chirp.csv").is_file()]
    if not present:
        print(f"ERROR: no {{joint}}_step.csv / {{joint}}_chirp.csv found in {in_dir}", file=sys.stderr)
        return 2

    joints_out: dict[str, dict] = {}
    for joint in present:
        print(f"fitting {joint} ...")
        try:
            joints_out[joint] = _fit_joint(joint, in_dir, pose, args.r2_min)
        except Exception as exc:  # noqa: BLE001
            print(f"  [{joint}] skipped: {exc}", file=sys.stderr)

    if not joints_out:
        print("ERROR: no joint could be fitted", file=sys.stderr)
        return 1

    # sanity: stiffness hierarchy shoulder > elbow > wrist (doc §12).
    ks = {j: joints_out[j]["stiffness"] for j in joints_out}
    print(f"stiffness by joint: { {j: round(v, 1) for j, v in ks.items()} }")

    document = {
        "meta": {
            "robot_serial": args.robot_serial,
            "date": _dt.date.today().isoformat(),
            "id_pose_rad": [round(float(x), 6) for x in pose],
            "payload_kg": args.payload_kg,
            "interface": "forward_position_controller@60Hz",
            "source": "ur3e_sysid.fit_gains",
        },
        "joints": joints_out,
        "domain_randomization": {
            "stiffness_pct": args.dr_pct,
            "damping_pct": args.dr_pct,
            "latency_s_range": [0.0, max((j["latency_s"] for j in joints_out.values()), default=0.0)],
        },
    }
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(document, sort_keys=False, default_flow_style=False), encoding="utf-8")
    print(f"wrote {out_path} ({len(joints_out)} joints)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
