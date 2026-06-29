#!/usr/bin/env python3
"""Quick FRF sanity check for ur3e_sysid chirp sweeps: measured gain vs fitted model.

Reads ``recordings/sysid/<joint>_chirp.csv`` and the fitted
``ur3e_actuator_identified.yaml``, then prints, per joint, the identified
``(K, D, wn, zeta, latency, R^2)`` and a gain table comparing the **measured**
closed-loop gain ``std(q)/std(q_cmd)`` per 1 Hz band against the fitted 2nd-order
model ``|wn^2 / (wn^2 - w^2 + 2*zeta*wn*j*w)|``.

Use it to confirm the chirp band actually reached the -3 dB roll-off (so ``wn``/``K``
are *identified*, not extrapolated): if the measured gain stays ~1.0 across the whole
band, push ``run_sweep --f1`` higher (with a smaller ``--amplitude``). Pure numpy;
run under the ROS / system python (no scipy or torch needed).

  python3 scripts/sysid_frf_check.py                 # all joints present in --in-dir
  python3 scripts/sysid_frf_check.py elbow wrist_1   # selected joints (short or full)
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


def load_chirp(path: Path):
    t, qc, q = [], [], []
    f0 = f1 = 0.0
    with open(path) as fh:
        for row in csv.DictReader(fh):
            t.append(float(row["t"]))
            qc.append(float(row["q_cmd"]))
            q.append(float(row["q"]))
            f0 = float(row["f0"])
            f1 = float(row["f1"])
    return np.array(t), np.array(qc), np.array(q), f0, f1


def model_gain(fhz, wn: float, zeta: float):
    w = 2.0 * np.pi * np.asarray(fhz, dtype=float)
    return np.abs(wn * wn / (wn * wn - w * w + 2.0 * zeta * wn * 1j * w))


def check(joint: str, in_dir: str, gains: dict) -> None:
    csv_path = Path(in_dir) / f"{joint}_chirp.csv"
    if not csv_path.is_file():
        print(f"{joint}: no chirp CSV at {csv_path}")
        return
    t, qc, q, f0, f1 = load_chirp(csv_path)
    qc0 = qc - np.mean(qc)
    q0 = q - np.mean(q)
    finst = f0 + (f1 - f0) * (t - t[0]) / (t[-1] - t[0])

    g = gains.get(joint, {})
    wn = g.get("wn")
    zeta = g.get("zeta")
    header = f"{joint}: "
    if wn:
        header += (
            f"K={g['stiffness']:.0f} D={g['damping']:.2f} "
            f"wn={wn / (2 * np.pi):.1f}Hz zeta={zeta:.2f} "
            f"L={g['latency_s'] * 1e3:.0f}ms R2_chirp={g.get('fit_r2_chirp')}"
        )
    else:
        header += "(not in YAML; gain table only)"
    print(header)
    print(f"  band {f0:.1f}-{f1:.1f} Hz    f(Hz)  measured   model")

    prev = None
    cross = None
    for lo in np.arange(np.floor(f0), np.ceil(f1)):
        mask = (finst >= lo) & (finst < lo + 1)
        if int(mask.sum()) < 30:
            continue
        fc = lo + 0.5
        gm = float(np.std(q0[mask]) / max(np.std(qc0[mask]), 1e-9))
        mm = float(model_gain(fc, wn, zeta)) if wn else float("nan")
        flag = ""
        if prev is not None and prev >= 0.707 > gm and cross is None:
            cross = fc
            flag = "  <- ~-3 dB"
        print(f"            {fc:6.1f}   {gm:6.3f}   {mm:6.3f}{flag}")
        prev = gm

    if cross:
        extra = f" (wn_fit={wn / (2 * np.pi):.1f} Hz)" if wn else ""
        print(f"  -3 dB measured ~ {cross:.1f} Hz{extra}")
    else:
        print("  no -3 dB crossing in band -> band too low: push --f1 higher (or fine if already steep)")
    amp = float(np.std(qc0) * 1e3)
    if amp < 8.0:
        print(f"  note: small cmd amplitude ({amp:.1f} mrad) -> check coherence at top of band")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="FRF sanity check for ur3e_sysid chirp sweeps.")
    p.add_argument("joints", nargs="*", help="joint names (short or full); default: all chirp CSVs found")
    p.add_argument("--in-dir", default="recordings/sysid")
    p.add_argument("--yaml", default="ur3e_actuator_identified.yaml")
    args = p.parse_args(argv)

    gains: dict = {}
    yaml_path = Path(args.yaml)
    if yaml_path.is_file():
        try:
            import yaml

            gains = (yaml.safe_load(yaml_path.read_text()) or {}).get("joints", {})
        except Exception as exc:  # noqa: BLE001
            print(f"(could not read {yaml_path}: {exc})", file=sys.stderr)

    def norm(name: str) -> str:
        return name if name.endswith("_joint") else f"{name}_joint"

    if args.joints:
        joints = [norm(j) for j in args.joints]
    else:
        joints = sorted(x.name[: -len("_chirp.csv")] for x in Path(args.in_dir).glob("*_chirp.csv"))
    if not joints:
        print(f"no chirp CSVs found in {args.in_dir}", file=sys.stderr)
        return 2
    for j in joints:
        check(j, args.in_dir, gains)
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
