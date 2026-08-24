#!/usr/bin/env python3
"""Regenerate the two computed figures of the internship report.

Both figures are produced from data already on disk, so they can be rebuilt
without re-running the perception pipeline:

* ``trace-convergence`` uses the matched detection/ground-truth samples written
  by the offline benchmark (``evaluation/matched_samples_trace.csv`` of a
  benchmark sequence) and shows how the fitted 3D trajectory converges as new
  width measurements arrive.
* ``intrinsic-poses`` uses the intrinsic calibration report
  (``intrinsics_from_mire_robust_constrained_report.json``) and shows the pose
  diversity of the views retained by the robust selection.

Usage:
    python3 scripts/plot_report_figures.py trace-convergence
    python3 scripts/plot_report_figures.py intrinsic-poses
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DEFAULT_BENCHMARK = os.path.expanduser(
    "~/Documents/EventGen/ball_event_dataset_v0/benchmark/datasets/benchmark_fast_throw_0500"
)
DEFAULT_SEQUENCE = "sequence_000110"
DEFAULT_INTRINSICS = "recordings/mire_calibration/intrinsics_from_mire_robust_constrained_report.json"

BLUE = "#1f6fb4"
RED = "#c0392b"
FITS = ["#9ecae1", "#4292c6", "#08519c"]


def _fit_linear(t, q):
    a, b = np.polyfit(t, q, 1)
    return lambda tt: a * tt + b


def _fit_quadratic(t, q):
    a, b, c = np.polyfit(t, q, 2)
    return lambda tt: a * tt ** 2 + b * tt + c


def trace_convergence(args):
    path = os.path.join(args.benchmark, "sequences", args.sequence,
                        "evaluation", "matched_samples_trace.csv")
    gt_t, gt_x, gt_y, gt_z = [], [], [], []
    es_t, es_x, es_y, es_z = [], [], [], []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            t = float(row["timestamp_s"])
            gt_t.append(t)
            gt_x.append(float(row["gt_x_cam_m"]))
            gt_y.append(float(row["gt_y_cam_m"]))
            gt_z.append(float(row["gt_z_cam_m"]))
            if row["detected"] == "1" and row["est_x_m"]:
                es_t.append(t)
                es_x.append(float(row["est_x_m"]))
                es_y.append(float(row["est_y_m"]))
                es_z.append(float(row["est_z_m"]))
    gt_t = np.array(gt_t); gt_x = np.array(gt_x); gt_y = np.array(gt_y); gt_z = np.array(gt_z)
    es_t = np.array(es_t); es_x = np.array(es_x); es_y = np.array(es_y); es_z = np.array(es_z)
    order = np.argsort(es_t)
    es_t, es_x, es_y, es_z = es_t[order], es_x[order], es_y[order], es_z[order]

    t0 = gt_t.min()
    gt_t -= t0
    es_t -= t0
    span = np.linspace(gt_t.min(), gt_t.max(), 200)

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.0))
    ax_h, ax_z = axes

    ax_h.plot(gt_t, -gt_y, color=RED, lw=2.0, label="vérité terrain", zorder=2)
    ax_h.scatter(es_t, -es_y, s=12, color=BLUE, alpha=0.45,
                 label="estimations Trace", zorder=3)
    ax_z.plot(gt_t, gt_z, color=RED, lw=2.0, zorder=2)
    ax_z.scatter(es_t, es_z, s=12, color=BLUE, alpha=0.45, zorder=3)

    fractions = [0.30, 0.60, 1.00]
    for colour, frac in zip(FITS, fractions):
        keep = max(4, int(round(frac * len(es_t))))
        tt, xx, yy, zz = es_t[:keep], es_x[:keep], es_y[:keep], es_z[:keep]
        t_cut = tt[-1]
        fz = _fit_linear(tt, zz)
        fy = _fit_quadratic(tt, yy)
        seen = span <= t_cut
        label = "ajustement sur les %d premiers points (t = %.2f s)" % (keep, t_cut)
        ax_h.plot(span[seen], -fy(span[seen]), color=colour, lw=1.8, zorder=4, label=label)
        ax_h.plot(span[~seen], -fy(span[~seen]), color=colour, lw=1.4, ls="--", zorder=4)
        ax_z.plot(span[seen], fz(span[seen]), color=colour, lw=1.8, zorder=4)
        ax_z.plot(span[~seen], fz(span[~seen]), color=colour, lw=1.4, ls="--", zorder=4)

    ax_h.set_xlabel("temps (s)")
    ax_h.set_ylabel("hauteur (m)")
    ax_h.set_title("Hauteur")
    ax_z.set_xlabel("temps (s)")
    ax_z.set_ylabel("profondeur $Z$ (m)")
    ax_z.set_title("Profondeur")
    lo = min(gt_z.min(), np.percentile(es_z, 2)) - 0.05
    hi = max(gt_z.max(), np.percentile(es_z, 98)) + 0.05
    ax_z.set_ylim(lo, hi)
    for ax in axes:
        ax.grid(alpha=0.25)
    ax_h.legend(fontsize=8, loc="upper left", framealpha=0.9)
    fig.tight_layout()
    fig.savefig(args.output, dpi=200)
    print("wrote", args.output, "from", path,
          "(%d samples, %d detections)" % (len(gt_t), len(es_t)))


def intrinsic_poses(args):
    data = json.load(open(args.report))
    tilts, dists, centres = [], [], []
    for view, geom in zip(data["views"], data["view_geometries"]):
        rvec = np.asarray(view["rvec"], dtype=float)
        theta = np.linalg.norm(rvec)
        if theta < 1e-12:
            rot = np.eye(3)
        else:
            k = rvec / theta
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            rot = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K
        normal = rot @ np.array([0.0, 0.0, 1.0])
        tilts.append(math.degrees(math.acos(min(1.0, abs(normal[2])))))
        dists.append(view["tvec_mm"][2] / 1000.0)
        centres.append((geom["center_x"], geom["center_y"]))

    fig, axes = plt.subplots(1, 2, figsize=(10.0, 3.6))
    ax_hist, ax_cov = axes
    ax_hist.hist(tilts, bins=np.arange(0, 50, 5), color=BLUE, alpha=0.85, edgecolor="white")
    ax_hist.axvline(15, color=RED, ls="--", lw=1.5)
    ax_hist.text(15.6, ax_hist.get_ylim()[1] * 0.92, "15°", color=RED, fontsize=9)
    ax_hist.set_xlabel("inclinaison de la mire par rapport au plan image (°)")
    ax_hist.set_ylabel("nombre de vues")
    ax_hist.set_title("Diversité angulaire des %d vues retenues" % len(tilts))

    width = data["image_size"]["width"]
    height = data["image_size"]["height"]
    scatter = ax_cov.scatter([c[0] for c in centres], [c[1] for c in centres],
                             c=dists, cmap="viridis", s=45, edgecolor="black", linewidth=0.4)
    ax_cov.add_patch(plt.Rectangle((0, 0), width, height, fill=False, edgecolor="black", lw=1.0))
    ax_cov.set_xlim(-20, width + 20)
    ax_cov.set_ylim(height + 20, -20)
    ax_cov.set_xlabel("$u$ (px)")
    ax_cov.set_ylabel("$v$ (px)")
    ax_cov.set_title("Position des mires dans l'image")
    bar = fig.colorbar(scatter, ax=ax_cov)
    bar.set_label("distance mire-caméra (m)")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(args.output, dpi=200)
    print("wrote", args.output,
          "tilt min/median/max = %.1f / %.1f / %.1f deg" % (
              min(tilts), float(np.median(tilts)), max(tilts)),
          "distance %.2f-%.2f m" % (min(dists), max(dists)))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    conv = sub.add_parser("trace-convergence")
    conv.add_argument("--benchmark", default=DEFAULT_BENCHMARK)
    conv.add_argument("--sequence", default=DEFAULT_SEQUENCE)
    conv.add_argument("--output", default="images/Algo_Trace/trace_convergence.png")
    conv.set_defaults(func=trace_convergence)

    poses = sub.add_parser("intrinsic-poses")
    poses.add_argument("--report", default=DEFAULT_INTRINSICS)
    poses.add_argument("--output", default="images/Calibration/poses_intrinseques.png")
    poses.set_defaults(func=intrinsic_poses)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
