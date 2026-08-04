#!/usr/bin/env python3
"""Turn method_comparison_benchmark output into Trace vs circle-fitting metrics.

Reads the detections_*.csv files and run_manifest.json produced by
``method_comparison_benchmark`` and writes, per sequence and aggregated:

  * RMSE of the 3D position against the interpolated ground truth, plus the
    per-axis breakdown (depth is the z column of camera_optical, the axis both
    methods derive from an apparent size, hence the one that actually
    discriminates them);
  * robust error statistics (median, p95) so a handful of aberrant estimates
    cannot carry the comparison;
  * signed depth bias, which quantifies the systematic depth offset the report
    describes qualitatively for circle fitting;
  * detection coverage and rate - essential context, because RMSE alone is
    meaningless when one method produces 500 estimates and the other 12.

Standard library only; matplotlib is used for the optional figures when it is
importable.

Usage:
    python3 scripts/compare_trace_vs_circle.py evaluation/method_comparison/run1
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

METHODS = ("trace", "circle")
METHOD_LABEL = {"trace": "Trace", "circle": "Circle fitting"}


# ---------------------------------------------------------------------------
# Small statistics helpers (kept stdlib so the script runs anywhere)
# ---------------------------------------------------------------------------


def rms(values: list[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def mean(values: list[float]) -> float:
    if not values:
        return float("nan")
    return sum(values) / len(values)


def quantile(values: list[float], q: float) -> float:
    """Linear-interpolation quantile, same convention as numpy's default."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = q * (len(ordered) - 1)
    low = int(math.floor(position))
    high = min(low + 1, len(ordered) - 1)
    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


# ---------------------------------------------------------------------------
# Input
# ---------------------------------------------------------------------------


@dataclass
class Detection:
    sequence: str
    method: str
    t: float
    est: tuple[float, float, float]
    gt_valid: bool
    gt_visible: bool
    gt: tuple[float, float, float]
    err: tuple[float, float, float]
    err_norm: float
    size_px: float
    runtime_ms: float


@dataclass
class SequenceInfo:
    name: str
    gt_first_s: float = float("nan")
    gt_last_s: float = float("nan")
    ball_radius_mm: float = float("nan")
    fx: float = float("nan")
    error: str = ""


def _float(row: dict[str, str], key: str) -> float:
    text = (row.get(key) or "").strip()
    if not text:
        return float("nan")
    try:
        return float(text)
    except ValueError:
        return float("nan")


def load_detections(run_dir: Path) -> list[Detection]:
    detections: list[Detection] = []
    for path in sorted(run_dir.glob("detections_*.csv")):
        with path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                detections.append(
                    Detection(
                        sequence=row["sequence"],
                        method=row["method"],
                        t=_float(row, "t_est_s"),
                        est=(
                            _float(row, "x_cam_m"),
                            _float(row, "y_cam_m"),
                            _float(row, "z_cam_m"),
                        ),
                        gt_valid=(row.get("gt_valid") or "0").strip() == "1",
                        gt_visible=(row.get("gt_visible") or "0").strip() == "1",
                        gt=(
                            _float(row, "gt_x_cam_m"),
                            _float(row, "gt_y_cam_m"),
                            _float(row, "gt_z_cam_m"),
                        ),
                        err=(
                            _float(row, "err_x_m"),
                            _float(row, "err_y_m"),
                            _float(row, "err_z_m"),
                        ),
                        err_norm=_float(row, "err_norm_m"),
                        size_px=_float(row, "size_px"),
                        runtime_ms=_float(row, "runtime_ms"),
                    )
                )
    return detections


def load_sequence_info(run_dir: Path) -> dict[str, SequenceInfo]:
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.is_file():
        return {}

    with manifest_path.open() as handle:
        manifest = json.load(handle)

    info: dict[str, SequenceInfo] = {}
    for entry in manifest.get("sequences", []):
        name = entry.get("name", "")
        info[name] = SequenceInfo(
            name=name,
            gt_first_s=float(entry.get("ground_truth_first_s", float("nan"))),
            gt_last_s=float(entry.get("ground_truth_last_s", float("nan"))),
            ball_radius_mm=float(entry.get("ball_radius_mm", float("nan"))),
            fx=float(entry.get("fx", float("nan"))),
            error=entry.get("error", "") or "",
        )
    return info


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------


@dataclass
class Metrics:
    sequence: str
    method: str
    n_estimates: int = 0
    n_matched: int = 0
    rmse_3d: float = float("nan")
    rmse_x: float = float("nan")
    rmse_y: float = float("nan")
    rmse_depth: float = float("nan")
    median_err: float = float("nan")
    p95_err: float = float("nan")
    depth_bias: float = float("nan")
    depth_mae: float = float("nan")
    coverage: float = float("nan")
    estimates_per_s: float = float("nan")
    first_estimate_latency_s: float = float("nan")
    runtime_ms_mean: float = float("nan")
    depth_buckets: list[tuple[float, float, int]] = field(default_factory=list)


def compute_metrics(
    sequence: str,
    method: str,
    detections: list[Detection],
    info: SequenceInfo | None,
    coverage_tol_s: float,
    require_visible: bool,
) -> Metrics:
    metrics = Metrics(sequence=sequence, method=method)
    metrics.n_estimates = len(detections)

    matched = [d for d in detections if d.gt_valid and not math.isnan(d.err_norm)]
    if require_visible:
        matched = [d for d in matched if d.gt_visible]
    metrics.n_matched = len(matched)

    if matched:
        metrics.rmse_3d = rms([d.err_norm for d in matched])
        metrics.rmse_x = rms([d.err[0] for d in matched])
        metrics.rmse_y = rms([d.err[1] for d in matched])
        metrics.rmse_depth = rms([d.err[2] for d in matched])
        metrics.median_err = quantile([d.err_norm for d in matched], 0.50)
        metrics.p95_err = quantile([d.err_norm for d in matched], 0.95)
        # Signed, so a systematic "too close to the camera" shows up as a
        # negative number rather than being hidden by the squaring.
        metrics.depth_bias = mean([d.err[2] for d in matched])
        metrics.depth_mae = mean([abs(d.err[2]) for d in matched])

        metrics.depth_buckets = compute_depth_buckets(matched)

    runtimes = [d.runtime_ms for d in detections if not math.isnan(d.runtime_ms)]
    metrics.runtime_ms_mean = mean(runtimes)

    # Coverage over the labelled ground-truth span: what fraction of the flight
    # this method actually produced an estimate for.
    if info is not None and not math.isnan(info.gt_first_s) and not math.isnan(info.gt_last_s):
        span = info.gt_last_s - info.gt_first_s
        if span > 0.0:
            metrics.estimates_per_s = len(detections) / span
            times = sorted(d.t for d in detections if not math.isnan(d.t))
            metrics.coverage = covered_fraction(
                times, info.gt_first_s, info.gt_last_s, coverage_tol_s
            )
            if times:
                metrics.first_estimate_latency_s = times[0] - info.gt_first_s

    return metrics


def covered_fraction(times: list[float], start: float, end: float, tol: float) -> float:
    """Fraction of [start, end] within `tol` of at least one estimate."""
    span = end - start
    if span <= 0.0 or not times:
        return 0.0

    intervals: list[tuple[float, float]] = []
    for t in times:
        lo = max(start, t - tol)
        hi = min(end, t + tol)
        if hi > lo:
            intervals.append((lo, hi))

    if not intervals:
        return 0.0

    intervals.sort()
    covered = 0.0
    current_lo, current_hi = intervals[0]
    for lo, hi in intervals[1:]:
        if lo > current_hi:
            covered += current_hi - current_lo
            current_lo, current_hi = lo, hi
        else:
            current_hi = max(current_hi, hi)
    covered += current_hi - current_lo

    return covered / span


def compute_depth_buckets(
    matched: list[Detection], bucket_width_m: float = 0.5
) -> list[tuple[float, float, int]]:
    """(bucket start, RMSE 3D, count) grouped by true depth."""
    buckets: dict[int, list[float]] = {}
    for d in matched:
        gt_depth = d.gt[2]
        if math.isnan(gt_depth):
            continue
        index = int(math.floor(gt_depth / bucket_width_m))
        buckets.setdefault(index, []).append(d.err_norm)

    return [
        (index * bucket_width_m, rms(errors), len(errors))
        for index, errors in sorted(buckets.items())
    ]


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

CSV_FIELDS = [
    "sequence",
    "method",
    "n_estimates",
    "n_matched",
    "rmse_3d_m",
    "rmse_x_m",
    "rmse_y_m",
    "rmse_depth_m",
    "median_err_m",
    "p95_err_m",
    "depth_bias_m",
    "depth_mae_m",
    "coverage",
    "estimates_per_s",
    "first_estimate_latency_s",
    "runtime_ms_mean",
]


def metrics_row(m: Metrics) -> dict[str, object]:
    def fmt(value: float, digits: int = 6) -> str:
        return "" if math.isnan(value) else f"{value:.{digits}f}"

    return {
        "sequence": m.sequence,
        "method": m.method,
        "n_estimates": m.n_estimates,
        "n_matched": m.n_matched,
        "rmse_3d_m": fmt(m.rmse_3d),
        "rmse_x_m": fmt(m.rmse_x),
        "rmse_y_m": fmt(m.rmse_y),
        "rmse_depth_m": fmt(m.rmse_depth),
        "median_err_m": fmt(m.median_err),
        "p95_err_m": fmt(m.p95_err),
        "depth_bias_m": fmt(m.depth_bias),
        "depth_mae_m": fmt(m.depth_mae),
        "coverage": fmt(m.coverage, 4),
        "estimates_per_s": fmt(m.estimates_per_s, 2),
        "first_estimate_latency_s": fmt(m.first_estimate_latency_s, 4),
        "runtime_ms_mean": fmt(m.runtime_ms_mean, 4),
    }


def write_csv(path: Path, all_metrics: list[Metrics]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for m in all_metrics:
            writer.writerow(metrics_row(m))


def cell(value: float, scale: float = 1.0, digits: int = 1, suffix: str = "") -> str:
    if math.isnan(value):
        return "n/a"
    return f"{value * scale:.{digits}f}{suffix}"


def write_markdown(
    path: Path,
    per_sequence: dict[str, dict[str, Metrics]],
    aggregate: dict[str, Metrics],
    sequence_info: dict[str, SequenceInfo],
    require_visible: bool,
    coverage_tol_s: float,
) -> None:
    lines: list[str] = []
    lines.append("# Trace vs circle fitting on the simulated sequences")
    lines.append("")
    lines.append(
        "Positions are compared in `camera_optical` metres against the "
        "time-interpolated ground truth; `depth` is the z axis, the one both "
        "methods derive from an apparent size."
    )
    lines.append("")
    if require_visible:
        lines.append(
            "Only estimates falling in a ground-truth stretch flagged visible "
            "are scored."
        )
        lines.append("")
    lines.append(
        f"Coverage = fraction of the labelled flight within "
        f"±{coverage_tol_s * 1e3:.0f} ms of at least one estimate."
    )
    lines.append("")

    lines.append("## Aggregate over all sequences")
    lines.append("")
    lines.append(_comparison_table(aggregate))
    lines.append("")

    lines.append("## Per sequence")
    lines.append("")
    for name in sorted(per_sequence):
        info = sequence_info.get(name)
        lines.append(f"### {name}")
        if info is not None and info.error:
            lines.append("")
            lines.append(f"> skipped by the benchmark: {info.error}")
        lines.append("")
        lines.append(_comparison_table(per_sequence[name]))
        lines.append("")

    lines.append("## RMSE 3D vs true depth")
    lines.append("")
    lines.append(
        "Aggregated over all sequences, in 0.5 m buckets of true depth. "
        "Depth from an apparent size degrades with distance: the further the "
        "ball, the fewer pixels carry the estimate."
    )
    lines.append("")
    lines.append(_depth_bucket_table(aggregate))
    lines.append("")

    path.write_text("\n".join(lines))


def _comparison_table(metrics_by_method: dict[str, Metrics]) -> str:
    present = [m for m in METHODS if m in metrics_by_method]
    if not present:
        return "_no data_"

    header = "| Metric | " + " | ".join(METHOD_LABEL[m] for m in present) + " |"
    separator = "|---|" + "---|" * len(present)

    rows = [
        ("Estimates", lambda m: str(m.n_estimates)),
        ("Scored against GT", lambda m: str(m.n_matched)),
        ("**RMSE 3D (mm)**", lambda m: cell(m.rmse_3d, 1e3)),
        ("RMSE X (mm)", lambda m: cell(m.rmse_x, 1e3)),
        ("RMSE Y (mm)", lambda m: cell(m.rmse_y, 1e3)),
        ("**RMSE depth (mm)**", lambda m: cell(m.rmse_depth, 1e3)),
        ("Median error (mm)", lambda m: cell(m.median_err, 1e3)),
        ("p95 error (mm)", lambda m: cell(m.p95_err, 1e3)),
        ("Depth bias, signed (mm)", lambda m: cell(m.depth_bias, 1e3)),
        ("Depth MAE (mm)", lambda m: cell(m.depth_mae, 1e3)),
        ("Coverage", lambda m: cell(m.coverage, 100.0, 1, " %")),
        ("Estimates / s", lambda m: cell(m.estimates_per_s, 1.0, 1)),
        ("First estimate (ms after GT start)", lambda m: cell(m.first_estimate_latency_s, 1e3)),
        ("Mean runtime / estimate (ms)", lambda m: cell(m.runtime_ms_mean, 1.0, 3)),
    ]

    lines = [header, separator]
    for label, accessor in rows:
        values = " | ".join(accessor(metrics_by_method[m]) for m in present)
        lines.append(f"| {label} | {values} |")

    return "\n".join(lines)


def _depth_bucket_table(aggregate: dict[str, Metrics]) -> str:
    present = [m for m in METHODS if m in aggregate and aggregate[m].depth_buckets]
    if not present:
        return "_no data_"

    starts = sorted({start for m in present for start, _, _ in aggregate[m].depth_buckets})
    header = "| True depth (m) | " + " | ".join(
        f"{METHOD_LABEL[m]} RMSE (mm) / n" for m in present
    ) + " |"
    separator = "|---|" + "---|" * len(present)

    lines = [header, separator]
    for start in starts:
        cells = []
        for method in present:
            match = next(
                (b for b in aggregate[method].depth_buckets if b[0] == start), None
            )
            cells.append("n/a" if match is None else f"{match[1] * 1e3:.1f} / {match[2]}")
        lines.append(f"| {start:.1f}–{start + 0.5:.1f} | " + " | ".join(cells) + " |")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Figures (optional)
# ---------------------------------------------------------------------------


def write_figures(
    run_dir: Path,
    detections: list[Detection],
    per_sequence: dict[str, dict[str, Metrics]],
    require_visible: bool,
) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    figures_dir = run_dir / "figures"
    figures_dir.mkdir(exist_ok=True)

    sequences = sorted({d.sequence for d in detections})

    for sequence in sequences:
        fig, (ax_err, ax_depth) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        has_data = False

        for method in METHODS:
            rows = [
                d
                for d in detections
                if d.sequence == sequence
                and d.method == method
                and d.gt_valid
                and not math.isnan(d.err_norm)
                and (d.gt_visible or not require_visible)
            ]
            if not rows:
                continue
            has_data = True
            rows.sort(key=lambda d: d.t)
            times = [d.t for d in rows]
            ax_err.plot(times, [d.err_norm * 1e3 for d in rows], ".", ms=3,
                        label=METHOD_LABEL[method])
            ax_depth.plot(times, [d.est[2] for d in rows], ".", ms=3,
                          label=f"{METHOD_LABEL[method]} estimate")

        if not has_data:
            plt.close(fig)
            continue

        truth = sorted(
            (d for d in detections if d.sequence == sequence and d.gt_valid),
            key=lambda d: d.t,
        )
        if truth:
            ax_depth.plot([d.t for d in truth], [d.gt[2] for d in truth], "k-",
                          lw=1, label="ground truth")

        ax_err.set_ylabel("3D error (mm)")
        ax_err.set_title(f"{sequence}: error vs time")
        ax_err.legend()
        ax_err.grid(alpha=0.3)

        ax_depth.set_ylabel("depth z (m)")
        ax_depth.set_xlabel("time (s)")
        ax_depth.set_title("estimated vs true depth")
        ax_depth.legend()
        ax_depth.grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(figures_dir / f"{sequence}_error.png", dpi=130)
        plt.close(fig)

    # RMSE per sequence, side by side.
    fig, ax = plt.subplots(figsize=(max(6, 1.6 * len(sequences)), 4.5))
    width = 0.38
    for offset, method in enumerate(METHODS):
        values = [
            per_sequence.get(s, {}).get(method, Metrics(s, method)).rmse_3d * 1e3
            if not math.isnan(
                per_sequence.get(s, {}).get(method, Metrics(s, method)).rmse_3d
            )
            else 0.0
            for s in sequences
        ]
        positions = [i + (offset - 0.5) * width for i in range(len(sequences))]
        ax.bar(positions, values, width=width, label=METHOD_LABEL[method])

    ax.set_xticks(range(len(sequences)))
    ax.set_xticklabels(sequences, rotation=30, ha="right")
    ax.set_ylabel("RMSE 3D (mm)")
    ax.set_title("3D RMSE against ground truth")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figures_dir / "rmse_by_sequence.png", dpi=130)
    plt.close(fig)

    return True


# ---------------------------------------------------------------------------


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path,
                        help="output directory of method_comparison_benchmark")
    parser.add_argument("--coverage-tol-ms", type=float, default=20.0,
                        help="half-width of the coverage window (default 20 ms)")
    parser.add_argument("--include-invisible", action="store_true",
                        help="also score estimates landing where the ball is "
                             "flagged out of frame (default: skip them)")
    parser.add_argument("--no-figures", action="store_true",
                        help="skip the matplotlib figures")
    args = parser.parse_args(argv)

    run_dir: Path = args.run_dir
    if not run_dir.is_dir():
        print(f"error: not a directory: {run_dir}", file=sys.stderr)
        return 1

    detections = load_detections(run_dir)
    if not detections:
        print(f"error: no detections_*.csv found in {run_dir}", file=sys.stderr)
        return 1

    sequence_info = load_sequence_info(run_dir)
    if not sequence_info:
        print(f"warning: no run_manifest.json in {run_dir}; coverage and rate "
              f"cannot be computed", file=sys.stderr)

    coverage_tol_s = args.coverage_tol_ms * 1e-3
    require_visible = not args.include_invisible

    sequences = sorted({d.sequence for d in detections})
    per_sequence: dict[str, dict[str, Metrics]] = {}
    ordered: list[Metrics] = []

    for sequence in sequences:
        per_sequence[sequence] = {}
        for method in METHODS:
            rows = [d for d in detections
                    if d.sequence == sequence and d.method == method]
            if not rows:
                continue
            metrics = compute_metrics(
                sequence, method, rows, sequence_info.get(sequence),
                coverage_tol_s, require_visible,
            )
            per_sequence[sequence][method] = metrics
            ordered.append(metrics)

    # Aggregate: pool every estimate, then average the per-sequence coverage and
    # rate (those are per-flight quantities, pooling them would be meaningless).
    aggregate: dict[str, Metrics] = {}
    for method in METHODS:
        rows = [d for d in detections if d.method == method]
        if not rows:
            continue
        combined = compute_metrics(
            "ALL", method, rows, None, coverage_tol_s, require_visible
        )
        coverages = [
            per_sequence[s][method].coverage
            for s in sequences
            if method in per_sequence[s]
            and not math.isnan(per_sequence[s][method].coverage)
        ]
        rates = [
            per_sequence[s][method].estimates_per_s
            for s in sequences
            if method in per_sequence[s]
            and not math.isnan(per_sequence[s][method].estimates_per_s)
        ]
        latencies = [
            per_sequence[s][method].first_estimate_latency_s
            for s in sequences
            if method in per_sequence[s]
            and not math.isnan(per_sequence[s][method].first_estimate_latency_s)
        ]
        combined.coverage = mean(coverages) if coverages else float("nan")
        combined.estimates_per_s = mean(rates) if rates else float("nan")
        combined.first_estimate_latency_s = mean(latencies) if latencies else float("nan")
        aggregate[method] = combined
        ordered.append(combined)

    csv_path = run_dir / "comparison_summary.csv"
    md_path = run_dir / "comparison_summary.md"
    write_csv(csv_path, ordered)
    write_markdown(md_path, per_sequence, aggregate, sequence_info,
                   require_visible, coverage_tol_s)

    figures_written = False
    if not args.no_figures:
        figures_written = write_figures(run_dir, detections, per_sequence, require_visible)

    print(f"Sequences: {len(sequences)}")
    for method in METHODS:
        if method in aggregate:
            m = aggregate[method]
            print(
                f"  {METHOD_LABEL[method]:>14}: {m.n_estimates:6d} estimates, "
                f"{m.n_matched:6d} scored, RMSE 3D = {cell(m.rmse_3d, 1e3)} mm, "
                f"RMSE depth = {cell(m.rmse_depth, 1e3)} mm, "
                f"coverage = {cell(m.coverage, 100.0, 1)} %"
            )
    print(f"\nWrote {csv_path}")
    print(f"Wrote {md_path}")
    if figures_written:
        print(f"Wrote {run_dir / 'figures'}")
    elif not args.no_figures:
        print("matplotlib not available: figures skipped (tables are complete)")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
