#!/usr/bin/env python3
"""Search good Trace (or circle-fitting) parameters against Isaac ground truth.

Drives `build/ball_tracker_h5_benchmark` over a set of sequences, scores each
candidate parameter set against the ground-truth CSVs, and reports the best one.

Two things keep the result honest:

* **Train/test split.** Parameters are searched on the train sequences only and
  re-scored once on held-out test sequences. The test score is the one to
  quote; a large train/test gap means the search overfitted.
* **UI clamp bounds.** The search space matches the clamps in `Ui` (Gui.h), so
  a tuned value can actually be dialled into the live GUI. The offline
  benchmark does not clamp, and a value outside those bounds would silently
  behave differently live.

Detection rate is a hard constraint rather than a weighted term: a candidate
that detects almost nothing can post an excellent RMSE on the few easy samples
it kept, so candidates below the floor are rejected outright.

Examples
--------
Tune Trace on the three detailed sequences::

    python3 scripts/tune_trace_params.py --sequences sequences/sequence_000{1,2,3} \\
        --trials 200 --jobs 8

Tune on a benchmark dataset, holding out a quarter of the sequences::

    python3 scripts/tune_trace_params.py \\
        --benchmark /home/rigon/Documents/EventGen/ball_event_dataset_v0/benchmark/datasets/benchmark_fast_throw_0500 \\
        --limit 40 --test-fraction 0.25 --trials 300 --jobs 8
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEFAULT_BINARY = REPO / "build" / "ball_tracker_h5_benchmark"

# name -> (low, high, kind). Bounds mirror the Ui clamps in Gui.h so a tuned
# value stays dialable in the live GUI; `border_pixels` has no slider and is
# bounded by the clamp in EstimateSupportedEdges.
TRACE_SPACE = {
    "trace_memory_ms": (1.0, 500.0, "float"),
    "line_bin_width_px": (1.0, 48.0, "float"),
    "local_window_px": (8.0, 240.0, "float"),
    "pca_period_ms": (2.0, 80.0, "float"),
    "width_step_px": (8.0, 90.0, "float"),
    "support_divisor": (8.0, 60.0, "float"),
    "support_min_count": (1, 20, "int"),
    "support_max_count": (2, 30, "int"),
    "support_radius_px": (0.5, 4.0, "float"),
    "border_percent": (0.0, 10.0, "float"),
    "border_pixels": (0.0, 4.0, "float"),
    "line_order": (["linear", "quad"], None, "choice"),
    "edge_refine": ([True, False], None, "choice"),
    "width_smoothing": ([True, False], None, "choice"),
    "polarity_mode": (["all", "positive", "negative"], None, "choice"),
}

CIRCLE_SPACE = {
    "window_ms": (1.0, 500.0, "float"),
    "bandwidth": (1, 300, "int"),
    "min_nb": (1, 100, "int"),
    "max_events": (100, 20000, "int"),
    "alpha": (0.0, 1.0, "float"),
    "coef": (0.0, 2.0, "float"),
    "filter_size": (1.0, 300.0, "float"),
    "max_residual": (1.0, 60.0, "float"),
    "sym_coef": (0.0, 120.0, "float"),
    "sym_coef2": (0.0, 1000.0, "float"),
    "depth_jump_gate_mm": (50.0, 5000.0, "float"),
    "slice_mode": (0, 2, "int"),
}

TRACE_BASELINE = {
    "trace_memory_ms": 150.0,
    "line_bin_width_px": 4.0,
    "local_window_px": 65.69,
    "pca_period_ms": 36.10,
    "width_step_px": 8.0,
    "support_divisor": 28.0,
    "support_min_count": 3,
    "support_max_count": 9,
    "support_radius_px": 1.75,
    "border_percent": 3.5,
    "border_pixels": 0.0,
    "line_order": "quad",
    "edge_refine": False,
    "width_smoothing": False,
    "polarity_mode": "all",
}

CIRCLE_BASELINE = {
    "window_ms": 15.0,
    "bandwidth": 50,
    "min_nb": 5,
    "max_events": 1000,
    "alpha": 0.5,
    "coef": 0.45,
    "filter_size": 115.0,
    "max_residual": 19.0,
    "sym_coef": 29.0,
    "sym_coef2": 157.0,
    "depth_jump_gate_mm": 250.0,
    "slice_mode": 0,
}


# --------------------------------------------------------------------------- #
# scoring


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(value: str | None) -> float:
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return math.nan


class Sequence:
    """A sequence plus its ground truth, preloaded once."""

    def __init__(self, path: Path):
        self.path = path
        self.name = path.name
        self.events = path / "events_v2e" / "events_filtered.h5"
        self.ground_truth = path / "labels" / "ground_truth.csv"
        self.camera = path / "camera" / "intrinsics.json"
        self.metadata = path / "metadata.json"
        rows = read_csv_rows(self.ground_truth)
        self.times = [to_float(r["timestamp_s"]) for r in rows]
        self.gt = [
            (
                to_float(r["ball_x_cam_m"]),
                to_float(r["ball_y_cam_m"]),
                to_float(r["ball_z_cam_m"]),
            )
            for r in rows
        ]

    def usable(self) -> bool:
        return all(p.exists() for p in (self.events, self.ground_truth, self.camera, self.metadata))

    def interpolate(self, t: float) -> tuple[float, float, float] | None:
        times = self.times
        if not times or t < times[0] - 1e-9 or t > times[-1] + 1e-9:
            return None
        hi = 0
        while hi < len(times) and times[hi] < t:
            hi += 1
        if hi == 0:
            lo = hi = 0
            a = 0.0
        elif hi >= len(times):
            lo = hi = len(times) - 1
            a = 0.0
        else:
            lo = hi - 1
            dt = times[hi] - times[lo]
            a = 0.0 if dt <= 0 else (t - times[lo]) / dt
        p, q = self.gt[lo], self.gt[hi]
        return tuple(p[i] + (q[i] - p[i]) * a for i in range(3))  # type: ignore[return-value]

    def score(self, detections_csv: Path) -> tuple[float, float]:
        """(rmse_3d, detection_rate); rmse is NaN when nothing was detected."""
        rows = read_csv_rows(detections_csv)
        if not rows:
            return math.nan, 0.0
        errors = []
        detected = 0
        for r in rows:
            if r.get("detected") != "1":
                continue
            detected += 1
            gt = self.interpolate(to_float(r["timestamp_s"]))
            if gt is None:
                continue
            est = (to_float(r["x_est_m"]), to_float(r["y_est_m"]), to_float(r["z_est_m"]))
            if any(math.isnan(v) for v in est):
                continue
            errors.append(math.dist(est, gt))
        rate = detected / len(rows)
        if not errors:
            return math.nan, rate
        return math.sqrt(sum(e * e for e in errors) / len(errors)), rate


# --------------------------------------------------------------------------- #
# candidate evaluation


def write_config(path: Path, method: str, params: dict, output_period_ms: float) -> None:
    lines = ["output:", f"  output_period_ms: {output_period_ms}", f"{method}:"]
    for key, value in params.items():
        if isinstance(value, bool):
            lines.append(f"  {key}: {'true' if value else 'false'}")
        elif isinstance(value, str):
            lines.append(f'  {key}: "{value}"')
        else:
            lines.append(f"  {key}: {value}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_one(binary: Path, seq: Sequence, method: str, config: Path, workdir: Path) -> tuple[float, float]:
    out = workdir / f"{seq.name}_{method}.csv"
    cmd = [
        str(binary),
        "--events-h5", str(seq.events),
        "--ground-truth", str(seq.ground_truth),
        "--camera", str(seq.camera),
        "--metadata", str(seq.metadata),
        f"--output-{method}", str(out),
        "--runtime-output", str(workdir / f"{seq.name}_runtime.json"),
        "--config", str(config),
        "--mode", method,
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not out.exists():
        return math.nan, 0.0
    return seq.score(out)


def evaluate(
    binary: Path,
    sequences: list[Sequence],
    method: str,
    params: dict,
    output_period_ms: float,
    jobs: int,
    min_rate: float,
) -> dict:
    """Mean RMSE over sequences that detected anything, plus coverage stats."""
    with tempfile.TemporaryDirectory(prefix="tune_trace_") as tmp:
        workdir = Path(tmp)
        config = workdir / "config.yaml"
        write_config(config, method, params, output_period_ms)
        with ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
            results = list(pool.map(lambda s: run_one(binary, s, method, config, workdir), sequences))

    rmses = [r for r, _ in results if math.isfinite(r)]
    rates = [rate for _, rate in results]
    covered = len(rmses)
    mean_rate = sum(rates) / len(rates) if rates else 0.0
    mean_rmse = sum(rmses) / len(rmses) if rmses else math.nan

    # A candidate that detects almost nothing scores a flattering RMSE on the
    # handful of easy samples it kept, so coverage and rate gate the result
    # instead of being averaged into it.
    feasible = (
        covered == len(sequences)
        and mean_rate >= min_rate
        and math.isfinite(mean_rmse)
    )
    return {
        "mean_rmse_3d_m": mean_rmse,
        "mean_detection_rate": mean_rate,
        "sequences_with_detection": covered,
        "sequences": len(sequences),
        "feasible": feasible,
        "score": mean_rmse if feasible else math.inf,
    }


# --------------------------------------------------------------------------- #
# search


def sample(space: dict, rng: random.Random) -> dict:
    out = {}
    for key, (low, high, kind) in space.items():
        if kind == "choice":
            out[key] = rng.choice(low)
        elif kind == "int":
            out[key] = rng.randint(int(low), int(high))
        else:
            out[key] = round(rng.uniform(low, high), 4)
    if "support_max_count" in out and "support_min_count" in out:
        out["support_max_count"] = max(out["support_max_count"], out["support_min_count"])
    return out


def coordinate_values(space: dict, key: str, current, rng: random.Random, steps: int) -> list:
    low, high, kind = space[key]
    if kind == "choice":
        return [v for v in low if v != current]
    values = []
    span = (high - low) / 4.0
    for _ in range(steps):
        candidate = current + rng.uniform(-span, span)
        candidate = min(max(candidate, low), high)
        values.append(int(round(candidate)) if kind == "int" else round(candidate, 4))
    # Always probe the bounds: several of these parameters turn out to be
    # monotonic over the useful range.
    values += [low, high] if kind != "int" else [int(low), int(high)]
    return [v for v in values if v != current]


def search(args, space: dict, baseline: dict, train: list[Sequence], binary: Path) -> tuple[dict, dict, list]:
    rng = random.Random(args.seed)
    trials: list[dict] = []

    def score_of(params: dict, label: str) -> dict:
        result = evaluate(binary, train, args.method, params, args.output_period_ms, args.jobs, args.min_detection_rate)
        trials.append({"label": label, "params": dict(params), **result})
        flag = "" if result["feasible"] else "  (rejeté: couverture/taux)"
        print(
            f"  [{label:>18}] RMSE3D={result['mean_rmse_3d_m']:.4f} m  "
            f"rate={result['mean_detection_rate']:.2f}  "
            f"cov={result['sequences_with_detection']}/{result['sequences']}{flag}",
            flush=True,
        )
        return result

    print(f"Baseline ({args.method}) sur {len(train)} séquences d'entraînement:")
    best_params = dict(baseline)
    best = score_of(best_params, "baseline")
    if not math.isfinite(best["score"]):
        print("  ATTENTION: la baseline ne satisfait pas la contrainte de taux; "
              "abaisse --min-detection-rate pour que la comparaison ait un sens.")

    print(f"\nRecherche aléatoire ({args.trials} essais):")
    for i in range(args.trials):
        params = sample(space, rng)
        result = score_of(params, f"random {i + 1}")
        if result["score"] < best["score"]:
            best, best_params = result, params
            print(f"    -> nouveau meilleur: {best['score']:.4f} m")

    print(f"\nDescente par coordonnées ({args.rounds} passes):")
    for round_index in range(args.rounds):
        improved = False
        for key in space:
            for value in coordinate_values(space, key, best_params[key], rng, args.coordinate_steps):
                params = dict(best_params)
                params[key] = value
                if "support_max_count" in params and "support_min_count" in params:
                    params["support_max_count"] = max(params["support_max_count"], params["support_min_count"])
                result = score_of(params, f"r{round_index + 1} {key}")
                if result["score"] < best["score"]:
                    best, best_params, improved = result, params, True
                    print(f"    -> {key} = {value}  ({best['score']:.4f} m)")
        if not improved:
            print("  (plus d'amélioration, arrêt)")
            break

    return best_params, best, trials


# --------------------------------------------------------------------------- #


def collect_sequences(args) -> list[Sequence]:
    paths: list[Path] = [Path(p) for p in args.sequences]
    if args.benchmark:
        paths += sorted((Path(args.benchmark) / "sequences").glob("sequence_*"))
    sequences = []
    for path in paths:
        seq = Sequence(path) if (path / "labels" / "ground_truth.csv").exists() else None
        if seq is None or not seq.usable():
            continue
        sequences.append(seq)
        if args.limit and len(sequences) >= args.limit:
            break
    return sequences


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sequences", nargs="*", default=[], help="sequence directories")
    p.add_argument("--benchmark", help="benchmark dataset root (adds its sequences/)")
    p.add_argument("--limit", type=int, help="cap the number of sequences used")
    p.add_argument("--method", choices=["trace", "circle"], default="trace")
    p.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    p.add_argument("--trials", type=int, default=120, help="random-search candidates")
    p.add_argument("--rounds", type=int, default=2, help="coordinate-descent passes")
    p.add_argument("--coordinate-steps", type=int, default=3, help="probes per parameter per pass")
    p.add_argument("--test-fraction", type=float, default=0.25, help="held-out share of sequences")
    p.add_argument("--min-detection-rate", type=float, default=0.30,
                   help="candidates below this mean detection rate are rejected")
    p.add_argument("--output-period-ms", type=float, default=2.0)
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", type=Path, help="write the result JSON here")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if not args.binary.exists():
        print(f"Binaire introuvable: {args.binary}\nCompile-le avec: source env.sh && build", file=sys.stderr)
        return 1

    sequences = collect_sequences(args)
    if len(sequences) < 2:
        print("Il faut au moins 2 séquences utilisables (events_filtered.h5 + labels + camera + metadata).", file=sys.stderr)
        return 1

    rng = random.Random(args.seed)
    shuffled = sequences[:]
    rng.shuffle(shuffled)
    n_test = max(1, round(len(shuffled) * args.test_fraction)) if args.test_fraction > 0 else 0
    test, train = shuffled[:n_test], shuffled[n_test:]
    if not train:
        train, test = shuffled, []

    space = TRACE_SPACE if args.method == "trace" else CIRCLE_SPACE
    baseline = TRACE_BASELINE if args.method == "trace" else CIRCLE_BASELINE

    print(f"{len(sequences)} séquences: {len(train)} entraînement, {len(test)} test")
    print(f"Test: {', '.join(s.name for s in test) or '(aucune)'}\n")

    best_params, best_train, trials = search(args, space, baseline, train, args.binary)

    result = {
        "method": args.method,
        "train_sequences": [s.name for s in train],
        "test_sequences": [s.name for s in test],
        "baseline_params": baseline,
        "best_params": best_params,
        "train": best_train,
    }

    if test:
        print("\nRe-scoring sur les séquences held-out:")
        base_test = evaluate(args.binary, test, args.method, baseline, args.output_period_ms, args.jobs, args.min_detection_rate)
        best_test = evaluate(args.binary, test, args.method, best_params, args.output_period_ms, args.jobs, args.min_detection_rate)
        result["baseline_test"] = base_test
        result["best_test"] = best_test
        print(f"  baseline : RMSE3D={base_test['mean_rmse_3d_m']:.4f} m  rate={base_test['mean_detection_rate']:.2f}")
        print(f"  optimisé : RMSE3D={best_test['mean_rmse_3d_m']:.4f} m  rate={best_test['mean_detection_rate']:.2f}")
        if math.isfinite(base_test["mean_rmse_3d_m"]) and math.isfinite(best_test["mean_rmse_3d_m"]):
            gain = 100.0 * (1.0 - best_test["mean_rmse_3d_m"] / base_test["mean_rmse_3d_m"])
            print(f"  gain sur données non vues: {gain:+.1f} %")
            if best_train["mean_rmse_3d_m"] < 0.6 * best_test["mean_rmse_3d_m"]:
                print("  ATTENTION: écart train/test important, la recherche a probablement surappris.")

    print("\nMeilleurs paramètres:")
    for key, value in best_params.items():
        marker = "" if value == baseline.get(key) else "   <- modifié"
        print(f"  {key}: {value}{marker}")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
        trials_csv = args.out.with_suffix(".trials.csv")
        keys = sorted({k for t in trials for k in t["params"]})
        with trials_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["label", "score", "mean_rmse_3d_m", "mean_detection_rate", "feasible", *keys])
            for t in trials:
                writer.writerow([
                    t["label"], t["score"], t["mean_rmse_3d_m"], t["mean_detection_rate"], t["feasible"],
                    *[t["params"].get(k, "") for k in keys],
                ])
        print(f"\nÉcrit: {args.out} et {trials_csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
