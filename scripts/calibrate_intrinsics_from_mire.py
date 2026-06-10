#!/usr/bin/env python3
"""Calibrate DVXplorer intrinsics from blinking-mire observation JSON files."""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2 as cv
import numpy as np


DEFAULT_INPUT_DIR = "recordings/mire_calibration"
DEFAULT_OUTPUT_XML = "recordings/mire_calibration/intrinsics_from_mire.xml"
DEFAULT_OUTPUT_JSON = "recordings/mire_calibration/intrinsics_from_mire_report.json"


@dataclass
class Observation:
    path: Path
    image_size: Tuple[int, int]
    object_points: np.ndarray
    image_points: np.ndarray
    matched_count: int
    event_count: int
    created_at: str


def load_observation(path: Path, min_points: int) -> Optional[Observation]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)

    camera = data.get("camera", {})
    resolution = camera.get("resolution_px", {})
    width = int(resolution.get("width", 0))
    height = int(resolution.get("height", 0))
    if width <= 0 or height <= 0:
        print(f"[skip] {path}: missing camera resolution", file=sys.stderr)
        return None

    detection = data.get("detection", {})
    matches = detection.get("matches", [])
    if len(matches) < min_points:
        print(f"[skip] {path}: only {len(matches)} matches", file=sys.stderr)
        return None

    def match_key(match: Dict[str, object]) -> Tuple[int, int]:
        return int(match.get("row", 0)), int(match.get("col", 0))

    object_points: List[List[float]] = []
    image_points: List[List[float]] = []
    for match in sorted(matches, key=match_key):
        object_mm = match["object_mm"]
        camera_px = match["camera_px"]
        object_points.append(
            [
                float(object_mm["x"]),
                float(object_mm["y"]),
                float(object_mm.get("z", 0.0)),
            ]
        )
        image_points.append([float(camera_px["x"]), float(camera_px["y"])])

    return Observation(
        path=path,
        image_size=(width, height),
        object_points=np.asarray(object_points, dtype=np.float32),
        image_points=np.asarray(image_points, dtype=np.float32),
        matched_count=len(matches),
        event_count=int(camera.get("events_accumulated", 0)),
        created_at=str(data.get("created_at", "")),
    )


def collect_observations(args: argparse.Namespace) -> List[Observation]:
    pattern = str(Path(args.input_dir) / args.glob)
    paths = [Path(path) for path in sorted(glob.glob(pattern))]
    observations: List[Observation] = []
    for path in paths:
        observation = load_observation(path, args.min_points)
        if observation is not None:
            observations.append(observation)
    return observations


def validate_observations(observations: Sequence[Observation], min_views: int) -> Tuple[int, int]:
    if len(observations) < min_views:
        raise RuntimeError(
            f"Need at least {min_views} valid observations, found {len(observations)}. "
            "Capture the mire from several different positions/orientations."
        )

    image_sizes = {obs.image_size for obs in observations}
    if len(image_sizes) != 1:
        raise RuntimeError(f"All observations must share one image size, found: {sorted(image_sizes)}")

    point_counts = {len(obs.object_points) for obs in observations}
    if len(point_counts) != 1:
        print(
            f"[warn] observations have different point counts: {sorted(point_counts)}",
            file=sys.stderr,
        )

    return observations[0].image_size


def view_geometry(observation: Observation) -> Dict[str, float]:
    points = observation.image_points
    min_xy = np.min(points, axis=0)
    max_xy = np.max(points, axis=0)
    width = float(max_xy[0] - min_xy[0])
    height = float(max_xy[1] - min_xy[1])
    return {
        "center_x": float((min_xy[0] + max_xy[0]) * 0.5),
        "center_y": float((min_xy[1] + max_xy[1]) * 0.5),
        "width_px": width,
        "height_px": height,
        "area_px2": width * height,
    }


def diversity_warnings(observations: Sequence[Observation]) -> Tuple[List[str], List[Dict[str, float]]]:
    geometries = [view_geometry(obs) for obs in observations]
    warnings: List[str] = []
    unique_groups: List[Dict[str, float]] = []

    for geometry in geometries:
        duplicate = False
        for group in unique_groups:
            center_shift = math.hypot(
                geometry["center_x"] - group["center_x"],
                geometry["center_y"] - group["center_y"],
            )
            scale_shift = abs(geometry["area_px2"] - group["area_px2"]) / max(1.0, group["area_px2"])
            if center_shift < 8.0 and scale_shift < 0.04:
                duplicate = True
                break
        if not duplicate:
            unique_groups.append(geometry)

    if len(unique_groups) < max(3, len(observations) // 2):
        warnings.append(
            f"Only {len(unique_groups)} clearly different view groups found for "
            f"{len(observations)} observations. Move/rotate the screen more between captures."
        )

    centers = np.asarray([[g["center_x"], g["center_y"]] for g in geometries], dtype=np.float64)
    if centers.size > 0:
        span = np.ptp(centers, axis=0)
        if span[0] < 120.0 or span[1] < 100.0:
            warnings.append(
                "The mire centers cover a small part of the sensor. "
                "Use poses near corners and edges for better intrinsics."
            )

    areas = np.asarray([g["area_px2"] for g in geometries], dtype=np.float64)
    if areas.size > 0:
        area_ratio = float(np.max(areas) / max(1.0, np.min(areas)))
        if area_ratio < 1.4:
            warnings.append(
                "The apparent mire size changes little between views. "
                "Capture near/far poses to constrain focal length."
            )

    return warnings, geometries


def calibration_flags(args: argparse.Namespace) -> int:
    flags = 0
    if args.zero_tangent_dist:
        flags |= cv.CALIB_ZERO_TANGENT_DIST
    if args.fix_k1:
        flags |= cv.CALIB_FIX_K1
    if args.fix_k2:
        flags |= cv.CALIB_FIX_K2
    if args.fix_k3:
        flags |= cv.CALIB_FIX_K3
    if args.fix_principal_point:
        flags |= cv.CALIB_FIX_PRINCIPAL_POINT
    if args.fix_aspect_ratio:
        flags |= cv.CALIB_FIX_ASPECT_RATIO
    return flags


def run_calibration(
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    flags: int,
    use_intrinsic_guess: bool,
) -> Tuple[
    float,
    np.ndarray,
    np.ndarray,
    Sequence[np.ndarray],
    Sequence[np.ndarray],
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    object_points = [obs.object_points for obs in observations]
    image_points = [obs.image_points for obs in observations]

    camera_matrix: Optional[np.ndarray] = None
    if use_intrinsic_guess or flags & cv.CALIB_FIX_ASPECT_RATIO:
        camera_matrix = cv.initCameraMatrix2D(object_points, image_points, image_size, 1.0)
        flags |= cv.CALIB_USE_INTRINSIC_GUESS
    else:
        camera_matrix = None

    dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    criteria = (
        cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT,
        100,
        1e-8,
    )

    result = cv.calibrateCameraExtended(
        object_points,
        image_points,
        image_size,
        camera_matrix,
        dist_coeffs,
        flags=flags,
        criteria=criteria,
    )
    return result


def reprojection_errors(
    observations: Sequence[Observation],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvecs: Sequence[np.ndarray],
    tvecs: Sequence[np.ndarray],
) -> Tuple[List[Dict[str, object]], float, float]:
    reports: List[Dict[str, object]] = []
    all_errors: List[float] = []

    for obs, rvec, tvec in zip(observations, rvecs, tvecs):
        projected, _ = cv.projectPoints(obs.object_points, rvec, tvec, camera_matrix, dist_coeffs)
        projected = projected.reshape(-1, 2)
        diff = projected - obs.image_points
        errors = np.linalg.norm(diff, axis=1)
        all_errors.extend(float(error) for error in errors)
        rms = math.sqrt(float(np.mean(errors * errors))) if errors.size else float("nan")
        reports.append(
            {
                "file": str(obs.path),
                "created_at": obs.created_at,
                "points": int(len(obs.object_points)),
                "events_accumulated": obs.event_count,
                "rms_px": rms,
                "mean_px": float(np.mean(errors)) if errors.size else float("nan"),
                "max_px": float(np.max(errors)) if errors.size else float("nan"),
                "rvec": np.asarray(rvec).reshape(-1).astype(float).tolist(),
                "tvec_mm": np.asarray(tvec).reshape(-1).astype(float).tolist(),
            }
        )

    all_errors_array = np.asarray(all_errors, dtype=np.float64)
    mean_error = float(np.mean(all_errors_array)) if all_errors_array.size else float("nan")
    max_error = float(np.max(all_errors_array)) if all_errors_array.size else float("nan")
    return reports, mean_error, max_error


def solve_view_with_fixed_intrinsics(
    observation: Observation,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Dict[str, object]:
    ok, rvec, tvec = cv.solvePnP(
        observation.object_points,
        observation.image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return {
            "file": str(observation.path),
            "rms_px": float("inf"),
            "mean_px": float("inf"),
            "max_px": float("inf"),
        }

    projected, _ = cv.projectPoints(
        observation.object_points,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - observation.image_points, axis=1)
    return {
        "file": str(observation.path),
        "rms_px": math.sqrt(float(np.mean(errors * errors))),
        "mean_px": float(np.mean(errors)),
        "max_px": float(np.max(errors)),
    }


def fixed_intrinsics_view_errors(
    observations: Sequence[Observation],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> List[Dict[str, object]]:
    return [
        solve_view_with_fixed_intrinsics(obs, camera_matrix, dist_coeffs)
        for obs in observations
    ]


def select_robust_observations(
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    args: argparse.Namespace,
) -> Tuple[List[Observation], Dict[str, object]]:
    if len(observations) <= args.ransac_sample_size:
        return list(observations), {
            "enabled": True,
            "reason": "not enough observations for random subsampling",
            "selected_files": [str(obs.path) for obs in observations],
            "excluded_files": [],
        }

    rng = np.random.default_rng(args.ransac_seed)
    flags = calibration_flags(args)
    sample_size = max(args.min_views, min(args.ransac_sample_size, len(observations)))
    best_inliers: List[int] = []
    best_mean = float("inf")
    best_errors: List[Dict[str, object]] = []

    for _ in range(args.ransac_iterations):
        sample_indices = sorted(rng.choice(len(observations), size=sample_size, replace=False).tolist())
        sample = [observations[index] for index in sample_indices]
        try:
            rms, camera_matrix, dist_coeffs, *_ = run_calibration(
                sample,
                image_size,
                flags,
                args.use_intrinsic_guess,
            )
        except cv.error:
            continue

        errors = fixed_intrinsics_view_errors(observations, camera_matrix, dist_coeffs)
        inliers = [
            index for index, report in enumerate(errors)
            if float(report["rms_px"]) <= args.ransac_threshold_px
        ]
        if not inliers:
            continue
        mean = float(np.mean([float(errors[index]["rms_px"]) for index in inliers]))
        better = len(inliers) > len(best_inliers)
        tied_but_cleaner = len(inliers) == len(best_inliers) and mean < best_mean
        if better or tied_but_cleaner:
            best_inliers = inliers
            best_mean = mean
            best_errors = errors

    if not best_inliers:
        return list(observations), {
            "enabled": True,
            "reason": "no robust consensus found; using all observations",
            "selected_files": [str(obs.path) for obs in observations],
            "excluded_files": [],
        }

    for _ in range(args.ransac_refine_iterations):
        selected = [observations[index] for index in best_inliers]
        if len(selected) < args.min_views:
            break
        rms, camera_matrix, dist_coeffs, *_ = run_calibration(
            selected,
            image_size,
            flags,
            args.use_intrinsic_guess,
        )
        errors = fixed_intrinsics_view_errors(observations, camera_matrix, dist_coeffs)
        next_inliers = [
            index for index, report in enumerate(errors)
            if float(report["rms_px"]) <= args.ransac_threshold_px
        ]
        if next_inliers == best_inliers or len(next_inliers) < args.min_views:
            best_errors = errors
            break
        best_inliers = next_inliers
        best_errors = errors

    selected_indices = set(best_inliers)
    selected_observations = [
        obs for index, obs in enumerate(observations)
        if index in selected_indices
    ]
    excluded_observations = [
        obs for index, obs in enumerate(observations)
        if index not in selected_indices
    ]
    error_by_file = {
        str(report["file"]): report
        for report in best_errors
    }

    return selected_observations, {
        "enabled": True,
        "threshold_px": args.ransac_threshold_px,
        "iterations": args.ransac_iterations,
        "sample_size": sample_size,
        "selected_count": len(selected_observations),
        "excluded_count": len(excluded_observations),
        "selected_files": [str(obs.path) for obs in selected_observations],
        "excluded_files": [str(obs.path) for obs in excluded_observations],
        "fixed_intrinsics_view_errors": error_by_file,
    }


def write_opencv_xml(
    path: Path,
    camera_name: str,
    image_size: Tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rms: float,
    args: argparse.Namespace,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv.FileStorage(str(path), cv.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Could not write {path}")

    fs.startWriteStruct(camera_name, cv.FileNode_MAP)
    fs.write("camera_matrix", camera_matrix)
    fs.write("distortion_coefficients", dist_coeffs.reshape(-1, 1))
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("use_fisheye_model", 0)
    fs.write("calibration_error", float(rms))
    fs.endWriteStruct()

    fs.write("use_fisheye_model", 0)
    fs.write("type", "camera")
    fs.write("pattern_width", 5)
    fs.write("pattern_height", 4)
    fs.write("pattern_type", "blinking_mire")
    fs.write("board_width", 5)
    fs.write("board_height", 4)
    fs.write("square_size", 0.0)
    fs.write("calibration_error", float(rms))
    fs.write("calibration_time", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    fs.write("source", "scripts/calibrate_intrinsics_from_mire.py")
    fs.write("input_dir", str(args.input_dir))
    fs.release()


def write_report_json(
    path: Path,
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rms: float,
    std_intrinsics: np.ndarray,
    per_view_errors: np.ndarray,
    view_reports: Sequence[Dict[str, object]],
    mean_error: float,
    max_error: float,
    diversity: Sequence[Dict[str, float]],
    warnings: Sequence[str],
    robust_info: Dict[str, object],
    args: argparse.Namespace,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "created_at": datetime.now().isoformat(timespec="milliseconds"),
        "input_dir": str(args.input_dir),
        "glob": args.glob,
        "observation_count": len(observations),
        "image_size": {"width": image_size[0], "height": image_size[1]},
        "rms_px": float(rms),
        "mean_reprojection_error_px": mean_error,
        "max_reprojection_error_px": max_error,
        "camera_matrix": camera_matrix.astype(float).tolist(),
        "distortion_coefficients": dist_coeffs.reshape(-1).astype(float).tolist(),
        "std_deviations_intrinsics": np.asarray(std_intrinsics).reshape(-1).astype(float).tolist(),
        "opencv_per_view_errors": np.asarray(per_view_errors).reshape(-1).astype(float).tolist(),
        "views": list(view_reports),
        "view_geometries": list(diversity),
        "warnings": list(warnings),
        "robust_selection": robust_info,
        "flags": {
            "zero_tangent_dist": args.zero_tangent_dist,
            "fix_k1": args.fix_k1,
            "fix_k2": args.fix_k2,
            "fix_k3": args.fix_k3,
            "fix_principal_point": args.fix_principal_point,
            "fix_aspect_ratio": args.fix_aspect_ratio,
            "use_intrinsic_guess": args.use_intrinsic_guess,
        },
    }
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)


def print_summary(
    observations: Sequence[Observation],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rms: float,
    mean_error: float,
    max_error: float,
    view_reports: Sequence[Dict[str, object]],
    warnings: Sequence[str],
) -> None:
    print(f"Observations used: {len(observations)}")
    print(f"Image size: {observations[0].image_size[0]}x{observations[0].image_size[1]}")
    print(f"RMS reprojection error: {rms:.4f} px")
    print(f"Mean / max reprojection error: {mean_error:.4f} / {max_error:.4f} px")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(dist_coeffs.reshape(-1))
    print("Per-view RMS:")
    for view in view_reports:
        print(f"  {Path(str(view['file'])).name}: {float(view['rms_px']):.4f} px")
    for warning in warnings:
        print(f"[warn] {warning}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", default=DEFAULT_INPUT_DIR, help="Directory containing mire_observation_*.json files.")
    parser.add_argument("--glob", default="mire_observation_*.json", help="Input JSON glob relative to input-dir.")
    parser.add_argument("--output-xml", default=DEFAULT_OUTPUT_XML, help="OpenCV XML calibration output.")
    parser.add_argument("--output-json", default=DEFAULT_OUTPUT_JSON, help="Human-readable JSON report output.")
    parser.add_argument("--camera-name", default="DVXplorer_mire", help="Top-level camera node name in the OpenCV XML.")
    parser.add_argument("--min-points", type=int, default=12, help="Minimum matched points per observation.")
    parser.add_argument("--min-views", type=int, default=3, help="Minimum observations required for calibration.")
    parser.add_argument("--use-intrinsic-guess", action="store_true", help="Initialize K with cv.initCameraMatrix2D.")
    parser.add_argument("--zero-tangent-dist", action="store_true", help="Force p1=p2=0.")
    parser.add_argument("--fix-k1", action="store_true", help="Keep k1 fixed at 0.")
    parser.add_argument("--fix-k2", action="store_true", help="Keep k2 fixed at 0.")
    parser.add_argument("--fix-k3", action="store_true", help="Keep k3 fixed at 0.")
    parser.add_argument("--fix-principal-point", action="store_true", help="Keep principal point fixed when using an intrinsic guess.")
    parser.add_argument("--fix-aspect-ratio", action="store_true", help="Keep fx/fy aspect ratio fixed from the initial guess.")
    parser.add_argument(
        "--robust",
        "--ransac",
        action="store_true",
        dest="robust",
        help="Select the best observation files with a RANSAC-style consensus before final calibration.",
    )
    parser.add_argument("--ransac-iterations", type=int, default=120, help="Number of random robust-selection trials.")
    parser.add_argument("--ransac-sample-size", type=int, default=10, help="Observation count per random trial.")
    parser.add_argument("--ransac-threshold-px", type=float, default=0.5, help="Per-view RMS threshold for inlier files.")
    parser.add_argument("--ransac-refine-iterations", type=int, default=3, help="Refinement passes after the best consensus.")
    parser.add_argument("--ransac-seed", type=int, default=7, help="Random seed for robust selection.")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    observations = collect_observations(args)
    image_size = validate_observations(observations, args.min_views)
    robust_info: Dict[str, object] = {"enabled": False}

    if args.robust:
        observations, robust_info = select_robust_observations(
            observations,
            image_size,
            args,
        )
        image_size = validate_observations(observations, args.min_views)
        print(
            "[robust] selected "
            f"{robust_info.get('selected_count', len(observations))} observations, "
            f"excluded {robust_info.get('excluded_count', 0)}"
        )

    warnings, diversity = diversity_warnings(observations)

    flags = calibration_flags(args)
    result = run_calibration(
        observations,
        image_size,
        flags,
        args.use_intrinsic_guess,
    )
    (
        rms,
        camera_matrix,
        dist_coeffs,
        rvecs,
        tvecs,
        std_intrinsics,
        _std_extrinsics,
        per_view_errors,
    ) = result

    view_reports, mean_error, max_error = reprojection_errors(
        observations,
        camera_matrix,
        dist_coeffs,
        rvecs,
        tvecs,
    )

    write_opencv_xml(
        Path(args.output_xml),
        args.camera_name,
        image_size,
        camera_matrix,
        dist_coeffs,
        float(rms),
        args,
    )
    write_report_json(
        Path(args.output_json),
        observations,
        image_size,
        camera_matrix,
        dist_coeffs,
        float(rms),
        std_intrinsics,
        per_view_errors,
        view_reports,
        mean_error,
        max_error,
        diversity,
        warnings,
        robust_info,
        args,
    )

    print_summary(
        observations,
        camera_matrix,
        dist_coeffs,
        float(rms),
        mean_error,
        max_error,
        view_reports,
        warnings,
    )
    print(f"OpenCV XML written: {args.output_xml}")
    print(f"JSON report written: {args.output_json}")
    if len(observations) < 10:
        print("[warn] For a stable intrinsic calibration, capture more varied mire poses.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
