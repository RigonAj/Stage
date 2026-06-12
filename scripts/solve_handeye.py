#!/usr/bin/env python3
"""Offline eye-to-hand solver: camera -> robot base from multi-sample JSON.

Implements section 4 of docs/ur3e_camera_base_calibration.md (repo
6-Dof-Ur3e-Catch-a-ball): the DVXplorer camera is fixed in the room, the
phone mire is mounted on the UR3e flange (tool0). Per pose i the collector
stores T_base_tool0(i) (FK) and T_camera_mire(i) (solvePnP), both as
xyz (meters) + quat xyzw. The solver recovers:

    T_base_camera   (the target, frame parent 'base')
    T_tool0_mire    (co-solved, compared against the CAD value)

OpenCV convention trap (the whole point of --self-test): an OpenCV argument
named ``X2Y`` is the matrix mapping point coordinates from X to Y, i.e.
``T_y_x``. Both measured inputs are therefore INVERTED before
``cv2.calibrateRobotWorldHandEye`` and both outputs are read back DIRECTLY:

    R_world2cam[i]    = T_tool0_base(i)  = inv(T_base_tool0(i))
    R_base2gripper[i] = T_mire_camera(i) = inv(T_camera_mire(i))
    output base2world   -> T_base_camera
    output gripper2cam  -> T_tool0_mire

Cross-checked against ``cv2.calibrateHandEye`` with swapped base/gripper
roles. Run ``--self-test`` before trusting anything, including this header.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2 as cv
import numpy as np


# --------------------------------------------------------------------------
# SE(3) helpers (R: 3x3, t: shape (3,), both float64, meters/radians)
# --------------------------------------------------------------------------


def invert_rt(R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    Rt = R.T
    return Rt, -Rt @ t


def compose_rt(
    R_a: np.ndarray, t_a: np.ndarray, R_b: np.ndarray, t_b: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """T_a · T_b."""
    return R_a @ R_b, R_a @ t_b + t_a


def rotation_error_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle between two rotations, in degrees."""
    cos_angle = (np.trace(R_a.T @ R_b) - 1.0) * 0.5
    return math.degrees(math.acos(min(1.0, max(-1.0, cos_angle))))


def quat_xyzw_from_matrix(R: np.ndarray) -> np.ndarray:
    """Rotation matrix to quaternion [x, y, z, w] (Shepperd's method)."""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w], dtype=np.float64)
    return quat / np.linalg.norm(quat)


def matrix_from_quat_xyzw(quat: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rpy_from_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    """ROS extrinsic X-Y-Z roll/pitch/yaw (R = Rz·Ry·Rx), radians."""
    pitch = math.asin(min(1.0, max(-1.0, -float(R[2, 0]))))
    if abs(R[2, 0]) < 0.9999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def random_rotation(rng: np.random.Generator, min_angle_deg: float = 10.0) -> np.ndarray:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = math.radians(rng.uniform(min_angle_deg, 170.0))
    R, _ = cv.Rodrigues(axis * angle)
    return R


def perturb_rt(
    R: np.ndarray,
    t: np.ndarray,
    rng: np.random.Generator,
    trans_noise_m: float,
    rot_noise_deg: float,
) -> Tuple[np.ndarray, np.ndarray]:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    dR, _ = cv.Rodrigues(axis * math.radians(rot_noise_deg))
    return dR @ R, t + rng.normal(scale=trans_noise_m, size=3)


def rotation_axis_diversity_deg(R_list: Sequence[np.ndarray]) -> Optional[float]:
    """Largest pairwise angle between relative-rotation axes (degrees).

    Hand-eye only constrains rotation if the relative rotation axes between
    poses are not all parallel; below ~15 deg the problem is ill-conditioned.
    """
    axes: List[np.ndarray] = []
    for i in range(1, len(R_list)):
        rvec, _ = cv.Rodrigues(R_list[i - 1].T @ R_list[i])
        angle = float(np.linalg.norm(rvec))
        if angle > 1e-6:
            axes.append((rvec / angle).reshape(3))
    if len(axes) < 2:
        return None
    worst = 0.0
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            cos_angle = abs(float(np.dot(axes[i], axes[j])))
            worst = max(worst, math.degrees(math.acos(min(1.0, cos_angle))))
    return worst


# --------------------------------------------------------------------------
# Core solver (section 4 recipe)
# --------------------------------------------------------------------------


def solve_robot_world_handeye(
    R_base_tool0: Sequence[np.ndarray],
    t_base_tool0: Sequence[np.ndarray],
    R_camera_mire: Sequence[np.ndarray],
    t_camera_mire: Sequence[np.ndarray],
) -> Dict[str, np.ndarray]:
    """Both measured inputs are inverted; both outputs are read directly."""
    if len(R_base_tool0) != len(R_camera_mire) or len(R_base_tool0) < 3:
        raise ValueError(f"need >= 3 aligned pose pairs, got {len(R_base_tool0)}")

    R_w2c, t_w2c, R_b2g, t_b2g = [], [], [], []
    for R_bt, t_bt, R_cm, t_cm in zip(R_base_tool0, t_base_tool0, R_camera_mire, t_camera_mire):
        R, t = invert_rt(R_bt, t_bt)  # = T_tool0_base(i)
        R_w2c.append(R)
        t_w2c.append(t.reshape(3, 1))
        R, t = invert_rt(R_cm, t_cm)  # = T_mire_camera(i)
        R_b2g.append(R)
        t_b2g.append(t.reshape(3, 1))

    R_base_camera, t_base_camera, R_tool0_mire, t_tool0_mire = cv.calibrateRobotWorldHandEye(
        R_world2cam=R_w2c,
        t_world2cam=t_w2c,
        R_base2gripper=R_b2g,
        t_base2gripper=t_b2g,
        method=cv.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
    )

    # Cross-check with the classic eye-to-hand trick on calibrateHandEye:
    # swap base/gripper roles by feeding the inverted robot poses; the
    # "cam2gripper" output is then T_base_camera. solvePnP poses go in DIRECT.
    R_check, t_check = cv.calibrateHandEye(
        R_gripper2base=R_w2c,
        t_gripper2base=t_w2c,
        R_target2cam=list(R_camera_mire),
        t_target2cam=[t.reshape(3, 1) for t in t_camera_mire],
        method=cv.CALIB_HAND_EYE_PARK,
    )

    return {
        "R_base_camera": np.asarray(R_base_camera, dtype=np.float64),
        "t_base_camera": np.asarray(t_base_camera, dtype=np.float64).reshape(3),
        "R_tool0_mire": np.asarray(R_tool0_mire, dtype=np.float64),
        "t_tool0_mire": np.asarray(t_tool0_mire, dtype=np.float64).reshape(3),
        "R_base_camera_check": np.asarray(R_check, dtype=np.float64),
        "t_base_camera_check": np.asarray(t_check, dtype=np.float64).reshape(3),
    }


def per_pose_residuals(
    solution: Dict[str, np.ndarray],
    R_base_tool0: Sequence[np.ndarray],
    t_base_tool0: Sequence[np.ndarray],
    R_camera_mire: Sequence[np.ndarray],
    t_camera_mire: Sequence[np.ndarray],
) -> List[Dict[str, float]]:
    """Predict T_camera_mire(i) from the solution and compare to solvePnP."""
    R_cb, t_cb = invert_rt(solution["R_base_camera"], solution["t_base_camera"])
    residuals: List[Dict[str, float]] = []
    for i in range(len(R_base_tool0)):
        R_pred, t_pred = compose_rt(R_cb, t_cb, R_base_tool0[i], t_base_tool0[i])
        R_pred, t_pred = compose_rt(R_pred, t_pred, solution["R_tool0_mire"], solution["t_tool0_mire"])
        residuals.append(
            {
                "index": i,
                "trans_mm": float(np.linalg.norm(t_pred - t_camera_mire[i]) * 1000.0),
                "rot_deg": rotation_error_deg(R_pred, R_camera_mire[i]),
            }
        )
    return residuals


def leave_one_out(
    R_base_tool0: Sequence[np.ndarray],
    t_base_tool0: Sequence[np.ndarray],
    R_camera_mire: Sequence[np.ndarray],
    t_camera_mire: Sequence[np.ndarray],
    t_full: np.ndarray,
    R_full: np.ndarray,
) -> Dict[str, float]:
    """Stability of T_base_camera when dropping one pose at a time."""
    worst_trans_mm = 0.0
    worst_rot_deg = 0.0
    n = len(R_base_tool0)
    for skip in range(n):
        keep = [i for i in range(n) if i != skip]
        partial = solve_robot_world_handeye(
            [R_base_tool0[i] for i in keep],
            [t_base_tool0[i] for i in keep],
            [R_camera_mire[i] for i in keep],
            [t_camera_mire[i] for i in keep],
        )
        worst_trans_mm = max(
            worst_trans_mm, float(np.linalg.norm(partial["t_base_camera"] - t_full) * 1000.0)
        )
        worst_rot_deg = max(worst_rot_deg, rotation_error_deg(partial["R_base_camera"], R_full))
    return {"worst_trans_mm": worst_trans_mm, "worst_rot_deg": worst_rot_deg}


def pixel_residuals(
    solution: Dict[str, np.ndarray],
    samples: Sequence[Dict[str, object]],
    R_base_tool0: Sequence[np.ndarray],
    t_base_tool0: Sequence[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Optional[Dict[str, object]]:
    """End-to-end pixel RMS (section 8): project the mire dots through the
    predicted pose chain and compare with the measured blob pixels. Needs
    per-sample 'matches' with object_mm and camera_px."""
    R_cb, t_cb = invert_rt(solution["R_base_camera"], solution["t_base_camera"])
    errors: List[float] = []
    per_sample: List[Dict[str, float]] = []
    for i, sample in enumerate(samples):
        matches = sample.get("matches")
        if not isinstance(matches, list) or not matches:
            continue
        object_mm = np.array(
            [[m["object_mm"]["x"], m["object_mm"]["y"], m["object_mm"].get("z", 0.0)] for m in matches],
            dtype=np.float64,
        )
        image_px = np.array(
            [[m["camera_px"]["x"], m["camera_px"]["y"]] for m in matches], dtype=np.float64
        )
        R_pred, t_pred = compose_rt(R_cb, t_cb, R_base_tool0[i], t_base_tool0[i])
        R_pred, t_pred = compose_rt(R_pred, t_pred, solution["R_tool0_mire"], solution["t_tool0_mire"])
        rvec, _ = cv.Rodrigues(R_pred)
        projected, _ = cv.projectPoints(
            object_mm, rvec, t_pred.reshape(3, 1) * 1000.0, camera_matrix, dist_coeffs
        )
        sample_errors = np.linalg.norm(projected.reshape(-1, 2) - image_px, axis=1)
        errors.extend(sample_errors.tolist())
        per_sample.append(
            {"index": i, "rms_px": float(np.sqrt(np.mean(sample_errors**2)))}
        )
    if not errors:
        return None
    arr = np.asarray(errors)
    return {
        "point_count": len(errors),
        "rms_px": float(np.sqrt(np.mean(arr**2))),
        "max_px": float(np.max(arr)),
        "per_sample": per_sample,
    }


# --------------------------------------------------------------------------
# Multi-sample JSON I/O (section 5 schema)
# --------------------------------------------------------------------------


def load_samples(
    path: Path,
) -> Tuple[Dict[str, object], List[Dict[str, object]], List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    units = payload.get("units")
    if units != "meters":
        raise ValueError(f"{path}: units must be 'meters', got {units!r}")
    samples = payload.get("samples", [])
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"{path}: no samples")

    R_bt, t_bt, R_cm, t_cm = [], [], [], []
    for i, sample in enumerate(samples):
        for key in ("T_base_tool0", "T_camera_mire"):
            if key not in sample:
                raise ValueError(f"{path}: sample {i} missing {key}")
        T_bt = sample["T_base_tool0"]
        T_cm = sample["T_camera_mire"]
        R_bt.append(matrix_from_quat_xyzw(T_bt["quat_xyzw"]))
        t_bt.append(np.asarray(T_bt["xyz"], dtype=np.float64))
        R_cm.append(matrix_from_quat_xyzw(T_cm["quat_xyzw"]))
        t_cm.append(np.asarray(T_cm["xyz"], dtype=np.float64))
    return payload, samples, R_bt, t_bt, R_cm, t_cm


def transform_to_json(R: np.ndarray, t: np.ndarray, parent: str, child: str) -> Dict[str, object]:
    roll, pitch, yaw = rpy_from_matrix(R)
    return {
        "parent": parent,
        "child": child,
        "xyz": [float(v) for v in t],
        "quat_xyzw": [float(v) for v in quat_xyzw_from_matrix(R)],
        "rpy_rad": [roll, pitch, yaw],
        "rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
    }


def write_result_yaml(path: Path, report: Dict[str, object]) -> None:
    """Hand-rolled YAML so the solver stays numpy+cv2 only."""

    def is_scalar_list(value: object) -> bool:
        return isinstance(value, list) and all(
            isinstance(v, (int, float, str, bool)) or v is None for v in value
        )

    def emit(value: object, indent: int) -> List[str]:
        pad = "  " * indent
        lines: List[str] = []
        if isinstance(value, dict):
            for key, item in value.items():
                if is_scalar_list(item) or not isinstance(item, (dict, list)) or not item:
                    lines.append(f"{pad}{key}: {json.dumps(item)}")
                else:
                    lines.append(f"{pad}{key}:")
                    lines.extend(emit(item, indent + 1))
        elif isinstance(value, list):
            for item in value:
                if isinstance(item, dict) and item:
                    lines.append(f"{pad}-")
                    lines.extend(emit(item, indent + 1))
                else:
                    lines.append(f"{pad}- {json.dumps(item)}")
        else:
            lines.append(f"{pad}{json.dumps(value)}")
        return lines

    lines = ["# Hand-eye calibration result (eye-to-hand, UR3e + DVXplorer)"]
    lines.extend(emit(report, 0))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# --------------------------------------------------------------------------
# Report / CLI
# --------------------------------------------------------------------------


def build_report(
    payload: Dict[str, object],
    samples: List[Dict[str, object]],
    R_bt: List[np.ndarray],
    t_bt: List[np.ndarray],
    R_cm: List[np.ndarray],
    t_cm: List[np.ndarray],
    samples_path: Optional[Path],
    intrinsics_path: Optional[Path],
) -> Dict[str, object]:
    solution = solve_robot_world_handeye(R_bt, t_bt, R_cm, t_cm)
    frames = payload.get("frames", {}) if isinstance(payload.get("frames"), dict) else {}
    base_frame = str(frames.get("robot_parent", "base"))
    tool_frame = str(frames.get("robot_child", "tool0"))
    camera_frame = str(frames.get("camera", "camera_optical"))
    mire_frame = str(frames.get("mire", "screen_center"))

    residuals = per_pose_residuals(solution, R_bt, t_bt, R_cm, t_cm)
    trans_residuals = [r["trans_mm"] for r in residuals]
    rot_residuals = [r["rot_deg"] for r in residuals]

    agreement_trans_mm = float(
        np.linalg.norm(solution["t_base_camera"] - solution["t_base_camera_check"]) * 1000.0
    )
    agreement_rot_deg = rotation_error_deg(
        solution["R_base_camera"], solution["R_base_camera_check"]
    )

    loo = (
        leave_one_out(R_bt, t_bt, R_cm, t_cm, solution["t_base_camera"], solution["R_base_camera"])
        if len(R_bt) >= 4
        else None
    )
    diversity = rotation_axis_diversity_deg(R_bt)

    # Screen normal sanity (section 1): mire Z goes INTO the screen, away
    # from the camera, so in camera frame its Z component must be positive.
    mire_z_in_camera = [float(R[2, 2]) for R in R_cm]
    normal_ok = all(v > 0.0 for v in mire_z_in_camera)

    report: Dict[str, object] = {
        "created_at": datetime.now().isoformat(timespec="milliseconds"),
        "units": "meters",
        "solver": "cv2.calibrateRobotWorldHandEye(CALIB_ROBOT_WORLD_HAND_EYE_SHAH)",
        "cross_check_solver": "cv2.calibrateHandEye(CALIB_HAND_EYE_PARK), swapped roles",
        "samples_file": str(samples_path) if samples_path else None,
        "sample_count": len(R_bt),
        "T_base_camera": transform_to_json(
            solution["R_base_camera"], solution["t_base_camera"], base_frame, camera_frame
        ),
        "T_tool0_mire": transform_to_json(
            solution["R_tool0_mire"], solution["t_tool0_mire"], tool_frame, mire_frame
        ),
        "validation": {
            "solver_agreement": {
                "trans_mm": agreement_trans_mm,
                "rot_deg": agreement_rot_deg,
            },
            "pose_residuals": {
                "trans_mean_mm": float(np.mean(trans_residuals)),
                "trans_max_mm": float(np.max(trans_residuals)),
                "rot_mean_deg": float(np.mean(rot_residuals)),
                "rot_max_deg": float(np.max(rot_residuals)),
                "per_pose": residuals,
            },
            "leave_one_out": loo,
            "rotation_axis_diversity_deg": diversity,
            "mire_normal_into_screen_ok": normal_ok,
        },
    }

    if intrinsics_path is not None:
        camera_matrix, dist_coeffs = load_intrinsics_xml(intrinsics_path)
        pixels = pixel_residuals(solution, samples, R_bt, t_bt, camera_matrix, dist_coeffs)
        report["validation"]["pixel_residuals"] = pixels
        report["intrinsics_xml"] = str(intrinsics_path)
    return report


def load_intrinsics_xml(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    fs = cv.FileStorage(str(path), cv.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"cannot open intrinsics file: {path}")
    try:
        root = fs.root()
        nodes = [root] + [root.getNode(k) for k in root.keys() if root.getNode(k).isMap()]
        for node in nodes:
            matrix_node = node.getNode("camera_matrix")
            dist_node = node.getNode("distortion_coefficients")
            if matrix_node.empty() or dist_node.empty():
                continue
            camera_matrix = np.asarray(matrix_node.mat(), dtype=np.float64)
            dist_coeffs = np.asarray(dist_node.mat(), dtype=np.float64).reshape(-1)
            if camera_matrix.shape == (3, 3):
                return camera_matrix, dist_coeffs
    finally:
        fs.release()
    raise RuntimeError(f"no camera_matrix/distortion_coefficients in {path}")


def print_report(report: Dict[str, object]) -> None:
    tbc = report["T_base_camera"]
    ttm = report["T_tool0_mire"]
    val = report["validation"]
    print(f"samples: {report['sample_count']}")
    print(f"T_{tbc['parent']}_{tbc['child']}:")
    print(f"  xyz [m]   : {['%.5f' % v for v in tbc['xyz']]}")
    print(f"  quat xyzw : {['%.6f' % v for v in tbc['quat_xyzw']]}")
    print(f"  rpy [deg] : {['%.3f' % v for v in tbc['rpy_deg']]}")
    print(f"T_{ttm['parent']}_{ttm['child']}:")
    print(f"  xyz [m]   : {['%.5f' % v for v in ttm['xyz']]}")
    print(f"  rpy [deg] : {['%.3f' % v for v in ttm['rpy_deg']]}")
    agreement = val["solver_agreement"]
    print(
        f"solver agreement: {agreement['trans_mm']:.3f} mm / {agreement['rot_deg']:.4f} deg"
    )
    residuals = val["pose_residuals"]
    print(
        f"pose residuals: trans mean {residuals['trans_mean_mm']:.2f} mm "
        f"max {residuals['trans_max_mm']:.2f} mm | "
        f"rot mean {residuals['rot_mean_deg']:.3f} deg max {residuals['rot_max_deg']:.3f} deg"
    )
    if val.get("leave_one_out"):
        loo = val["leave_one_out"]
        print(
            f"leave-one-out: worst {loo['worst_trans_mm']:.2f} mm / "
            f"{loo['worst_rot_deg']:.3f} deg"
        )
    if val.get("rotation_axis_diversity_deg") is not None:
        diversity = val["rotation_axis_diversity_deg"]
        flag = "" if diversity >= 15.0 else "  <-- POSES TOO PARALLEL, rotation ill-constrained"
        print(f"rotation axis diversity: {diversity:.1f} deg{flag}")
    print(f"mire normal into screen: {'ok' if val['mire_normal_into_screen_ok'] else 'WRONG SIDE'}")
    if val.get("pixel_residuals"):
        pixels = val["pixel_residuals"]
        print(
            f"pixel residuals: rms {pixels['rms_px']:.2f} px max {pixels['max_px']:.2f} px "
            f"({pixels['point_count']} points)"
        )
    quat = tbc["quat_xyzw"]
    xyz = tbc["xyz"]
    print(
        "static TF publisher:\n"
        f"  ros2 run tf2_ros static_transform_publisher "
        f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f} "
        f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f} "
        f"{tbc['parent']} {tbc['child']}"
    )


# --------------------------------------------------------------------------
# Self-test (section 4: mandatory before any real data)
# --------------------------------------------------------------------------


def make_synthetic_problem(
    rng: np.random.Generator, n_poses: int
) -> Dict[str, object]:
    R_bc = random_rotation(rng)
    t_bc = rng.uniform(-1.5, 1.5, size=3)
    R_tm = random_rotation(rng)
    t_tm = rng.uniform(-0.10, 0.10, size=3)

    R_cb, t_cb = invert_rt(R_bc, t_bc)
    R_bt_list, t_bt_list, R_cm_list, t_cm_list = [], [], [], []
    for _ in range(n_poses):
        R_bt = random_rotation(rng)
        t_bt = rng.uniform(-0.5, 0.5, size=3) + np.array([0.0, 0.0, 0.3])
        # T_camera_mire(i) = inv(T_base_camera) · T_base_tool0(i) · T_tool0_mire
        R_cm, t_cm = compose_rt(R_cb, t_cb, R_bt, t_bt)
        R_cm, t_cm = compose_rt(R_cm, t_cm, R_tm, t_tm)
        R_bt_list.append(R_bt)
        t_bt_list.append(t_bt)
        R_cm_list.append(R_cm)
        t_cm_list.append(t_cm)
    return {
        "R_bc": R_bc,
        "t_bc": t_bc,
        "R_tm": R_tm,
        "t_tm": t_tm,
        "R_bt": R_bt_list,
        "t_bt": t_bt_list,
        "R_cm": R_cm_list,
        "t_cm": t_cm_list,
    }


def synthetic_samples_json(problem: Dict[str, object], path: Path) -> None:
    """Write the synthetic problem using the exact section 5 schema."""
    samples = []
    for i in range(len(problem["R_bt"])):
        R_cm = problem["R_cm"][i]
        t_cm = problem["t_cm"][i]
        rvec, _ = cv.Rodrigues(R_cm)
        samples.append(
            {
                "index": i,
                "stamp": datetime.now().isoformat(timespec="milliseconds"),
                "T_base_tool0": {
                    "xyz": [float(v) for v in problem["t_bt"][i]],
                    "quat_xyzw": [float(v) for v in quat_xyzw_from_matrix(problem["R_bt"][i])],
                },
                "T_camera_mire": {
                    "xyz": [float(v) for v in t_cm],
                    "quat_xyzw": [float(v) for v in quat_xyzw_from_matrix(R_cm)],
                    "rvec": [float(v) for v in rvec.reshape(3)],
                    "tvec_mm": [float(v * 1000.0) for v in t_cm],
                },
                "joint_positions_rad": [0.0] * 6,
                "stationarity": {"trans_delta_mm": 0.0, "rot_delta_deg": 0.0},
                "reproj_rms_px": 0.0,
                "matched_dots": 19,
                "tilt_deg": 0.0,
                "ippe_ambiguity_ratio": 99.0,
            }
        )
    payload = {
        "created_at": datetime.now().isoformat(timespec="milliseconds"),
        "units": "meters",
        "frames": {
            "robot_parent": "base",
            "robot_child": "tool0",
            "camera": "camera_optical",
            "mire": "screen_center",
        },
        "intrinsics_xml": None,
        "samples": samples,
    }
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)


def run_self_test() -> int:
    failures = 0

    def check(name: str, ok: bool, detail: str = "") -> None:
        nonlocal failures
        print(f"{'ok ' if ok else 'FAIL'} {name}{(' | ' + detail) if detail else ''}")
        if not ok:
            failures += 1

    rng = np.random.default_rng(20260612)
    problem = make_synthetic_problem(rng, n_poses=15)

    diversity = rotation_axis_diversity_deg(problem["R_bt"])
    check("pose set has non-parallel rotation axes", diversity is not None and diversity > 30.0,
          f"diversity {diversity:.1f} deg")

    # 1. Noise-free: exact recovery, direct outputs.
    solution = solve_robot_world_handeye(
        problem["R_bt"], problem["t_bt"], problem["R_cm"], problem["t_cm"]
    )
    err_bc_t = float(np.linalg.norm(solution["t_base_camera"] - problem["t_bc"]))
    err_bc_r = rotation_error_deg(solution["R_base_camera"], problem["R_bc"])
    err_tm_t = float(np.linalg.norm(solution["t_tool0_mire"] - problem["t_tm"]))
    err_tm_r = rotation_error_deg(solution["R_tool0_mire"], problem["R_tm"])
    check(
        "noise-free T_base_camera exact",
        err_bc_t < 1e-6 and err_bc_r < 1e-5,
        f"trans {err_bc_t:.2e} m rot {err_bc_r:.2e} deg",
    )
    check(
        "noise-free T_tool0_mire exact",
        err_tm_t < 1e-6 and err_tm_r < 1e-5,
        f"trans {err_tm_t:.2e} m rot {err_tm_r:.2e} deg",
    )

    # 2. Both OpenCV solvers coincide on clean data.
    agree_t = float(
        np.linalg.norm(solution["t_base_camera"] - solution["t_base_camera_check"])
    )
    agree_r = rotation_error_deg(solution["R_base_camera"], solution["R_base_camera_check"])
    check(
        "calibrateRobotWorldHandEye == calibrateHandEye (clean)",
        agree_t < 1e-6 and agree_r < 1e-5,
        f"trans {agree_t:.2e} m rot {agree_r:.2e} deg",
    )

    # 3. Naive (non-inverted) inputs must NOT recover the ground truth:
    # guards against silently "fixing" the conventions later.
    naive = cv.calibrateRobotWorldHandEye(
        R_world2cam=list(problem["R_bt"]),
        t_world2cam=[t.reshape(3, 1) for t in problem["t_bt"]],
        R_base2gripper=list(problem["R_cm"]),
        t_base2gripper=[t.reshape(3, 1) for t in problem["t_cm"]],
        method=cv.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
    )
    naive_err = float(np.linalg.norm(np.asarray(naive[1]).reshape(3) - problem["t_bc"]))
    check(
        "naive (non-inverted) inputs give a wrong result",
        naive_err > 0.01,
        f"naive trans error {naive_err * 1000.0:.1f} mm",
    )

    # 4. Noise 0.1 mm / 0.05 deg on both measured inputs: error stays at
    # the noise level.
    noisy_R_bt, noisy_t_bt, noisy_R_cm, noisy_t_cm = [], [], [], []
    for i in range(len(problem["R_bt"])):
        R, t = perturb_rt(problem["R_bt"][i], problem["t_bt"][i], rng, 0.0001, 0.05)
        noisy_R_bt.append(R)
        noisy_t_bt.append(t)
        R, t = perturb_rt(problem["R_cm"][i], problem["t_cm"][i], rng, 0.0001, 0.05)
        noisy_R_cm.append(R)
        noisy_t_cm.append(t)
    noisy = solve_robot_world_handeye(noisy_R_bt, noisy_t_bt, noisy_R_cm, noisy_t_cm)
    noisy_err_t = float(np.linalg.norm(noisy["t_base_camera"] - problem["t_bc"]) * 1000.0)
    noisy_err_r = rotation_error_deg(noisy["R_base_camera"], problem["R_bc"])
    check(
        "noisy recovery stays at noise level",
        noisy_err_t < 5.0 and noisy_err_r < 0.25,
        f"trans {noisy_err_t:.3f} mm rot {noisy_err_r:.4f} deg",
    )
    noisy_agree_t = float(
        np.linalg.norm(noisy["t_base_camera"] - noisy["t_base_camera_check"]) * 1000.0
    )
    noisy_agree_r = rotation_error_deg(noisy["R_base_camera"], noisy["R_base_camera_check"])
    check(
        "solvers agree on noisy data (mm / tenths of deg)",
        noisy_agree_t < 10.0 and noisy_agree_r < 0.5,
        f"trans {noisy_agree_t:.3f} mm rot {noisy_agree_r:.4f} deg",
    )

    # 5. JSON round-trip through the section 5 schema + full report path.
    with tempfile.TemporaryDirectory() as tmp_dir:
        json_path = Path(tmp_dir) / "synthetic_samples.json"
        synthetic_samples_json(problem, json_path)
        payload, samples, R_bt, t_bt, R_cm, t_cm = load_samples(json_path)
        roundtrip = solve_robot_world_handeye(R_bt, t_bt, R_cm, t_cm)
        rt_err = float(np.linalg.norm(roundtrip["t_base_camera"] - problem["t_bc"]))
        check("JSON schema round-trip recovers ground truth", rt_err < 1e-6,
              f"trans {rt_err:.2e} m")

        report = build_report(payload, samples, R_bt, t_bt, R_cm, t_cm, json_path, None)
        residuals = report["validation"]["pose_residuals"]
        check(
            "per-pose residuals ~0 on clean data",
            residuals["trans_max_mm"] < 1e-3 and residuals["rot_max_deg"] < 1e-4,
            f"max {residuals['trans_max_mm']:.2e} mm",
        )
        loo = report["validation"]["leave_one_out"]
        check("leave-one-out stable on clean data", loo["worst_trans_mm"] < 1e-3,
              f"worst {loo['worst_trans_mm']:.2e} mm")
        yaml_path = Path(tmp_dir) / "result.yaml"
        write_result_yaml(yaml_path, report)
        check("result YAML written", yaml_path.is_file() and "T_base_camera" in yaml_path.read_text())

    # 6. Quaternion helpers are mutually consistent.
    for _ in range(20):
        R = random_rotation(rng, min_angle_deg=0.5)
        R_back = matrix_from_quat_xyzw(quat_xyzw_from_matrix(R))
        if rotation_error_deg(R, R_back) > 1e-6:
            check("quaternion round-trip", False)
            break
    else:
        check("quaternion round-trip", True)

    print("self-test " + ("ok" if failures == 0 else f"FAILED ({failures})"))
    return 0 if failures == 0 else 1


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples", nargs="?", help="Multi-sample JSON (section 5 schema).")
    parser.add_argument("--intrinsics", help="OpenCV intrinsics XML for pixel residuals "
                        "(default: intrinsics_xml field of the samples file).")
    parser.add_argument("--output-yaml", help="Write the result transforms + validation as YAML.")
    parser.add_argument("--output-json", help="Write the full report as JSON.")
    parser.add_argument("--self-test", action="store_true",
                        help="Run the synthetic convention test and exit.")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return run_self_test()
    if not args.samples:
        print("error: samples JSON required (or --self-test)", file=sys.stderr)
        return 2

    samples_path = Path(args.samples)
    payload, samples, R_bt, t_bt, R_cm, t_cm = load_samples(samples_path)

    intrinsics_path: Optional[Path] = None
    if args.intrinsics:
        intrinsics_path = Path(args.intrinsics)
    elif payload.get("intrinsics_xml"):
        candidate = Path(str(payload["intrinsics_xml"]))
        if candidate.is_file():
            intrinsics_path = candidate

    report = build_report(
        payload, samples, R_bt, t_bt, R_cm, t_cm, samples_path, intrinsics_path
    )
    print_report(report)

    if args.output_yaml:
        write_result_yaml(Path(args.output_yaml), report)
        print(f"YAML result: {args.output_yaml}")
    if args.output_json:
        with Path(args.output_json).open("w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
        print(f"JSON report: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
