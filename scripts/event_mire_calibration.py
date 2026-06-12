#!/usr/bin/env python3
"""Interactive blinking target for DVXplorer event-camera calibration."""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2 as cv
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


ROWS = 4
COLS = 5
MISSING_DOT = (1, 2)
ANCHOR_DOT = (0, 0)
EXPECTED_DOTS = ROWS * COLS - 1
SQUARE_EXPECTED_DOTS = 4
SQUARE_SEQUENCE = [
    {"id": "center_large", "label": "centre grand", "offset_x": 0.0, "offset_y": 0.0, "side_scale": 2.0},
    {"id": "upper_left_medium", "label": "haut gauche moyen", "offset_x": -0.65, "offset_y": -0.45, "side_scale": 1.35},
    {"id": "upper_right_small", "label": "haut droite petit", "offset_x": 0.75, "offset_y": -0.25, "side_scale": 1.10},
    {"id": "lower_center_medium", "label": "bas centre moyen", "offset_x": 0.20, "offset_y": 0.65, "side_scale": 1.55},
]


@dataclass
class MonitorInfo:
    name: str
    x: int
    y: int
    width_px: int
    height_px: int
    width_mm: float
    height_mm: float
    dpi_x: float
    dpi_y: float
    device_pixel_ratio: float
    source: str

    @property
    def valid_size_mm(self) -> bool:
        return self.width_mm > 0.0 and self.height_mm > 0.0

    @property
    def mm_per_px_x(self) -> float:
        return self.width_mm / self.width_px if self.width_px > 0 else 0.0

    @property
    def mm_per_px_y(self) -> float:
        return self.height_mm / self.height_px if self.height_px > 0 else 0.0

    def label(self) -> str:
        width_cm = self.width_mm / 10.0
        height_cm = self.height_mm / 10.0
        return (
            f"{self.name} - {self.width_px}x{self.height_px} "
            f"- {width_cm:.1f} x {height_cm:.1f} cm "
            f"- {self.mm_per_px_x:.3f} x {self.mm_per_px_y:.3f} mm/px"
        )

    def to_json(self) -> Dict[str, object]:
        return {
            "name": self.name,
            "source": self.source,
            "geometry_px": {
                "x": self.x,
                "y": self.y,
                "width": self.width_px,
                "height": self.height_px,
            },
            "size_mm": {"width": self.width_mm, "height": self.height_mm},
            "size_cm": {"width": self.width_mm / 10.0, "height": self.height_mm / 10.0},
            "mm_per_px": {"x": self.mm_per_px_x, "y": self.mm_per_px_y},
            "dpi": {"x": self.dpi_x, "y": self.dpi_y},
            "device_pixel_ratio": self.device_pixel_ratio,
        }


@dataclass
class ScreenDot:
    row: int
    col: int
    anchor: bool
    screen_x_px: float
    screen_y_px: float
    radius_px: float
    object_x_mm: float
    object_y_mm: float
    object_z_mm: float = 0.0
    label: Optional[str] = None

    @property
    def dot_id(self) -> str:
        if self.label is not None:
            return self.label
        return f"r{self.row}_c{self.col}"

    def to_json(self) -> Dict[str, object]:
        return {
            "id": self.dot_id,
            "row": self.row,
            "col": self.col,
            "anchor": self.anchor,
            "screen_px": {"x": self.screen_x_px, "y": self.screen_y_px},
            "radius_px": self.radius_px,
            "object_mm": {
                "x": self.object_x_mm,
                "y": self.object_y_mm,
                "z": self.object_z_mm,
            },
        }


@dataclass
class Blob:
    index: int
    x: float
    y: float
    area_px: int
    weight: float
    peak: float
    bbox: Tuple[int, int, int, int]

    def to_json(self) -> Dict[str, object]:
        return {
            "index": self.index,
            "center_px": {"x": self.x, "y": self.y},
            "area_px": self.area_px,
            "activity_sum": self.weight,
            "peak": self.peak,
            "bbox": {
                "x": self.bbox[0],
                "y": self.bbox[1],
                "width": self.bbox[2],
                "height": self.bbox[3],
            },
        }


@dataclass
class Match:
    dot: ScreenDot
    blob: Blob
    reproj_error_px: float

    def to_json(self) -> Dict[str, object]:
        return {
            "dot_id": self.dot.dot_id,
            "row": self.dot.row,
            "col": self.dot.col,
            "anchor": self.dot.anchor,
            "camera_px": {"x": self.blob.x, "y": self.blob.y},
            "object_mm": {
                "x": self.dot.object_x_mm,
                "y": self.dot.object_y_mm,
                "z": self.dot.object_z_mm,
            },
            "screen_px": {
                "x": self.dot.screen_x_px,
                "y": self.dot.screen_y_px,
            },
            "blob_area_px": self.blob.area_px,
            "activity_sum": self.blob.weight,
            "reprojection_error_px": self.reproj_error_px,
        }


def parse_xrandr_monitors() -> Dict[str, MonitorInfo]:
    """Read monitor geometry and physical size from xrandr when available."""
    try:
        proc = subprocess.run(
            ["xrandr", "--query"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return {}

    monitors: Dict[str, MonitorInfo] = {}
    pattern = re.compile(
        r"^(?P<name>\S+) connected(?: primary)? "
        r"(?P<w>\d+)x(?P<h>\d+)\+(?P<x>-?\d+)\+(?P<y>-?\d+)"
        r".*?\s(?P<mmw>\d+)mm x (?P<mmh>\d+)mm"
    )
    for line in proc.stdout.splitlines():
        match = pattern.match(line.strip())
        if not match:
            continue
        width_px = int(match.group("w"))
        height_px = int(match.group("h"))
        width_mm = float(match.group("mmw"))
        height_mm = float(match.group("mmh"))
        dpi_x = width_px / (width_mm / 25.4) if width_mm > 0 else 0.0
        dpi_y = height_px / (height_mm / 25.4) if height_mm > 0 else 0.0
        name = match.group("name")
        monitors[name] = MonitorInfo(
            name=name,
            x=int(match.group("x")),
            y=int(match.group("y")),
            width_px=width_px,
            height_px=height_px,
            width_mm=width_mm,
            height_mm=height_mm,
            dpi_x=dpi_x,
            dpi_y=dpi_y,
            device_pixel_ratio=1.0,
            source="xrandr",
        )
    return monitors


def detect_monitors(app: QtGui.QGuiApplication) -> List[MonitorInfo]:
    """Detect monitors with Qt, using xrandr values when they are more complete."""
    xrandr = parse_xrandr_monitors()
    monitors: List[MonitorInfo] = []

    for index, screen in enumerate(app.screens()):
        geometry = screen.geometry()
        name = screen.name() or f"screen-{index}"
        size = screen.physicalSize()
        dpr = float(screen.devicePixelRatio())
        qt_info = MonitorInfo(
            name=name,
            x=geometry.x(),
            y=geometry.y(),
            width_px=geometry.width(),
            height_px=geometry.height(),
            width_mm=float(size.width()),
            height_mm=float(size.height()),
            dpi_x=float(screen.physicalDotsPerInchX()),
            dpi_y=float(screen.physicalDotsPerInchY()),
            device_pixel_ratio=dpr,
            source="qt",
        )

        xr_info = xrandr.get(name)
        if xr_info is not None and xr_info.valid_size_mm:
            monitors.append(replace(xr_info, device_pixel_ratio=dpr, source="qt+xrandr"))
        else:
            monitors.append(qt_info)

    if not monitors:
        monitors = list(xrandr.values())

    return monitors


def apply_size_override(
    monitor: MonitorInfo,
    width_mm: Optional[float],
    height_mm: Optional[float],
) -> MonitorInfo:
    if width_mm is None and height_mm is None:
        return monitor
    return replace(
        monitor,
        width_mm=float(width_mm) if width_mm is not None else monitor.width_mm,
        height_mm=float(height_mm) if height_mm is not None else monitor.height_mm,
        source=f"{monitor.source}+manual-size",
    )


def select_monitor(monitors: Sequence[MonitorInfo], requested: Optional[str]) -> int:
    if not monitors:
        return -1
    if not requested:
        return 0
    if requested.isdigit():
        idx = int(requested)
        if 0 <= idx < len(monitors):
            return idx
    for idx, monitor in enumerate(monitors):
        if monitor.name == requested:
            return idx
    requested_lower = requested.lower()
    for idx, monitor in enumerate(monitors):
        if requested_lower in monitor.name.lower():
            return idx
    return 0


def build_mire_layout(
    width_px: int,
    height_px: int,
    mm_per_px_x: float,
    mm_per_px_y: float,
) -> Tuple[List[ScreenDot], Dict[str, float]]:
    spacing_px = 0.82 * min(width_px / float(COLS - 1), height_px / float(ROWS - 1))
    center_x = width_px * 0.5
    center_y = height_px * 0.5
    small_radius = spacing_px * 0.17
    anchor_radius = spacing_px * 0.285

    dots: List[ScreenDot] = []
    for row in range(ROWS):
        for col in range(COLS):
            if (row, col) == MISSING_DOT:
                continue
            x = center_x + (col - (COLS - 1) * 0.5) * spacing_px
            y = center_y + (row - (ROWS - 1) * 0.5) * spacing_px
            anchor = (row, col) == ANCHOR_DOT
            radius = anchor_radius if anchor else small_radius
            dots.append(
                ScreenDot(
                    row=row,
                    col=col,
                    anchor=anchor,
                    screen_x_px=x,
                    screen_y_px=y,
                    radius_px=radius,
                    object_x_mm=(x - center_x) * mm_per_px_x,
                    object_y_mm=(y - center_y) * mm_per_px_y,
                )
            )

    meta = {
        "spacing_px": spacing_px,
        "spacing_x_mm": spacing_px * mm_per_px_x,
        "spacing_y_mm": spacing_px * mm_per_px_y,
        "center_x_px": center_x,
        "center_y_px": center_y,
        "small_radius_px": small_radius,
        "anchor_radius_px": anchor_radius,
    }
    return dots, meta


def build_square_layout(
    width_px: int,
    height_px: int,
    mm_per_px_x: float,
    mm_per_px_y: float,
    offset_x: float = 0.0,
    offset_y: float = 0.0,
    side_scale: float = 2.0,
    variant_id: str = "center_large",
    variant_label: str = "centre grand",
) -> Tuple[List[ScreenDot], Dict[str, float]]:
    _, mire_meta = build_mire_layout(width_px, height_px, mm_per_px_x, mm_per_px_y)
    screen_center_x = width_px * 0.5
    screen_center_y = height_px * 0.5
    center_dx_px = offset_x * mire_meta["spacing_px"]
    center_dy_px = offset_y * mire_meta["spacing_px"]
    center_x = screen_center_x + center_dx_px
    center_y = screen_center_y + center_dy_px
    side_px = side_scale * mire_meta["spacing_px"]
    half_side_px = side_px * 0.5
    radius_px = mire_meta["small_radius_px"]
    corners = [
        ("tl", 0, 0, -half_side_px, -half_side_px),
        ("tr", 0, 1, half_side_px, -half_side_px),
        ("bl", 1, 0, -half_side_px, half_side_px),
        ("br", 1, 1, half_side_px, half_side_px),
    ]

    dots = [
        ScreenDot(
            row=row,
            col=col,
            anchor=False,
            screen_x_px=center_x + dx,
            screen_y_px=center_y + dy,
            radius_px=radius_px,
            object_x_mm=(center_dx_px + dx) * mm_per_px_x,
            object_y_mm=(center_dy_px + dy) * mm_per_px_y,
            label=label,
        )
        for label, row, col, dx, dy in corners
    ]
    meta = {
        "pattern": "square4",
        "variant_id": variant_id,
        "variant_label": variant_label,
        "offset_x_spacing": offset_x,
        "offset_y_spacing": offset_y,
        "side_scale_spacing": side_scale,
        "side_px": side_px,
        "side_x_mm": side_px * mm_per_px_x,
        "side_y_mm": side_px * mm_per_px_y,
        "center_x_px": center_x,
        "center_y_px": center_y,
        "object_center_x_mm": center_dx_px * mm_per_px_x,
        "object_center_y_mm": center_dy_px * mm_per_px_y,
        "radius_px": radius_px,
        "base_mire_spacing_px": mire_meta["spacing_px"],
    }
    return dots, meta


def split_sorted_into_groups(values: np.ndarray, groups: int) -> Optional[List[np.ndarray]]:
    if len(values) < groups:
        return None
    order = np.argsort(values)
    sorted_values = values[order]
    gaps = np.diff(sorted_values)
    if len(gaps) < groups - 1:
        return None
    cut_positions = np.argsort(gaps)[-(groups - 1) :] + 1
    cut_positions = np.sort(cut_positions)
    group_indices = np.split(order, cut_positions)
    if len(group_indices) != groups or any(len(g) == 0 for g in group_indices):
        return None
    group_indices.sort(key=lambda idxs: float(np.mean(values[idxs])))
    return group_indices


def detect_blobs(activity: np.ndarray, expected: int = EXPECTED_DOTS) -> List[Blob]:
    if activity.size == 0 or float(np.max(activity)) <= 0.0:
        return []

    max_value = float(np.max(activity))
    normalized = np.clip(activity / max_value * 255.0, 0, 255).astype(np.uint8)
    blurred = cv.GaussianBlur(normalized, (5, 5), 0)

    nonzero = blurred[blurred > 0]
    if nonzero.size == 0:
        return []
    percentile_threshold = int(np.percentile(nonzero, 70))
    otsu_threshold, otsu = cv.threshold(blurred, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    threshold = max(4, min(int(otsu_threshold), percentile_threshold))
    _, binary = cv.threshold(blurred, threshold, 255, cv.THRESH_BINARY)
    if int(np.count_nonzero(binary)) < expected:
        binary = otsu

    kernel = np.ones((3, 3), dtype=np.uint8)
    binary = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel)
    binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, kernel)

    labels_count, labels, stats, centroids = cv.connectedComponentsWithStats(binary, 8)
    min_area = max(5, int(activity.shape[0] * activity.shape[1] * 0.00001))
    max_area = int(activity.shape[0] * activity.shape[1] * 0.2)
    blobs: List[Blob] = []

    for label in range(1, labels_count):
        area = int(stats[label, cv.CC_STAT_AREA])
        if area < min_area or area > max_area:
            continue
        x = int(stats[label, cv.CC_STAT_LEFT])
        y = int(stats[label, cv.CC_STAT_TOP])
        w = int(stats[label, cv.CC_STAT_WIDTH])
        h = int(stats[label, cv.CC_STAT_HEIGHT])
        mask = labels[y : y + h, x : x + w] == label
        weights = activity[y : y + h, x : x + w][mask]
        if weights.size == 0:
            continue
        yy, xx = np.nonzero(mask)
        total = float(np.sum(weights))
        if total > 0.0:
            cx = float(np.sum((xx + x) * weights) / total)
            cy = float(np.sum((yy + y) * weights) / total)
        else:
            cx = float(centroids[label][0])
            cy = float(centroids[label][1])
        blobs.append(
            Blob(
                index=len(blobs),
                x=cx,
                y=cy,
                area_px=area,
                weight=total,
                peak=float(np.max(weights)),
                bbox=(x, y, w, h),
            )
        )

    blobs.sort(key=lambda blob: blob.weight, reverse=True)
    if len(blobs) > expected:
        blobs = blobs[:expected]
    for idx, blob in enumerate(blobs):
        blob.index = idx
    return blobs


def associate_blobs_to_layout(blobs: Sequence[Blob], dots: Sequence[ScreenDot]) -> Tuple[List[Match], str]:
    if len(blobs) < EXPECTED_DOTS:
        return [], f"not enough blobs: {len(blobs)}/{EXPECTED_DOTS}"

    selected = list(blobs[:EXPECTED_DOTS])
    points = np.array([[blob.x, blob.y] for blob in selected], dtype=np.float64)
    mean = np.mean(points, axis=0)
    centered = points - mean
    covariance = centered.T @ centered / max(1, len(points) - 1)
    eigvals, eigvecs = np.linalg.eigh(covariance)
    order = np.argsort(eigvals)[::-1]
    axes = eigvecs[:, order].T

    projections = centered @ axes.T
    range0 = float(np.ptp(projections[:, 0]))
    range1 = float(np.ptp(projections[:, 1]))
    if range1 > range0:
        axes = axes[[1, 0], :]
        projections = centered @ axes.T

    anchor_idx = int(np.argmax([blob.weight for blob in selected]))
    best_groups: Optional[List[np.ndarray]] = None
    best_coords: Optional[np.ndarray] = None
    best_score = math.inf

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            coords = projections.copy()
            coords[:, 0] *= sx
            coords[:, 1] *= sy
            x_rank = int(np.argsort(coords[:, 0]).tolist().index(anchor_idx))
            y_rank = int(np.argsort(coords[:, 1]).tolist().index(anchor_idx))
            row_groups = split_sorted_into_groups(coords[:, 1], ROWS)
            if row_groups is None:
                continue
            counts = sorted(len(group) for group in row_groups)
            count_penalty = sum(abs(a - b) for a, b in zip(counts, [4, 5, 5, 5]))
            row_with_anchor = next(
                (row for row, group in enumerate(row_groups) if anchor_idx in group),
                ROWS,
            )
            score = x_rank + y_rank + 10 * row_with_anchor + 20 * count_penalty
            if score < best_score:
                best_score = score
                best_groups = row_groups
                best_coords = coords

    if best_groups is None or best_coords is None:
        return [], "could not split blobs into grid rows"

    expected_cols_by_row = {
        0: [0, 1, 2, 3, 4],
        1: [0, 1, 3, 4],
        2: [0, 1, 2, 3, 4],
        3: [0, 1, 2, 3, 4],
    }
    dots_by_key = {(dot.row, dot.col): dot for dot in dots}
    matches: List[Match] = []

    for row, group in enumerate(best_groups):
        group_sorted = sorted(group.tolist(), key=lambda idx: float(best_coords[idx, 0]))
        expected_cols = expected_cols_by_row[row]
        if len(group_sorted) != len(expected_cols):
            return [], (
                f"row {row} has {len(group_sorted)} blobs, "
                f"expected {len(expected_cols)}"
            )
        for blob_idx, col in zip(group_sorted, expected_cols):
            dot = dots_by_key[(row, col)]
            blob = selected[blob_idx]
            matches.append(Match(dot=dot, blob=blob, reproj_error_px=0.0))

    if len(matches) != EXPECTED_DOTS:
        return [], f"matched {len(matches)}/{EXPECTED_DOTS}"

    src = np.array([[m.dot.object_x_mm, m.dot.object_y_mm] for m in matches], dtype=np.float64)
    dst = np.array([[m.blob.x, m.blob.y] for m in matches], dtype=np.float64)
    homography, _ = cv.findHomography(src, dst, method=0)
    if homography is not None:
        src_h = np.column_stack([src, np.ones(len(src))])
        projected = (homography @ src_h.T).T
        projected = projected[:, :2] / projected[:, 2:3]
        for match, point in zip(matches, projected):
            match.reproj_error_px = float(np.linalg.norm(point - np.array([match.blob.x, match.blob.y])))

    return matches, "ok"


def find_calibration_xml_files(search_dirs: Sequence[Path]) -> List[Path]:
    """List candidate OpenCV calibration XML files, newest first."""
    found: Dict[str, Path] = {}
    for directory in search_dirs:
        if not directory.is_dir():
            continue
        for path in directory.glob("*.xml"):
            try:
                camera_matrix, _, _ = load_calibration_xml(path)
            except (RuntimeError, cv.error):
                continue
            if camera_matrix is not None:
                found[str(path.resolve())] = path
    return sorted(found.values(), key=lambda p: p.stat().st_mtime, reverse=True)


def load_calibration_xml(path: Path) -> Tuple[np.ndarray, np.ndarray, str]:
    """Load camera_matrix and distortion_coefficients from an OpenCV XML file.

    Supports both a top-level camera node (chessboard tool, mire tool) and
    matrices stored directly at the root.
    """
    fs = cv.FileStorage(str(path), cv.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open calibration file: {path}")
    try:
        root = fs.root()
        candidate_nodes = [("", root)]
        for key in root.keys():
            node = root.getNode(key)
            if node.isMap():
                candidate_nodes.append((key, node))
        for name, node in candidate_nodes:
            matrix_node = node.getNode("camera_matrix")
            dist_node = node.getNode("distortion_coefficients")
            if matrix_node.empty() or dist_node.empty():
                continue
            camera_matrix = np.asarray(matrix_node.mat(), dtype=np.float64)
            dist_coeffs = np.asarray(dist_node.mat(), dtype=np.float64).reshape(-1)
            if camera_matrix.shape == (3, 3):
                return camera_matrix, dist_coeffs, name or path.stem
    finally:
        fs.release()
    raise RuntimeError(f"No camera_matrix/distortion_coefficients found in {path}")


def _adjacent_grid_pairs(matches: Sequence[Match]) -> List[Tuple[int, int]]:
    """Index pairs of matches that are horizontal or vertical grid neighbours."""
    by_key = {(m.dot.row, m.dot.col): idx for idx, m in enumerate(matches)}
    pairs: List[Tuple[int, int]] = []
    for (row, col), idx in by_key.items():
        right = by_key.get((row, col + 1))
        if right is not None:
            pairs.append((idx, right))
        below = by_key.get((row + 1, col))
        if below is not None:
            pairs.append((idx, below))
    return pairs


def _square_edge_pairs(matches: Sequence[Match]) -> List[Tuple[int, int]]:
    by_id = {m.dot.dot_id: idx for idx, m in enumerate(matches)}
    pairs: List[Tuple[int, int]] = []
    for a, b in (("tl", "tr"), ("tl", "bl"), ("tr", "br"), ("bl", "br")):
        if a in by_id and b in by_id:
            pairs.append((by_id[a], by_id[b]))
    return pairs


def solve_pose_from_matches(
    matches: Sequence[Match],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    min_points: int = 6,
) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray], str]:
    if len(matches) < min_points:
        return False, None, None, f"only {len(matches)} matched points"
    object_points = np.array(
        [[m.dot.object_x_mm, m.dot.object_y_mm, m.dot.object_z_mm] for m in matches],
        dtype=np.float64,
    )
    image_points = np.array([[m.blob.x, m.blob.y] for m in matches], dtype=np.float64)
    ok, rvec, tvec = cv.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs, flags=cv.SOLVEPNP_IPPE
    )
    if not ok:
        ok, rvec, tvec = cv.solvePnP(
            object_points, image_points, camera_matrix, dist_coeffs,
            flags=cv.SOLVEPNP_ITERATIVE,
        )
    if not ok:
        return False, None, None, "solvePnP failed"
    return True, rvec, tvec, "ok"


def plane_spacing_metrics(
    matches: Sequence[Match],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    pairs: Sequence[Tuple[int, int]],
) -> Dict[str, object]:
    if not pairs:
        return {
            "spacing_pairs": 0,
            "spacing_true_mean_mm": None,
            "spacing_measured_mean_mm": None,
            "spacing_mean_abs_error_mm": None,
            "spacing_max_abs_error_mm": None,
        }

    image_points = np.array([[m.blob.x, m.blob.y] for m in matches], dtype=np.float64)
    object_points = np.array(
        [[m.dot.object_x_mm, m.dot.object_y_mm, m.dot.object_z_mm] for m in matches],
        dtype=np.float64,
    )
    rotation, _ = cv.Rodrigues(rvec)
    plane_normal = rotation[:, 2]
    translation = tvec.reshape(3)
    undistorted = cv.undistortPoints(
        image_points.reshape(-1, 1, 2), camera_matrix, dist_coeffs
    ).reshape(-1, 2)
    rays = np.column_stack([undistorted, np.ones(len(undistorted))])
    denominators = rays @ plane_normal
    measured_mm: List[float] = []
    true_mm: List[float] = []
    if np.all(np.abs(denominators) > 1e-9):
        scales = (plane_normal @ translation) / denominators
        points_3d = rays * scales[:, None]
        for idx_a, idx_b in pairs:
            measured = float(np.linalg.norm(points_3d[idx_a] - points_3d[idx_b]))
            true = float(np.linalg.norm(object_points[idx_a] - object_points[idx_b]))
            measured_mm.append(measured)
            true_mm.append(true)

    spacing_errors = [m - t for m, t in zip(measured_mm, true_mm)]
    metrics: Dict[str, object] = {
        "spacing_pairs": len(spacing_errors),
        "spacing_true_mean_mm": float(np.mean(true_mm)) if true_mm else None,
        "spacing_measured_mean_mm": float(np.mean(measured_mm)) if measured_mm else None,
        "spacing_mean_abs_error_mm": float(np.mean(np.abs(spacing_errors))) if spacing_errors else None,
        "spacing_max_abs_error_mm": float(np.max(np.abs(spacing_errors))) if spacing_errors else None,
    }
    if spacing_errors and metrics["spacing_true_mean_mm"]:
        metrics["spacing_mean_error_percent"] = float(
            100.0 * np.mean(spacing_errors) / float(metrics["spacing_true_mean_mm"])
        )
    return metrics


def pose_summary(rvec: np.ndarray, tvec: np.ndarray) -> Dict[str, object]:
    rotation, _ = cv.Rodrigues(rvec)
    plane_normal = rotation[:, 2]
    translation = tvec.reshape(3)
    tilt_deg = math.degrees(
        math.acos(min(1.0, abs(float(plane_normal[2])) / max(1e-9, np.linalg.norm(plane_normal))))
    )
    return {
        "rvec": rvec.reshape(3).tolist(),
        "tvec": translation.tolist(),
        "distance_z_mm": float(translation[2]),
        "distance_norm_mm": float(np.linalg.norm(translation)),
        "tilt_deg": float(tilt_deg),
    }


def evaluate_calibration_on_matches(
    matches: Sequence[Match],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Dict[str, object]:
    """Validate fixed intrinsics against one mire capture.

    Returns reprojection errors, held-out prediction errors, the estimated
    camera-to-screen pose, and physical spacing measurements obtained by
    intersecting the back-projected rays with the estimated screen plane.
    """
    object_points = np.array(
        [[m.dot.object_x_mm, m.dot.object_y_mm, m.dot.object_z_mm] for m in matches],
        dtype=np.float64,
    )
    image_points = np.array([[m.blob.x, m.blob.y] for m in matches], dtype=np.float64)
    ok, rvec, tvec, reason = solve_pose_from_matches(matches, camera_matrix, dist_coeffs)
    if not ok:
        return {"valid": False, "reason": reason}
    assert rvec is not None and tvec is not None

    projected, _ = cv.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    errors = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)

    # Held-out check: pose from the border dots only, then predict the
    # interior dots that were never given to solvePnP.
    border = [
        i for i, m in enumerate(matches)
        if m.dot.row in (0, ROWS - 1) or m.dot.col in (0, COLS - 1)
    ]
    interior = [i for i in range(len(matches)) if i not in border]
    heldout_errors: Optional[np.ndarray] = None
    if len(border) >= 6 and len(interior) >= 2:
        ok_border, rvec_b, tvec_b = cv.solvePnP(
            object_points[border], image_points[border],
            camera_matrix, dist_coeffs, flags=cv.SOLVEPNP_IPPE,
        )
        if ok_border:
            predicted, _ = cv.projectPoints(
                object_points[interior], rvec_b, tvec_b, camera_matrix, dist_coeffs
            )
            heldout_errors = np.linalg.norm(
                predicted.reshape(-1, 2) - image_points[interior], axis=1
            )

    summary = pose_summary(rvec, tvec)
    spacing = plane_spacing_metrics(
        matches, camera_matrix, dist_coeffs, rvec, tvec, _adjacent_grid_pairs(matches)
    )
    result: Dict[str, object] = {
        "valid": True,
        "point_count": len(matches),
        "rms_px": float(np.sqrt(np.mean(errors**2))),
        "mean_px": float(np.mean(errors)),
        "max_px": float(np.max(errors)),
        "heldout_count": int(len(heldout_errors)) if heldout_errors is not None else 0,
        "heldout_mean_px": float(np.mean(heldout_errors)) if heldout_errors is not None else None,
        "heldout_max_px": float(np.max(heldout_errors)) if heldout_errors is not None else None,
    }
    result.update(summary)
    result.update(spacing)
    return result


def evaluate_square_validation(
    pose_matches: Sequence[Match],
    square_blobs: Sequence[Blob],
    square_dots: Sequence[ScreenDot],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[Dict[str, object], List[Match], List[Dict[str, object]]]:
    ok, rvec, tvec, reason = solve_pose_from_matches(pose_matches, camera_matrix, dist_coeffs)
    if not ok:
        return {"valid": False, "reason": reason}, [], []
    assert rvec is not None and tvec is not None
    if len(square_blobs) < len(square_dots):
        return {
            "valid": False,
            "reason": f"not enough square blobs: {len(square_blobs)}/{len(square_dots)}",
        }, [], []

    object_points = np.array(
        [[dot.object_x_mm, dot.object_y_mm, dot.object_z_mm] for dot in square_dots],
        dtype=np.float64,
    )
    projected, _ = cv.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected_points = projected.reshape(-1, 2)
    selected = list(square_blobs[: len(square_dots)])
    blob_points = np.array([[blob.x, blob.y] for blob in selected], dtype=np.float64)
    distances = np.linalg.norm(projected_points[:, None, :] - blob_points[None, :, :], axis=2)

    best_perm: Optional[Tuple[int, ...]] = None
    best_score = math.inf
    for perm in itertools.permutations(range(len(selected)), len(square_dots)):
        score = float(sum(distances[dot_idx, blob_idx] for dot_idx, blob_idx in enumerate(perm)))
        if score < best_score:
            best_score = score
            best_perm = tuple(perm)
    if best_perm is None:
        return {"valid": False, "reason": "could not associate square blobs"}, [], []

    matches: List[Match] = []
    for dot_idx, blob_idx in enumerate(best_perm):
        dot = square_dots[dot_idx]
        blob = selected[blob_idx]
        matches.append(
            Match(
                dot=dot,
                blob=blob,
                reproj_error_px=float(distances[dot_idx, blob_idx]),
            )
        )
    errors = np.array([match.reproj_error_px for match in matches], dtype=np.float64)
    summary = pose_summary(rvec, tvec)
    spacing = plane_spacing_metrics(
        matches, camera_matrix, dist_coeffs, rvec, tvec, _square_edge_pairs(matches)
    )
    expected = [
        {
            "dot_id": dot.dot_id,
            "object_mm": {
                "x": dot.object_x_mm,
                "y": dot.object_y_mm,
                "z": dot.object_z_mm,
            },
            "projected_px": {
                "x": float(point[0]),
                "y": float(point[1]),
            },
        }
        for dot, point in zip(square_dots, projected_points)
    ]
    result: Dict[str, object] = {
        "valid": True,
        "point_count": len(matches),
        "rms_px": float(np.sqrt(np.mean(errors**2))),
        "mean_px": float(np.mean(errors)),
        "max_px": float(np.max(errors)),
    }
    result.update(summary)
    result.update(spacing)
    return result, matches, expected


def normalize_activity_to_bgr(activity: np.ndarray) -> np.ndarray:
    if activity.size == 0 or float(np.max(activity)) <= 0.0:
        h, w = activity.shape if activity.ndim == 2 else (480, 640)
        return np.zeros((h, w, 3), dtype=np.uint8)
    normalized = np.clip(activity / float(np.max(activity)) * 255.0, 0, 255).astype(np.uint8)
    colored = cv.applyColorMap(normalized, cv.COLORMAP_INFERNO)
    return colored


def make_overlay(activity: np.ndarray, blobs: Sequence[Blob], matches: Sequence[Match]) -> np.ndarray:
    image = normalize_activity_to_bgr(activity)
    draw_blob_indicators(image, blobs, matches)
    return image


def draw_blob_indicators(
    image: np.ndarray,
    blobs: Sequence[Blob],
    matches: Sequence[Match] = (),
) -> np.ndarray:
    matched_blob_indices = {match.blob.index for match in matches}
    for blob in blobs:
        is_matched = blob.index in matched_blob_indices
        color = (0, 255, 0) if is_matched else (0, 180, 255)
        x, y, w, h = blob.bbox
        cv.rectangle(image, (x, y), (x + w, y + h), color, 1)
        cv.circle(image, (int(round(blob.x)), int(round(blob.y))), 7, color, 2)
        cv.drawMarker(
            image,
            (int(round(blob.x)), int(round(blob.y))),
            color,
            markerType=cv.MARKER_CROSS,
            markerSize=10,
            thickness=1,
        )
        cv.putText(
            image,
            f"#{blob.index}",
            (int(round(blob.x)) + 8, int(round(blob.y)) - 8),
            cv.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv.LINE_AA,
        )

    for match in matches:
        text = match.dot.dot_id if match.dot.label is not None else f"{match.dot.row},{match.dot.col}"
        cv.putText(
            image,
            text,
            (int(round(match.blob.x)) + 10, int(round(match.blob.y)) + 12),
            cv.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv.LINE_AA,
        )
    return image


def draw_expected_points(
    image: np.ndarray,
    expected: Sequence[Dict[str, object]],
) -> np.ndarray:
    for point in expected:
        projected = point.get("projected_px", {})
        if not isinstance(projected, dict):
            continue
        x = int(round(float(projected.get("x", 0.0))))
        y = int(round(float(projected.get("y", 0.0))))
        label = str(point.get("dot_id", "expected"))
        color = (255, 80, 80)
        cv.drawMarker(
            image,
            (x, y),
            color,
            markerType=cv.MARKER_TILTED_CROSS,
            markerSize=14,
            thickness=2,
        )
        cv.putText(
            image,
            f"E {label}",
            (x + 8, y + 16),
            cv.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv.LINE_AA,
        )
    return image


def draw_preview_banner(image: np.ndarray, text: str, color: Tuple[int, int, int]) -> np.ndarray:
    cv.rectangle(image, (0, 0), (image.shape[1], 30), (0, 0, 0), -1)
    cv.putText(
        image,
        text,
        (8, 21),
        cv.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        1,
        cv.LINE_AA,
    )
    return image


def pixmap_from_bgr(image: np.ndarray, target_size: QtCore.QSize) -> QtGui.QPixmap:
    rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    h, w, channels = rgb.shape
    qimage = QtGui.QImage(rgb.data, w, h, channels * w, QtGui.QImage.Format_RGB888).copy()
    pixmap = QtGui.QPixmap.fromImage(qimage)
    return pixmap.scaled(target_size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)


def load_dv_processing():
    try:
        import dv_processing as dv  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Python module dv_processing is missing. "
            "Install it with: sudo apt install dv-processing-python"
        ) from exc
    return dv


def read_descriptor_field(device: object, name: str) -> object:
    value = getattr(device, name, None)
    if callable(value):
        try:
            value = value()
        except TypeError:
            pass
    return value


def discover_camera_descriptions() -> List[str]:
    dv = load_dv_processing()
    devices = dv.io.camera.discover()
    descriptions: List[str] = []
    for index, device in enumerate(devices):
        model = read_descriptor_field(device, "cameraModel")
        serial = read_descriptor_field(device, "serialNumber")
        dev_type = read_descriptor_field(device, "deviceType")
        firmware = read_descriptor_field(device, "firmwareVersion")
        bus = read_descriptor_field(device, "busNumber")
        address = read_descriptor_field(device, "devAddress")
        descriptions.append(
            f"{index}: model={model} serial={serial} type={dev_type} "
            f"fw={firmware} usb={bus}:{address}"
        )
    return descriptions


class EventCamera:
    def __init__(self) -> None:
        dv = load_dv_processing()
        self.dv = dv
        self.capture = dv.io.camera.open()
        if not self.capture.isEventStreamAvailable():
            raise RuntimeError("Camera does not provide an event stream.")
        resolution = self.capture.getEventResolution()
        self.width, self.height = self._parse_resolution(resolution)

    @staticmethod
    def _parse_resolution(resolution: object) -> Tuple[int, int]:
        if isinstance(resolution, tuple) and len(resolution) >= 2:
            return int(resolution[0]), int(resolution[1])
        if isinstance(resolution, list) and len(resolution) >= 2:
            return int(resolution[0]), int(resolution[1])
        width_attr = getattr(resolution, "width", None)
        height_attr = getattr(resolution, "height", None)
        if callable(width_attr):
            width_attr = width_attr()
        if callable(height_attr):
            height_attr = height_attr()
        if width_attr is not None and height_attr is not None:
            return int(width_attr), int(height_attr)
        raise RuntimeError(f"Unsupported camera resolution object: {resolution!r}")

    def poll(self):
        return self.capture.getNextEventBatch()

    def close(self) -> None:
        close = getattr(self.capture, "close", None)
        if callable(close):
            close()


def event_coordinates(events: object) -> np.ndarray:
    coords_method = getattr(events, "coordinates", None)
    if callable(coords_method):
        coords = np.asarray(coords_method())
        if coords.ndim == 2 and coords.shape[1] >= 2:
            return coords[:, :2].astype(np.int32, copy=False)

    coords: List[Tuple[int, int]] = []
    for event in events:  # type: ignore[operator]
        x_attr = getattr(event, "x", None)
        y_attr = getattr(event, "y", None)
        x = x_attr() if callable(x_attr) else x_attr
        y = y_attr() if callable(y_attr) else y_attr
        coords.append((int(x), int(y)))
    return np.asarray(coords, dtype=np.int32)


class MireWindow(QtWidgets.QWidget):
    calibration_started = QtCore.pyqtSignal()
    calibration_done = QtCore.pyqtSignal()

    def __init__(self, blink_hz: float, gradient_softness: int) -> None:
        super().__init__()
        self.setWindowTitle("Mire calibration")
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
        self.setStyleSheet("background: black;")
        self.monitor = MonitorInfo("unknown", 0, 0, 640, 480, 0.0, 0.0, 0.0, 0.0, 1.0, "none")
        self.dots: List[ScreenDot] = []
        self.layout_meta: Dict[str, float] = {}
        self.pattern = "mire"
        self.square_variant: Dict[str, object] = dict(SQUARE_SEQUENCE[0])
        self.lit = False
        self.blink_hz = blink_hz
        self.gradient_softness = int(np.clip(gradient_softness, 0, 100))
        self.blink_timer = QtCore.QTimer(self)
        self.blink_timer.timeout.connect(self.toggle_lit)
        self.calibration_timer = QtCore.QTimer(self)
        self.calibration_timer.setSingleShot(True)
        self.calibration_timer.timeout.connect(self.finish_calibration_blink)
        self.restart_blink()

    def restart_blink(self) -> None:
        interval_ms = max(8, int(500.0 / max(0.1, self.blink_hz)))
        self.blink_timer.start(interval_ms)

    def set_blink_hz(self, blink_hz: float) -> None:
        self.blink_hz = blink_hz
        self.restart_blink()

    def set_gradient_softness(self, gradient_softness: int) -> None:
        self.gradient_softness = int(np.clip(gradient_softness, 0, 100))
        self.update()

    def set_pattern(self, pattern: str) -> None:
        if pattern not in {"mire", "square4"}:
            pattern = "mire"
        if self.pattern == pattern:
            return
        self.pattern = pattern
        self.update_layout()

    def set_square_variant(self, variant: Dict[str, object]) -> None:
        self.square_variant = dict(variant)
        if self.pattern == "square4":
            self.update_layout()

    def show_on_monitor(self, monitor: MonitorInfo) -> None:
        self.monitor = monitor
        self.setGeometry(monitor.x, monitor.y, monitor.width_px, monitor.height_px)
        self.update_layout()
        self.showFullScreen()
        self.raise_()
        self.activateWindow()

    def update_layout(self) -> None:
        if self.pattern == "square4":
            self.dots, self.layout_meta = build_square_layout(
                self.monitor.width_px,
                self.monitor.height_px,
                self.monitor.mm_per_px_x,
                self.monitor.mm_per_px_y,
                offset_x=float(self.square_variant.get("offset_x", 0.0)),
                offset_y=float(self.square_variant.get("offset_y", 0.0)),
                side_scale=float(self.square_variant.get("side_scale", 2.0)),
                variant_id=str(self.square_variant.get("id", "square")),
                variant_label=str(self.square_variant.get("label", "carre")),
            )
        else:
            self.dots, self.layout_meta = build_mire_layout(
                self.monitor.width_px,
                self.monitor.height_px,
                self.monitor.mm_per_px_x,
                self.monitor.mm_per_px_y,
            )
        self.update()

    def toggle_lit(self) -> None:
        self.lit = not self.lit
        self.update()

    def start_calibration_blink(self, duration_ms: int) -> None:
        self.blink_timer.stop()
        self.lit = False
        self.update()
        QtCore.QTimer.singleShot(80, self._start_active_calibration_blink)
        self.calibration_timer.start(max(100, duration_ms + 80))

    def _start_active_calibration_blink(self) -> None:
        self.lit = True
        self.update()
        self.restart_blink()
        self.calibration_started.emit()

    def finish_calibration_blink(self) -> None:
        self.restart_blink()
        self.calibration_done.emit()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor(0, 0, 0))
        if not self.lit:
            painter.end()
            return

        softness = self.gradient_softness / 100.0
        inner_stop = 0.85 - 0.65 * softness
        mid_stop = 0.96 - 0.24 * softness
        mid_value = int(245 - 145 * softness)

        for dot in self.dots:
            gradient = QtGui.QRadialGradient(
                QtCore.QPointF(dot.screen_x_px, dot.screen_y_px),
                dot.radius_px,
            )
            gradient.setColorAt(0.0, QtGui.QColor(255, 255, 255))
            gradient.setColorAt(inner_stop, QtGui.QColor(255, 255, 255))
            gradient.setColorAt(mid_stop, QtGui.QColor(mid_value, mid_value, mid_value))
            gradient.setColorAt(1.0, QtGui.QColor(0, 0, 0))
            painter.setBrush(QtGui.QBrush(gradient))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(
                QtCore.QPointF(dot.screen_x_px, dot.screen_y_px),
                dot.radius_px,
                dot.radius_px,
            )
        painter.end()


class ControlWindow(QtWidgets.QWidget):
    def __init__(self, args: argparse.Namespace, monitors: List[MonitorInfo]) -> None:
        super().__init__()
        self.args = args
        self.base_monitors = monitors
        self.selected_monitor_index = select_monitor(monitors, args.monitor)
        self.camera: Optional[EventCamera] = None
        self.activity: Optional[np.ndarray] = None
        self.live_activity: Optional[np.ndarray] = None
        self.event_count = 0
        self.live_event_count = 0
        self.accumulating = False
        self.accum_started_at = 0.0
        self.current_capture_duration_ms = int(args.accum_ms)
        self.last_preview_update = 0.0
        self.last_preview_blob_update = 0.0
        self.preview_blobs: List[Blob] = []
        self.last_blobs: List[Blob] = []
        self.last_matches: List[Match] = []
        self.last_export_paths: List[Path] = []
        self.test_mode = False
        self.square_phase: Optional[str] = None
        self.square_test_context: Optional[Dict[str, object]] = None
        self.square_validation_index = 0

        self.mire = MireWindow(args.blink_hz, args.gradient_softness)
        self.mire.calibration_started.connect(self.begin_accumulation)
        self.mire.calibration_done.connect(self.finish_calibration)

        self.setWindowTitle("Calibration mire evenementielle")
        self.resize(1060, 760)
        self._build_ui()
        self._connect_ui()
        self.refresh_monitor_labels()
        self.place_control_window()

        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_camera)
        self.poll_timer.start(5)

        if self.selected_monitor_index >= 0:
            self.show_mire_on_selected_monitor()
        QtCore.QTimer.singleShot(0, self.ensure_camera)

    def _build_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)

        monitor_row = QtWidgets.QHBoxLayout()
        self.monitor_combo = QtWidgets.QComboBox()
        self.monitor_combo.setMinimumWidth(560)
        self.show_mire_button = QtWidgets.QPushButton("Afficher mire")
        self.hide_mire_button = QtWidgets.QPushButton("Masquer mire")
        monitor_row.addWidget(QtWidgets.QLabel("Ecran mire:"))
        monitor_row.addWidget(self.monitor_combo, 1)
        monitor_row.addWidget(self.show_mire_button)
        monitor_row.addWidget(self.hide_mire_button)
        root.addLayout(monitor_row)

        self.monitor_details = QtWidgets.QLabel()
        self.monitor_details.setWordWrap(True)
        root.addWidget(self.monitor_details)

        camera_row = QtWidgets.QHBoxLayout()
        self.camera_status_label = QtWidgets.QLabel("Camera: non testee")
        self.camera_status_label.setMinimumHeight(28)
        self.camera_status_label.setStyleSheet(
            "padding: 4px 8px; background: #3b3320; color: #f4d27a; border: 1px solid #715f32;"
        )
        self.reconnect_camera_button = QtWidgets.QPushButton("Reconnecter camera")
        camera_row.addWidget(self.camera_status_label, 1)
        camera_row.addWidget(self.reconnect_camera_button)
        root.addLayout(camera_row)

        accum_row = QtWidgets.QHBoxLayout()
        self.accum_slider_label = QtWidgets.QLabel("Fenetre accumulation:")
        self.accum_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.accum_slider.setRange(50, 10000)
        self.accum_slider.setSingleStep(10)
        self.accum_slider.setPageStep(250)
        self.accum_slider.setTickInterval(1000)
        self.accum_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.accum_slider.setValue(int(np.clip(self.args.accum_ms, 50, 10000)))
        self.args.accum_ms = int(self.accum_slider.value())
        self.accum_value_label = QtWidgets.QLabel(f"{self.args.accum_ms} ms")
        self.accum_value_label.setMinimumWidth(70)
        accum_row.addWidget(self.accum_slider_label)
        accum_row.addWidget(self.accum_slider, 1)
        accum_row.addWidget(self.accum_value_label)
        root.addLayout(accum_row)

        mire_params_row = QtWidgets.QHBoxLayout()
        self.blink_slider_label = QtWidgets.QLabel("Frequence:")
        self.blink_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.blink_slider.setRange(5, 300)
        self.blink_slider.setSingleStep(1)
        self.blink_slider.setPageStep(10)
        self.blink_slider.setTickInterval(50)
        self.blink_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.blink_slider.setValue(int(np.clip(round(self.args.blink_hz * 10.0), 5, 300)))
        self.args.blink_hz = self.blink_slider.value() / 10.0
        self.blink_value_label = QtWidgets.QLabel(f"{self.args.blink_hz:.1f} Hz")
        self.blink_value_label.setMinimumWidth(70)
        mire_params_row.addWidget(self.blink_slider_label)
        mire_params_row.addWidget(self.blink_slider, 1)
        mire_params_row.addWidget(self.blink_value_label)

        self.gradient_slider_label = QtWidgets.QLabel("Gradient:")
        self.gradient_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.gradient_slider.setRange(0, 100)
        self.gradient_slider.setSingleStep(1)
        self.gradient_slider.setPageStep(10)
        self.gradient_slider.setTickInterval(25)
        self.gradient_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.gradient_slider.setValue(int(np.clip(self.args.gradient_softness, 0, 100)))
        self.args.gradient_softness = int(self.gradient_slider.value())
        self.gradient_value_label = QtWidgets.QLabel(f"{self.args.gradient_softness} %")
        self.gradient_value_label.setMinimumWidth(55)
        mire_params_row.addWidget(self.gradient_slider_label)
        mire_params_row.addWidget(self.gradient_slider, 1)
        mire_params_row.addWidget(self.gradient_value_label)
        root.addLayout(mire_params_row)

        controls = QtWidgets.QHBoxLayout()
        self.calib_button = QtWidgets.QPushButton("Calib")
        self.erase_button = QtWidgets.QPushButton("Erase")
        self.reset_button = QtWidgets.QPushButton("Reset")
        self.calib_button.setMinimumHeight(38)
        self.erase_button.setMinimumHeight(38)
        self.reset_button.setMinimumHeight(38)
        controls.addWidget(self.calib_button)
        controls.addWidget(self.erase_button)
        controls.addWidget(self.reset_button)
        controls.addStretch(1)
        root.addLayout(controls)

        test_row = QtWidgets.QHBoxLayout()
        self.calib_file_combo = QtWidgets.QComboBox()
        self.calib_file_combo.setMinimumWidth(360)
        self.refresh_calib_button = QtWidgets.QPushButton("Rafraichir")
        self.measured_distance_edit = QtWidgets.QLineEdit()
        self.measured_distance_edit.setPlaceholderText("distance reelle camera-ecran en mm (optionnel)")
        self.measured_distance_edit.setMaximumWidth(300)
        self.test_button = QtWidgets.QPushButton("Test calib")
        self.square_test_button = QtWidgets.QPushButton("Test carre")
        self.test_button.setMinimumHeight(38)
        self.square_test_button.setMinimumHeight(38)
        test_row.addWidget(QtWidgets.QLabel("Calibration:"))
        test_row.addWidget(self.calib_file_combo, 1)
        test_row.addWidget(self.refresh_calib_button)
        test_row.addWidget(self.measured_distance_edit)
        test_row.addWidget(self.test_button)
        test_row.addWidget(self.square_test_button)
        root.addLayout(test_row)
        self.populate_calibration_files()

        self.preview_label = QtWidgets.QLabel()
        self.preview_label.setMinimumSize(720, 480)
        self.preview_label.setAlignment(QtCore.Qt.AlignCenter)
        self.preview_label.setStyleSheet("background: #111; color: #ddd; border: 1px solid #333;")
        self.preview_label.setText("Preview camera / accumulation")
        root.addWidget(self.preview_label, 1)

        self.status_text = QtWidgets.QPlainTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(150)
        root.addWidget(self.status_text)

    def _connect_ui(self) -> None:
        self.monitor_combo.currentIndexChanged.connect(self.on_monitor_changed)
        self.show_mire_button.clicked.connect(self.show_mire_on_selected_monitor)
        self.hide_mire_button.clicked.connect(self.mire.hide)
        self.reconnect_camera_button.clicked.connect(self.reconnect_camera)
        self.accum_slider.valueChanged.connect(self.on_accum_slider_changed)
        self.blink_slider.valueChanged.connect(self.on_blink_slider_changed)
        self.gradient_slider.valueChanged.connect(self.on_gradient_slider_changed)
        self.calib_button.clicked.connect(self.start_calibration)
        self.erase_button.clicked.connect(self.erase_current)
        self.reset_button.clicked.connect(self.reset_all)
        self.test_button.clicked.connect(self.start_test)
        self.square_test_button.clicked.connect(self.start_square_test)
        self.refresh_calib_button.clicked.connect(self.populate_calibration_files)

    def selected_monitor(self) -> Optional[MonitorInfo]:
        if not (0 <= self.selected_monitor_index < len(self.base_monitors)):
            return None
        monitor = self.base_monitors[self.selected_monitor_index]
        return apply_size_override(monitor, self.args.screen_width_mm, self.args.screen_height_mm)

    def refresh_monitor_labels(self) -> None:
        self.monitor_combo.blockSignals(True)
        self.monitor_combo.clear()
        for idx, monitor in enumerate(self.base_monitors):
            self.monitor_combo.addItem(monitor.label(), idx)
        if 0 <= self.selected_monitor_index < self.monitor_combo.count():
            self.monitor_combo.setCurrentIndex(self.selected_monitor_index)
        self.monitor_combo.blockSignals(False)
        self.update_monitor_details()

    def update_monitor_details(self) -> None:
        monitor = self.selected_monitor()
        if monitor is None:
            self.monitor_details.setText("Aucun ecran detecte.")
            return
        manual = " oui" if "manual-size" in monitor.source else " non"
        self.monitor_details.setText(
            f"Selection: {monitor.label()} | source={monitor.source} | "
            f"override manuel={manual}"
        )

    def place_control_window(self) -> None:
        if not self.base_monitors:
            return
        control_monitor = None
        for idx, monitor in enumerate(self.base_monitors):
            if idx != self.selected_monitor_index:
                control_monitor = monitor
                break
        if control_monitor is None:
            control_monitor = self.base_monitors[self.selected_monitor_index]

        width = min(1060, max(760, control_monitor.width_px - 120))
        height = min(760, max(560, control_monitor.height_px - 120))
        x = control_monitor.x + max(20, (control_monitor.width_px - width) // 2)
        y = control_monitor.y + max(20, (control_monitor.height_px - height) // 2)
        self.setGeometry(x, y, width, height)

    def append_status(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.status_text.appendPlainText(f"[{stamp}] {message}")

    def set_capture_buttons_enabled(self, enabled: bool) -> None:
        self.calib_button.setEnabled(enabled)
        self.test_button.setEnabled(enabled)
        self.square_test_button.setEnabled(enabled)

    def set_camera_status(self, message: str, state: str) -> None:
        if state == "ok":
            style = "padding: 4px 8px; background: #17351f; color: #9ff0a8; border: 1px solid #2d7d3a;"
        elif state == "warn":
            style = "padding: 4px 8px; background: #3b3320; color: #f4d27a; border: 1px solid #715f32;"
        else:
            style = "padding: 4px 8px; background: #3a1b1b; color: #ffaaaa; border: 1px solid #7f3333;"
        self.camera_status_label.setText(message)
        self.camera_status_label.setStyleSheet(style)

    def on_accum_slider_changed(self, value: int) -> None:
        self.args.accum_ms = int(value)
        self.accum_value_label.setText(f"{self.args.accum_ms} ms")
        if self.accumulating:
            self.append_status(
                "Le changement de fenetre d'accumulation sera applique a la prochaine capture."
            )

    def on_blink_slider_changed(self, value: int) -> None:
        self.args.blink_hz = float(value) / 10.0
        self.blink_value_label.setText(f"{self.args.blink_hz:.1f} Hz")
        self.mire.set_blink_hz(self.args.blink_hz)

    def on_gradient_slider_changed(self, value: int) -> None:
        self.args.gradient_softness = int(value)
        self.gradient_value_label.setText(f"{self.args.gradient_softness} %")
        self.mire.set_gradient_softness(self.args.gradient_softness)

    def on_monitor_changed(self, index: int) -> None:
        self.selected_monitor_index = int(self.monitor_combo.itemData(index))
        self.update_monitor_details()
        self.place_control_window()
        if self.mire.isVisible():
            self.show_mire_on_selected_monitor()

    def show_mire_on_selected_monitor(self) -> None:
        monitor = self.selected_monitor()
        if monitor is None:
            self.append_status("Impossible d'afficher la mire: aucun ecran.")
            return
        if not monitor.valid_size_mm:
            self.append_status(
                "Taille physique inconnue. Utiliser --screen-width-mm et --screen-height-mm."
            )
        self.mire.set_blink_hz(self.args.blink_hz)
        self.mire.set_gradient_softness(self.args.gradient_softness)
        self.mire.show_on_monitor(monitor)
        self.append_status(f"Mire affichee sur {monitor.label()}")

    def ensure_camera(self) -> bool:
        if self.camera is not None:
            return True
        self.set_camera_status("Camera: recherche...", "warn")
        try:
            devices = discover_camera_descriptions()
            if not devices:
                self.set_camera_status("Camera: aucune DVXplorer detectee", "error")
                self.append_status("Aucune camera detectee par dv.io.camera.discover().")
                return False
            self.append_status("Camera detectee: " + " | ".join(devices))
            self.camera = EventCamera()
        except Exception as exc:  # noqa: BLE001
            self.set_camera_status(f"Camera: erreur connexion - {exc}", "error")
            self.append_status(str(exc))
            return False
        self.activity = np.zeros((self.camera.height, self.camera.width), dtype=np.float32)
        self.live_activity = np.zeros((self.camera.height, self.camera.width), dtype=np.float32)
        self.live_event_count = 0
        self.set_camera_status(
            f"Camera connectee: {self.camera.width}x{self.camera.height} | events live: 0",
            "ok",
        )
        blank = draw_preview_banner(
            np.zeros((self.camera.height, self.camera.width, 3), dtype=np.uint8),
            "LIVE 0 events - bouger la mire ou changer la luminosite",
            (80, 220, 255),
        )
        self.preview_label.setPixmap(pixmap_from_bgr(blank, self.preview_label.size()))
        self.append_status(f"Camera ouverte: {self.camera.width}x{self.camera.height}")
        return True

    def reconnect_camera(self) -> None:
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        self.activity = None
        self.live_activity = None
        self.live_event_count = 0
        self.preview_blobs = []
        self.preview_label.clear()
        self.preview_label.setText("Reconnexion camera...")
        self.ensure_camera()

    def poll_camera(self) -> None:
        if self.camera is None:
            return
        try:
            events = self.camera.poll()
        except Exception as exc:  # noqa: BLE001
            self.set_camera_status(f"Camera: erreur lecture - {exc}", "error")
            self.append_status(f"Erreur camera: {exc}")
            return
        if events is None:
            return

        coords = event_coordinates(events)
        if coords.size == 0:
            return

        if self.live_activity is not None:
            valid_live = self.add_events_to_array(self.live_activity, coords)
            self.live_event_count += valid_live
        if self.accumulating:
            self.add_events_to_activity(coords)
        self.update_live_preview()

    @staticmethod
    def add_events_to_array(activity: np.ndarray, coords: np.ndarray) -> int:
        xs = coords[:, 0].astype(np.int32, copy=False)
        ys = coords[:, 1].astype(np.int32, copy=False)
        valid = (
            (xs >= 0)
            & (ys >= 0)
            & (xs < activity.shape[1])
            & (ys < activity.shape[0])
        )
        if not np.any(valid):
            return 0
        np.add.at(activity, (ys[valid], xs[valid]), 1.0)
        return int(np.count_nonzero(valid))

    def add_events_to_activity(self, coords: np.ndarray) -> None:
        if self.activity is None:
            return
        self.event_count += self.add_events_to_array(self.activity, coords)

    def update_live_preview(self) -> None:
        now = time.time()
        if now - self.last_preview_update < 0.04:
            return
        self.last_preview_update = now

        if self.accumulating and self.activity is not None:
            source_activity = self.activity
            if now - self.last_preview_blob_update >= 0.12:
                self.preview_blobs = detect_blobs(source_activity, EXPECTED_DOTS)
                self.last_preview_blob_update = now
            image = normalize_activity_to_bgr(self.activity)
            draw_blob_indicators(image, self.preview_blobs)
            banner = (
                f"ACCUM {self.event_count} events | "
                f"blobs {len(self.preview_blobs)}/{EXPECTED_DOTS} | "
                f"fenetre {self.current_capture_duration_ms} ms"
            )
            image = draw_preview_banner(image, banner, (0, 255, 255))
        elif self.live_activity is not None:
            source_activity = self.live_activity
            if now - self.last_preview_blob_update >= 0.12:
                self.preview_blobs = detect_blobs(source_activity, EXPECTED_DOTS)
                self.last_preview_blob_update = now
            image = normalize_activity_to_bgr(self.live_activity)
            draw_blob_indicators(image, self.preview_blobs)
            image = draw_preview_banner(
                image,
                f"LIVE {self.live_event_count} events | blobs {len(self.preview_blobs)}/{EXPECTED_DOTS}",
                (80, 220, 255),
            )
            self.live_activity *= 0.65
        else:
            return

        if self.camera is not None:
            self.set_camera_status(
                f"Camera connectee: {self.camera.width}x{self.camera.height} | "
                f"events live: {self.live_event_count}",
                "ok",
            )
        self.preview_label.setPixmap(pixmap_from_bgr(image, self.preview_label.size()))

    def start_calibration(self) -> None:
        self._start_capture(test_mode=False)

    def start_test(self) -> None:
        if self.selected_calibration_path() is None:
            self.append_status("Aucune calibration XML trouvee. Lancer d'abord calibrate_intrinsics_from_mire.py.")
            return
        self._start_capture(test_mode=True)

    def start_square_test(self) -> None:
        selected = self.selected_calibration_path()
        if selected is None:
            self.append_status("Aucune calibration XML trouvee. Lancer d'abord calibrate_intrinsics_from_mire.py.")
            return
        try:
            camera_matrix, dist_coeffs, node_name = load_calibration_xml(selected)
        except (RuntimeError, cv.error) as exc:
            self.append_status(f"Lecture calibration impossible: {exc}")
            return
        self.square_test_context = {
            "calibration_file": selected,
            "calibration_node": node_name,
            "camera_matrix": camera_matrix,
            "dist_coeffs": dist_coeffs,
            "square_sequence": [dict(variant) for variant in SQUARE_SEQUENCE],
            "square_results": [],
        }
        self.square_validation_index = 0
        self._start_capture(test_mode=False, square_phase="pose")

    def square_capture_duration_ms(self) -> int:
        return max(int(self.args.accum_ms) * 4, 1200)

    def _start_capture(
        self,
        test_mode: bool,
        square_phase: Optional[str] = None,
        duration_ms: Optional[int] = None,
    ) -> None:
        if not self.ensure_camera():
            return
        monitor = self.selected_monitor()
        if monitor is None:
            self.append_status("Aucun ecran selectionne.")
            return
        if square_phase == "validation":
            variant = SQUARE_SEQUENCE[self.square_validation_index]
            self.mire.set_square_variant(variant)
        self.mire.set_pattern("square4" if square_phase == "validation" else "mire")
        if not self.mire.isVisible():
            self.show_mire_on_selected_monitor()

        self.activity = np.zeros((self.camera.height, self.camera.width), dtype=np.float32)
        self.event_count = 0
        self.last_blobs = []
        self.last_matches = []
        self.preview_blobs = []
        self.last_preview_blob_update = 0.0
        self.accumulating = False
        self.accum_started_at = 0.0
        self.test_mode = test_mode
        self.square_phase = square_phase
        self.current_capture_duration_ms = int(duration_ms if duration_ms is not None else self.args.accum_ms)
        self.set_capture_buttons_enabled(False)
        if square_phase == "pose":
            message = "Test carre phase 1/2: estimation pose avec la mire 19 points."
        elif square_phase == "validation":
            variant = SQUARE_SEQUENCE[self.square_validation_index]
            message = (
                f"Test carre {self.square_validation_index + 1}/{len(SQUARE_SEQUENCE)}: "
                f"{variant['label']} pendant {self.current_capture_duration_ms} ms."
            )
        elif test_mode:
            message = "Sequence de test lancee: noir -> mire clignotante."
        else:
            message = "Sequence de calibration lancee: noir -> mire clignotante."
        self.append_status(message)
        self.mire.start_calibration_blink(self.current_capture_duration_ms)

    def begin_accumulation(self) -> None:
        if self.camera is None or self.activity is None:
            return
        self.accumulating = True
        self.accum_started_at = time.time()
        self.append_status(
            f"Accumulation active pour {self.current_capture_duration_ms} ms "
            f"(polarites ON/OFF additionnees)."
        )

    def finish_calibration(self) -> None:
        if not self.accumulating:
            return
        self.accumulating = False
        square_phase = self.square_phase
        if square_phase not in ("pose", "validation"):
            self.set_capture_buttons_enabled(True)
        elapsed_ms = (time.time() - self.accum_started_at) * 1000.0
        if self.activity is None:
            self.append_status("Aucune accumulation disponible.")
            self.square_phase = None
            self.square_test_context = None
            self.set_capture_buttons_enabled(True)
            return

        expected = SQUARE_EXPECTED_DOTS if square_phase == "validation" else EXPECTED_DOTS
        self.last_blobs = detect_blobs(self.activity, expected)
        if square_phase == "validation":
            self.finish_square_validation(elapsed_ms)
            return

        self.last_matches, reason = associate_blobs_to_layout(self.last_blobs, self.mire.dots)
        overlay = make_overlay(self.activity, self.last_blobs, self.last_matches)
        self.preview_label.setPixmap(pixmap_from_bgr(overlay, self.preview_label.size()))

        self.append_status(
            f"Detection: {len(self.last_blobs)} blobs, "
            f"{len(self.last_matches)}/{EXPECTED_DOTS} associations, "
            f"{self.event_count} events, {elapsed_ms:.0f} ms. {reason}"
        )
        if self.test_mode:
            self.test_mode = False
            if len(self.last_matches) >= self.args.min_matched:
                self.run_calibration_test(overlay, elapsed_ms)
            else:
                self.append_status(
                    f"Test ignore: associations insuffisantes "
                    f"({len(self.last_matches)}/{self.args.min_matched})."
                )
            return
        if square_phase == "pose":
            if len(self.last_matches) >= self.args.min_matched:
                self.finish_square_pose(overlay, elapsed_ms)
            else:
                self.square_phase = None
                self.square_test_context = None
                self.set_capture_buttons_enabled(True)
                self.append_status(
                    f"Test carre ignore: associations phase 1 insuffisantes "
                    f"({len(self.last_matches)}/{self.args.min_matched})."
                )
            return
        if len(self.last_matches) >= self.args.min_matched:
            self.export_observation(overlay, elapsed_ms)
        else:
            self.append_status(
                f"Export ignore: associations insuffisantes "
                f"({len(self.last_matches)}/{self.args.min_matched})."
            )

    def finish_square_pose(self, overlay: np.ndarray, elapsed_ms: float) -> None:
        if self.square_test_context is None:
            self.square_phase = None
            self.set_capture_buttons_enabled(True)
            self.append_status("Test carre abandonne: contexte interne manquant.")
            return

        camera_matrix = self.square_test_context["camera_matrix"]
        dist_coeffs = self.square_test_context["dist_coeffs"]
        assert isinstance(camera_matrix, np.ndarray)
        assert isinstance(dist_coeffs, np.ndarray)
        pose_result = evaluate_calibration_on_matches(self.last_matches, camera_matrix, dist_coeffs)
        if not pose_result.get("valid"):
            self.square_phase = None
            self.square_test_context = None
            self.set_capture_buttons_enabled(True)
            self.append_status(f"Test carre echoue phase 1: {pose_result.get('reason')}")
            return

        self.square_test_context.update(
            {
                "pose_elapsed_ms": elapsed_ms,
                "pose_event_count": self.event_count,
                "pose_layout": dict(self.mire.layout_meta),
                "pose_matches": list(self.last_matches),
                "pose_result": pose_result,
            }
        )
        self.append_status(
            f"Phase 1 ok: pose estimee avec {len(self.last_matches)} points, "
            f"rms {pose_result['rms_px']:.2f} px, distance {pose_result['distance_norm_mm']:.0f} mm."
        )
        self.preview_label.setPixmap(pixmap_from_bgr(overlay, self.preview_label.size()))
        self.square_validation_index = 0
        self._start_capture(
            test_mode=False,
            square_phase="validation",
            duration_ms=self.square_capture_duration_ms(),
        )

    def finish_square_validation(self, elapsed_ms: float) -> None:
        context = self.square_test_context
        if context is None:
            self.square_phase = None
            self.set_capture_buttons_enabled(True)
            self.append_status("Test carre abandonne: contexte phase 1 manquant.")
            return

        camera_matrix = context["camera_matrix"]
        dist_coeffs = context["dist_coeffs"]
        pose_matches = context.get("pose_matches", [])
        records = context.setdefault("square_results", [])
        assert isinstance(camera_matrix, np.ndarray)
        assert isinstance(dist_coeffs, np.ndarray)
        assert isinstance(pose_matches, list)
        assert isinstance(records, list)
        variant = dict(SQUARE_SEQUENCE[self.square_validation_index])

        result, square_matches, expected_points = evaluate_square_validation(
            pose_matches,
            self.last_blobs,
            self.mire.dots,
            camera_matrix,
            dist_coeffs,
        )
        self.last_matches = square_matches
        overlay = make_overlay(self.activity, self.last_blobs, self.last_matches)
        draw_expected_points(overlay, expected_points)

        measured_mm = self.measured_distance_mm()
        if measured_mm is not None and result.get("valid"):
            distance_error = 100.0 * (result["distance_norm_mm"] - measured_mm) / measured_mm
            result["measured_distance_mm"] = measured_mm
            result["distance_error_percent"] = distance_error

        spacing_error = result.get("spacing_mean_abs_error_mm")
        record = {
            "index": self.square_validation_index,
            "variant": variant,
            "duration_ms_requested": self.current_capture_duration_ms,
            "duration_ms_measured": elapsed_ms,
            "blink_hz": self.args.blink_hz,
            "events_accumulated": self.event_count,
            "layout": dict(self.mire.layout_meta),
            "dots": [dot.to_json() for dot in self.mire.dots],
            "blobs": [blob.to_json() for blob in self.last_blobs],
            "expected_projected_points": expected_points,
            "matches": [match.to_json() for match in square_matches],
            "result": result,
        }
        records.append(record)

        if result.get("valid"):
            self.append_status(
                f"Carre {self.square_validation_index + 1}/{len(SQUARE_SEQUENCE)} "
                f"({variant['label']}): rms {result['rms_px']:.2f} px, "
                f"max {result['max_px']:.2f} px, distance {result['distance_norm_mm']:.0f} mm."
            )
            if spacing_error is not None:
                self.append_status(
                    f"  Cote mesure {result['spacing_measured_mean_mm']:.2f} mm "
                    f"vs reel {result['spacing_true_mean_mm']:.2f} mm "
                    f"(erreur moy {spacing_error:.2f} mm)."
                )
        else:
            self.append_status(
                f"Carre {self.square_validation_index + 1}/{len(SQUARE_SEQUENCE)} "
                f"({variant['label']}) echoue: {result.get('reason')}"
            )

        if result.get("valid"):
            spacing_text = (
                f" spacing {float(spacing_error):.1f}mm"
                if spacing_error is not None
                else ""
            )
            banner = (
                f"SQUARE {self.square_validation_index + 1}/{len(SQUARE_SEQUENCE)} "
                f"rms {result['rms_px']:.2f}px{spacing_text}"
            )
            color = (0, 255, 0)
        else:
            banner = (
                f"SQUARE {self.square_validation_index + 1}/{len(SQUARE_SEQUENCE)} "
                f"failed"
            )
            color = (0, 180, 255)
        overlay = draw_preview_banner(overlay.copy(), banner, color)
        self.preview_label.setPixmap(pixmap_from_bgr(overlay, self.preview_label.size()))

        if self.square_validation_index + 1 < len(SQUARE_SEQUENCE):
            self.square_validation_index += 1
            QtCore.QTimer.singleShot(
                300,
                lambda: self._start_capture(
                    test_mode=False,
                    square_phase="validation",
                    duration_ms=self.square_capture_duration_ms(),
                ),
            )
            return

        self.export_square_sequence_report(overlay)

    def square_aggregate_result(self, records: Sequence[Dict[str, object]]) -> Dict[str, object]:
        errors: List[float] = []
        spacing_errors: List[float] = []
        valid_count = 0
        for record in records:
            result = record.get("result", {})
            if not isinstance(result, dict) or not result.get("valid"):
                continue
            valid_count += 1
            for match in record.get("matches", []):
                if isinstance(match, dict):
                    value = match.get("reprojection_error_px")
                    if value is not None:
                        errors.append(float(value))
            spacing_error = result.get("spacing_mean_abs_error_mm")
            if spacing_error is not None:
                spacing_errors.append(float(spacing_error))

        aggregate: Dict[str, object] = {
            "valid": valid_count == len(records) and len(records) > 0,
            "square_count": len(records),
            "valid_square_count": valid_count,
            "point_count": len(errors),
            "rms_px": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            "mean_px": float(np.mean(errors)) if errors else None,
            "max_px": float(np.max(errors)) if errors else None,
            "spacing_mean_abs_error_mm": float(np.mean(spacing_errors)) if spacing_errors else None,
            "spacing_max_abs_error_mm": float(np.max(spacing_errors)) if spacing_errors else None,
        }
        context = self.square_test_context or {}
        pose_result = context.get("pose_result", {})
        if isinstance(pose_result, dict):
            for key in ("distance_z_mm", "distance_norm_mm", "tilt_deg"):
                if key in pose_result:
                    aggregate[key] = pose_result[key]
        measured_mm = self.measured_distance_mm()
        if measured_mm is not None and aggregate.get("distance_norm_mm") is not None:
            aggregate["measured_distance_mm"] = measured_mm
            aggregate["distance_error_percent"] = (
                100.0 * (float(aggregate["distance_norm_mm"]) - measured_mm) / measured_mm
            )
        return aggregate

    def export_square_sequence_report(self, overlay: np.ndarray) -> None:
        context = self.square_test_context
        if context is None:
            self.square_phase = None
            self.set_capture_buttons_enabled(True)
            self.append_status("Test carre abandonne: contexte final manquant.")
            return

        camera_matrix = context["camera_matrix"]
        dist_coeffs = context["dist_coeffs"]
        pose_matches = context.get("pose_matches", [])
        records = context.get("square_results", [])
        assert isinstance(camera_matrix, np.ndarray)
        assert isinstance(dist_coeffs, np.ndarray)
        assert isinstance(pose_matches, list)
        assert isinstance(records, list)
        aggregate = self.square_aggregate_result(records)
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        json_path = output_dir / f"square_test_{stamp}.json"
        png_path = output_dir / f"square_test_{stamp}.png"

        rms_text = (
            f"rms {float(aggregate['rms_px']):.2f}px"
            if aggregate.get("rms_px") is not None
            else "rms n/a"
        )
        spacing_text = ""
        if aggregate.get("spacing_mean_abs_error_mm") is not None:
            spacing_text = f" spacing {float(aggregate['spacing_mean_abs_error_mm']):.1f}mm"
        distance_text = ""
        if aggregate.get("distance_norm_mm") is not None:
            distance_text = f" dist {float(aggregate['distance_norm_mm']):.0f}mm"
        banner = (
            f"SQUARE TEST valid {aggregate['valid_square_count']}/{aggregate['square_count']} "
            f"{rms_text}{spacing_text}{distance_text}"
        )
        overlay = draw_preview_banner(overlay.copy(), banner, (0, 255, 0))
        self.preview_label.setPixmap(pixmap_from_bgr(overlay, self.preview_label.size()))

        selected = context["calibration_file"]
        node_name = context["calibration_node"]
        monitor = self.selected_monitor()
        payload = {
            "created_at": datetime.now().isoformat(timespec="milliseconds"),
            "calibration_file": str(selected),
            "calibration_node": node_name,
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.tolist(),
            "monitor": monitor.to_json() if monitor is not None else None,
            "phase_pose": {
                "duration_ms_measured": context.get("pose_elapsed_ms"),
                "events_accumulated": context.get("pose_event_count"),
                "layout": context.get("pose_layout"),
                "matches": [match.to_json() for match in pose_matches],
                "result": context.get("pose_result"),
            },
            "square_sequence": context.get("square_sequence", []),
            "phase_squares": records,
            "aggregate_result": aggregate,
            "files": {"overlay_png": str(png_path)},
        }
        with json_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        cv.imwrite(str(png_path), overlay)
        self.last_export_paths = [json_path, png_path]
        self.append_status(
            f"Rapport test carre: {json_path} "
            f"({aggregate['valid_square_count']}/{aggregate['square_count']} carres valides)"
        )

        self.square_phase = None
        self.square_test_context = None
        self.square_validation_index = 0
        self.mire.set_pattern("mire")
        self.set_capture_buttons_enabled(True)

    def export_observation(self, overlay: np.ndarray, elapsed_ms: float) -> None:
        monitor = self.selected_monitor()
        if monitor is None:
            return
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        json_path = output_dir / f"mire_observation_{stamp}.json"
        png_path = output_dir / f"mire_overlay_{stamp}.png"

        payload = {
            "created_at": datetime.now().isoformat(timespec="milliseconds"),
            "camera": {
                "resolution_px": {
                    "width": int(self.activity.shape[1]) if self.activity is not None else 0,
                    "height": int(self.activity.shape[0]) if self.activity is not None else 0,
                },
                "events_accumulated": self.event_count,
            },
            "monitor": monitor.to_json(),
            "mire": {
                "rows": ROWS,
                "cols": COLS,
                "missing_dot": {"row": MISSING_DOT[0], "col": MISSING_DOT[1]},
                "anchor_dot": {"row": ANCHOR_DOT[0], "col": ANCHOR_DOT[1]},
                "render": {
                    "blink_hz": self.args.blink_hz,
                    "gradient_softness_percent": self.args.gradient_softness,
                },
                "layout": self.mire.layout_meta,
                "dots": [dot.to_json() for dot in self.mire.dots],
            },
            "accumulation": {
                "duration_ms_requested": self.args.accum_ms,
                "duration_ms_measured": elapsed_ms,
                "blink_hz": self.args.blink_hz,
                "polarity_mode": "positive_count + negative_count",
            },
            "detection": {
                "expected_dots": EXPECTED_DOTS,
                "min_matched": self.args.min_matched,
                "blob_count": len(self.last_blobs),
                "matched_count": len(self.last_matches),
                "blobs": [blob.to_json() for blob in self.last_blobs],
                "matches": [match.to_json() for match in self.last_matches],
            },
            "files": {"overlay_png": str(png_path)},
        }
        with json_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        cv.imwrite(str(png_path), overlay)
        self.last_export_paths = [json_path, png_path]
        self.append_status(f"Export: {json_path}")
        self.append_status(f"Overlay: {png_path}")

    def populate_calibration_files(self) -> None:
        previous = self.selected_calibration_path()
        search_dirs = [Path(self.args.output_dir), Path(".")]
        files = find_calibration_xml_files(search_dirs)
        self.calib_file_combo.blockSignals(True)
        self.calib_file_combo.clear()
        for path in files:
            self.calib_file_combo.addItem(path.name, str(path))
        if previous is not None:
            index = self.calib_file_combo.findData(str(previous))
            if index >= 0:
                self.calib_file_combo.setCurrentIndex(index)
        self.calib_file_combo.blockSignals(False)
        if not files:
            self.calib_file_combo.addItem("Aucune calibration XML trouvee", None)

    def selected_calibration_path(self) -> Optional[Path]:
        data = self.calib_file_combo.currentData() if hasattr(self, "calib_file_combo") else None
        return Path(data) if data else None

    def measured_distance_mm(self) -> Optional[float]:
        text = self.measured_distance_edit.text().strip().replace(",", ".")
        if not text:
            return None
        try:
            value = float(text)
        except ValueError:
            self.append_status(f"Distance mesuree invalide: '{text}' (attendu: mm).")
            return None
        return value if value > 0.0 else None

    def run_calibration_test(self, overlay: np.ndarray, elapsed_ms: float) -> None:
        selected = self.selected_calibration_path()
        if selected is None:
            self.append_status("Test impossible: aucune calibration selectionnee.")
            return
        try:
            camera_matrix, dist_coeffs, node_name = load_calibration_xml(selected)
        except (RuntimeError, cv.error) as exc:
            self.append_status(f"Lecture calibration impossible: {exc}")
            return

        result = evaluate_calibration_on_matches(self.last_matches, camera_matrix, dist_coeffs)
        if not result.get("valid"):
            self.append_status(f"Test echoue: {result.get('reason')}")
            return

        measured_mm = self.measured_distance_mm()
        fx = float(camera_matrix[0, 0])
        fy = float(camera_matrix[1, 1])
        self.append_status(
            f"TEST {selected.name} ({node_name}) fx={fx:.1f} fy={fy:.1f} | "
            f"{result['point_count']} points"
        )
        self.append_status(
            f"  Reprojection: rms {result['rms_px']:.2f} px, max {result['max_px']:.2f} px | "
            f"points tenus a l'ecart ({result['heldout_count']}): "
            + (
                f"moy {result['heldout_mean_px']:.2f} px, max {result['heldout_max_px']:.2f} px"
                if result["heldout_mean_px"] is not None
                else "indisponible"
            )
        )
        if result["spacing_measured_mean_mm"] is not None:
            self.append_status(
                f"  Espacement mire: mesure {result['spacing_measured_mean_mm']:.2f} mm "
                f"vs reel {result['spacing_true_mean_mm']:.2f} mm "
                f"(erreur moy {result['spacing_mean_abs_error_mm']:.2f} mm"
                + (
                    f", {result['spacing_mean_error_percent']:+.2f} %)"
                    if "spacing_mean_error_percent" in result
                    else ")"
                )
            )
        distance_line = (
            f"  Distance camera-ecran estimee: {result['distance_norm_mm']:.0f} mm "
            f"(z={result['distance_z_mm']:.0f} mm, tilt {result['tilt_deg']:.1f} deg)"
        )
        if measured_mm is not None:
            error_percent = 100.0 * (result["distance_norm_mm"] - measured_mm) / measured_mm
            distance_line += f" | mesuree {measured_mm:.0f} mm -> erreur {error_percent:+.1f} %"
            result["measured_distance_mm"] = measured_mm
            result["distance_error_percent"] = error_percent
        self.append_status(distance_line)

        # Compare every other available calibration on the same capture.
        comparisons: List[Dict[str, object]] = []
        for path in find_calibration_xml_files([Path(self.args.output_dir), Path(".")]):
            if path.resolve() == selected.resolve():
                continue
            try:
                other_matrix, other_dist, other_name = load_calibration_xml(path)
            except (RuntimeError, cv.error):
                continue
            other = evaluate_calibration_on_matches(self.last_matches, other_matrix, other_dist)
            if not other.get("valid"):
                continue
            other["file"] = str(path)
            other["node"] = other_name
            other["fx"] = float(other_matrix[0, 0])
            comparisons.append(other)
            line = (
                f"  Comparaison {path.name}: rms {other['rms_px']:.2f} px, "
                f"distance {other['distance_norm_mm']:.0f} mm"
            )
            if measured_mm is not None:
                other_error = 100.0 * (other["distance_norm_mm"] - measured_mm) / measured_mm
                line += f" (erreur {other_error:+.1f} %)"
            self.append_status(line)

        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        json_path = output_dir / f"calibration_test_{stamp}.json"
        png_path = output_dir / f"calibration_test_{stamp}.png"
        banner = (
            f"TEST {selected.name} rms {result['rms_px']:.2f}px "
            f"dist {result['distance_norm_mm']:.0f}mm"
        )
        overlay = draw_preview_banner(overlay.copy(), banner, (0, 255, 0))
        self.preview_label.setPixmap(pixmap_from_bgr(overlay, self.preview_label.size()))

        monitor = self.selected_monitor()
        payload = {
            "created_at": datetime.now().isoformat(timespec="milliseconds"),
            "calibration_file": str(selected),
            "calibration_node": node_name,
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.tolist(),
            "monitor": monitor.to_json() if monitor is not None else None,
            "accumulation": {
                "duration_ms_requested": self.args.accum_ms,
                "duration_ms_measured": elapsed_ms,
                "blink_hz": self.args.blink_hz,
                "events_accumulated": self.event_count,
            },
            "matches": [match.to_json() for match in self.last_matches],
            "result": result,
            "comparisons": comparisons,
            "files": {"overlay_png": str(png_path)},
        }
        with json_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        cv.imwrite(str(png_path), overlay)
        self.last_export_paths = [json_path, png_path]
        self.append_status(f"Rapport de test: {json_path}")

    def erase_current(self) -> None:
        self.accumulating = False
        self.set_capture_buttons_enabled(True)
        self.square_phase = None
        self.square_test_context = None
        self.square_validation_index = 0
        if self.camera is not None:
            self.activity = np.zeros((self.camera.height, self.camera.width), dtype=np.float32)
        self.event_count = 0
        self.last_blobs = []
        self.last_matches = []
        self.preview_blobs = []
        self.last_preview_blob_update = 0.0
        for path in self.last_export_paths:
            try:
                path.unlink()
                self.append_status(f"Supprime: {path}")
            except FileNotFoundError:
                pass
            except OSError as exc:
                self.append_status(f"Impossible de supprimer {path}: {exc}")
        self.last_export_paths = []
        self.preview_label.clear()
        self.preview_label.setText("Preview camera / accumulation")
        self.append_status("Accumulation courante effacee.")

    def reset_all(self) -> None:
        self.erase_current()
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        self.mire.lit = False
        self.mire.set_pattern("mire")
        self.mire.restart_blink()
        self.mire.update()
        self.set_camera_status("Camera: reset, appuyer sur Reconnecter camera", "warn")
        self.append_status("Reset complet.")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        if self.camera is not None:
            self.camera.close()
        self.mire.close()
        super().closeEvent(event)


def render_synthetic_activity(
    width: int,
    height: int,
    dots: Sequence[ScreenDot],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> np.ndarray:
    activity = np.zeros((height, width), dtype=np.float32)
    object_points = np.array(
        [[dot.object_x_mm, dot.object_y_mm, dot.object_z_mm] for dot in dots],
        dtype=np.float64,
    )
    projected, _ = cv.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    for dot, point in zip(dots, projected.reshape(-1, 2)):
        cx = int(round(float(point[0])))
        cy = int(round(float(point[1])))
        if not (0 <= cx < width and 0 <= cy < height):
            continue
        radius = 11 if dot.anchor else 7
        cv.circle(activity, (cx, cy), radius, 80.0 if dot.anchor else 45.0, -1)
    noise_y = np.random.default_rng(123).integers(0, height, 200)
    noise_x = np.random.default_rng(456).integers(0, width, 200)
    activity[noise_y, noise_x] += 1.0
    return activity


def make_synthetic_activity() -> Tuple[np.ndarray, List[ScreenDot], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    width, height = 640, 480
    dots, _ = build_mire_layout(width, height, 1.0, 1.0)
    camera_matrix = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.zeros(5, dtype=np.float64)
    rvec = np.array([[0.05], [-0.18], [0.03]], dtype=np.float64)
    tvec = np.array([[15.0], [5.0], [680.0]], dtype=np.float64)
    activity = render_synthetic_activity(width, height, dots, camera_matrix, dist_coeffs, rvec, tvec)
    return activity, dots, camera_matrix, dist_coeffs, rvec, tvec


def run_self_test() -> int:
    activity, dots, camera_matrix, dist_coeffs, rvec, tvec = make_synthetic_activity()
    blobs = detect_blobs(activity)
    matches, reason = associate_blobs_to_layout(blobs, dots)
    print(f"synthetic blobs: {len(blobs)}")
    print(f"synthetic matches: {len(matches)} ({reason})")
    if len(dots) != EXPECTED_DOTS:
        print(f"layout failed: {len(dots)} dots")
        return 1
    if len(blobs) != EXPECTED_DOTS:
        print("blob detection failed")
        return 1
    if len(matches) != EXPECTED_DOTS:
        print("association failed")
        return 1
    pose_result = evaluate_calibration_on_matches(matches, camera_matrix, dist_coeffs)
    print(f"synthetic pose valid: {pose_result.get('valid')} rms={pose_result.get('rms_px')}")
    if not pose_result.get("valid"):
        print(f"pose evaluation failed: {pose_result.get('reason')}")
        return 1

    for index, variant in enumerate(SQUARE_SEQUENCE):
        square_dots, _ = build_square_layout(
            640,
            480,
            1.0,
            1.0,
            offset_x=float(variant["offset_x"]),
            offset_y=float(variant["offset_y"]),
            side_scale=float(variant["side_scale"]),
            variant_id=str(variant["id"]),
            variant_label=str(variant["label"]),
        )
        square_activity = render_synthetic_activity(
            640, 480, square_dots, camera_matrix, dist_coeffs, rvec, tvec
        )
        square_blobs = detect_blobs(square_activity, SQUARE_EXPECTED_DOTS)
        square_result, square_matches, _ = evaluate_square_validation(
            matches, square_blobs, square_dots, camera_matrix, dist_coeffs
        )
        print(f"synthetic square {index + 1} blobs: {len(square_blobs)}")
        print(
            f"synthetic square {index + 1} matches: {len(square_matches)} "
            f"rms={square_result.get('rms_px')} ({square_result.get('reason', 'ok')})"
        )
        if len(square_blobs) != SQUARE_EXPECTED_DOTS:
            print("square blob detection failed")
            return 1
        if len(square_matches) != SQUARE_EXPECTED_DOTS or not square_result.get("valid"):
            print("square validation failed")
            return 1
        if float(square_result["rms_px"]) > 1.5:
            print("square validation error too high")
            return 1
    print("self-test ok")
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-monitors", action="store_true", help="List detected monitors and exit.")
    parser.add_argument("--monitor", help="Monitor name or index for the fullscreen target.")
    parser.add_argument("--screen-width-mm", type=float, help="Manual active display width in mm.")
    parser.add_argument("--screen-height-mm", type=float, help="Manual active display height in mm.")
    parser.add_argument("--blink-hz", type=float, default=6.0, help="Mire blink frequency.")
    parser.add_argument("--accum-ms", type=int, default=240, help="Event accumulation duration in ms.")
    parser.add_argument(
        "--gradient-softness",
        type=int,
        default=55,
        help="Mire radial gradient softness from 0 hard edge to 100 soft fade.",
    )
    parser.add_argument(
        "--output-dir",
        default="recordings/mire_calibration",
        help="Directory for JSON observations and PNG overlays.",
    )
    parser.add_argument(
        "--min-matched",
        type=int,
        default=EXPECTED_DOTS,
        help="Minimum associated centers required before export.",
    )
    parser.add_argument("--self-test", action="store_true", help="Run synthetic blob/layout tests and exit.")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return run_self_test()

    app = QtWidgets.QApplication(sys.argv[:1])
    monitors = detect_monitors(app)
    if args.list_monitors:
        selected_idx = select_monitor(monitors, args.monitor)
        for idx, monitor in enumerate(monitors):
            shown = monitor
            if idx == selected_idx:
                shown = apply_size_override(monitor, args.screen_width_mm, args.screen_height_mm)
            print(f"{idx}: {shown.label()} (source={shown.source}, offset={shown.x}+{shown.y})")
        return 0

    window = ControlWindow(args, monitors)
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
