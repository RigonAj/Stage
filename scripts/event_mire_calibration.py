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
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, replace
from datetime import datetime, timedelta
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


@dataclass(frozen=True)
class DotGridPattern:
    pattern_id: str
    label: str
    rows: int
    cols: int
    missing_dot: Optional[Tuple[int, int]]
    anchor_dot: Tuple[int, int]

    @property
    def expected_dots(self) -> int:
        missing = 1 if self.missing_dot is not None else 0
        return self.rows * self.cols - missing

    def to_json(self) -> Dict[str, object]:
        return {
            "id": self.pattern_id,
            "label": self.label,
            "type": "dot_grid",
            "rows": self.rows,
            "cols": self.cols,
            "expected_dots": self.expected_dots,
            "missing_dot": (
                {"row": self.missing_dot[0], "col": self.missing_dot[1]}
                if self.missing_dot is not None
                else None
            ),
            "anchor_dot": {"row": self.anchor_dot[0], "col": self.anchor_dot[1]},
        }


DOT_GRID_PATTERNS: List[DotGridPattern] = [
    DotGridPattern("mire", "Asymetrique 5x4 - 19 points", ROWS, COLS, MISSING_DOT, ANCHOR_DOT),
    DotGridPattern("grid_5x4", "Grille complete 5x4 - 20 points", 4, 5, None, (0, 0)),
    DotGridPattern("grid_7x5", "Grille complete 7x5 - 35 points", 5, 7, None, (0, 0)),
    DotGridPattern("grid_9x6", "Grille complete 9x6 - 54 points", 6, 9, None, (0, 0)),
]
DOT_GRID_PATTERN_BY_ID = {pattern.pattern_id: pattern for pattern in DOT_GRID_PATTERNS}
DEFAULT_PATTERN_ID = "mire"
SQUARE_SEQUENCE = [
    {"id": "center_large", "label": "centre grand", "offset_x": 0.0, "offset_y": 0.0, "side_scale": 2.0},
    {"id": "upper_left_medium", "label": "haut gauche moyen", "offset_x": -0.65, "offset_y": -0.45, "side_scale": 1.35},
    {"id": "upper_right_small", "label": "haut droite petit", "offset_x": 0.75, "offset_y": -0.25, "side_scale": 1.10},
    {"id": "lower_center_medium", "label": "bas centre moyen", "offset_x": 0.20, "offset_y": 0.65, "side_scale": 1.55},
]

PRESET_COLORS: List[Tuple[str, Tuple[int, int, int]]] = [
    ("Noir", (0, 0, 0)),
    ("Blanc", (255, 255, 255)),
    ("Vert", (0, 255, 0)),
    ("Rouge", (0, 0, 255)),
    ("Bleu", (255, 0, 0)),
    ("Jaune", (0, 255, 255)),
    ("Cyan", (255, 255, 0)),
    ("Magenta", (255, 0, 255)),
    ("Orange", (0, 165, 255)),
    ("Gris clair", (200, 200, 200)),
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
    center_method: str = "unknown"
    center_agreement_px: Optional[float] = None

    def to_json(self) -> Dict[str, object]:
        return {
            "index": self.index,
            "center_px": {"x": self.x, "y": self.y},
            "center_method": self.center_method,
            "center_agreement_px": self.center_agreement_px,
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


@dataclass
class EventFilterSnapshot:
    enabled: bool
    name: str
    support_duration_us: int
    cutoff_hz: float
    incoming_events: int
    outgoing_events: int
    error: Optional[str] = None

    @property
    def reduction_factor(self) -> float:
        if self.incoming_events <= 0:
            return 0.0
        discarded = max(0, self.incoming_events - self.outgoing_events)
        return discarded / float(self.incoming_events)

    def to_json(self) -> Dict[str, object]:
        return {
            "enabled": self.enabled,
            "name": self.name,
            "support_duration_us": self.support_duration_us,
            "cutoff_hz": self.cutoff_hz,
            "incoming_events": self.incoming_events,
            "outgoing_events": self.outgoing_events,
            "reduction_factor": self.reduction_factor,
            "error": self.error,
        }

    def summary(self) -> str:
        if not self.enabled:
            return "filtre bruit off"
        if self.incoming_events <= 0:
            return "filtre bruit 0 event"
        return (
            f"filtre bruit {self.outgoing_events}/{self.incoming_events} events "
            f"(-{100.0 * self.reduction_factor:.1f}%)"
        )


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


def pattern_by_id(pattern_id: str) -> DotGridPattern:
    return DOT_GRID_PATTERN_BY_ID.get(pattern_id, DOT_GRID_PATTERN_BY_ID[DEFAULT_PATTERN_ID])


def build_dot_grid_layout(
    width_px: int,
    height_px: int,
    mm_per_px_x: float,
    mm_per_px_y: float,
    pattern: DotGridPattern,
) -> Tuple[List[ScreenDot], Dict[str, object]]:
    spacing_px = 0.82 * min(
        width_px / float(max(1, pattern.cols - 1)),
        height_px / float(max(1, pattern.rows - 1)),
    )
    center_x = width_px * 0.5
    center_y = height_px * 0.5
    small_radius = spacing_px * 0.17
    anchor_radius = spacing_px * 0.285

    dots: List[ScreenDot] = []
    for row in range(pattern.rows):
        for col in range(pattern.cols):
            if pattern.missing_dot is not None and (row, col) == pattern.missing_dot:
                continue
            x = center_x + (col - (pattern.cols - 1) * 0.5) * spacing_px
            y = center_y + (row - (pattern.rows - 1) * 0.5) * spacing_px
            anchor = (row, col) == pattern.anchor_dot
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
        "pattern": pattern.pattern_id,
        "pattern_type": "dot_grid",
        "pattern_label": pattern.label,
        "rows": pattern.rows,
        "cols": pattern.cols,
        "expected_dots": pattern.expected_dots,
        "missing_dot": (
            {"row": pattern.missing_dot[0], "col": pattern.missing_dot[1]}
            if pattern.missing_dot is not None
            else None
        ),
        "anchor_dot": {"row": pattern.anchor_dot[0], "col": pattern.anchor_dot[1]},
        "spacing_px": spacing_px,
        "spacing_x_mm": spacing_px * mm_per_px_x,
        "spacing_y_mm": spacing_px * mm_per_px_y,
        "center_x_px": center_x,
        "center_y_px": center_y,
        "small_radius_px": small_radius,
        "anchor_radius_px": anchor_radius,
    }
    return dots, meta


def build_mire_layout(
    width_px: int,
    height_px: int,
    mm_per_px_x: float,
    mm_per_px_y: float,
    pattern_id: str = DEFAULT_PATTERN_ID,
) -> Tuple[List[ScreenDot], Dict[str, object]]:
    return build_dot_grid_layout(
        width_px,
        height_px,
        mm_per_px_x,
        mm_per_px_y,
        pattern_by_id(pattern_id),
    )


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
) -> Tuple[List[ScreenDot], Dict[str, object]]:
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


def robust_weighted_center(
    yy: np.ndarray,
    xx: np.ndarray,
    weights: np.ndarray,
    origin_x: int,
    origin_y: int,
    fallback: Tuple[float, float],
) -> Tuple[Tuple[float, float], bool]:
    if weights.size == 0:
        return fallback, False

    weights_f = weights.astype(np.float64, copy=False)
    floor = float(np.percentile(weights_f, 20))
    ceiling = float(np.percentile(weights_f, 95))
    robust = np.clip(weights_f - floor, 0.0, max(ceiling - floor, 1e-6))
    total = float(np.sum(robust))
    if total <= 1e-6:
        total = float(np.sum(weights_f))
        if total <= 1e-6:
            return fallback, False
        robust = weights_f

    cx = float(np.sum((xx + origin_x) * robust) / total)
    cy = float(np.sum((yy + origin_y) * robust) / total)
    return (cx, cy), True


def shape_center_from_mask(
    mask: np.ndarray,
    origin_x: int,
    origin_y: int,
    fallback: Tuple[float, float],
) -> Tuple[Tuple[float, float], str]:
    mask_u8 = mask.astype(np.uint8, copy=False)
    contours, _ = cv.findContours(mask_u8 * 255, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if contours:
        contour = max(contours, key=cv.contourArea)
        if len(contour) >= 5 and cv.contourArea(contour) >= 3.0:
            try:
                (cx, cy), _, _ = cv.fitEllipse(contour)
                center = (float(cx + origin_x), float(cy + origin_y))
                if all(math.isfinite(value) for value in center):
                    return center, "ellipse"
            except cv.error:
                pass

        moments = cv.moments(contour)
        if abs(moments["m00"]) > 1e-6:
            return (
                (
                    float(moments["m10"] / moments["m00"] + origin_x),
                    float(moments["m01"] / moments["m00"] + origin_y),
                ),
                "contour_moments",
            )

    moments = cv.moments(mask_u8)
    if abs(moments["m00"]) > 1e-6:
        return (
            (
                float(moments["m10"] / moments["m00"] + origin_x),
                float(moments["m01"] / moments["m00"] + origin_y),
            ),
            "binary_moments",
        )
    return fallback, "component_centroid"


def estimate_blob_center(
    mask: np.ndarray,
    activity_roi: np.ndarray,
    component_centroid: Tuple[float, float],
    origin_x: int,
    origin_y: int,
) -> Tuple[float, float, str, Optional[float]]:
    yy, xx = np.nonzero(mask)
    weights = activity_roi[mask]
    weighted_center, has_weighted = robust_weighted_center(
        yy,
        xx,
        weights,
        origin_x,
        origin_y,
        component_centroid,
    )
    shape_center, shape_method = shape_center_from_mask(
        mask,
        origin_x,
        origin_y,
        component_centroid,
    )

    if not has_weighted:
        return shape_center[0], shape_center[1], shape_method, None

    agreement = float(np.linalg.norm(np.subtract(shape_center, weighted_center)))
    max_reasonable_delta = max(2.0, 0.12 * max(mask.shape))
    if agreement <= max_reasonable_delta:
        # The component shape carries the target geometry; the activity centroid
        # adds a small sub-pixel correction without letting one bright side pull
        # the center too far.
        blend_shape = 0.85
        cx = blend_shape * shape_center[0] + (1.0 - blend_shape) * weighted_center[0]
        cy = blend_shape * shape_center[1] + (1.0 - blend_shape) * weighted_center[1]
        return cx, cy, f"{shape_method}+weighted", agreement

    return shape_center[0], shape_center[1], shape_method, agreement


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
        activity_roi = activity[y : y + h, x : x + w]
        weights = activity_roi[mask]
        if weights.size == 0:
            continue
        total = float(np.sum(weights))
        cx, cy, center_method, center_agreement = estimate_blob_center(
            mask,
            activity_roi,
            (float(centroids[label][0]), float(centroids[label][1])),
            x,
            y,
        )
        blobs.append(
            Blob(
                index=len(blobs),
                x=cx,
                y=cy,
                area_px=area,
                weight=total,
                peak=float(np.max(weights)),
                bbox=(x, y, w, h),
                center_method=center_method,
                center_agreement_px=center_agreement,
            )
        )

    blobs.sort(key=lambda blob: blob.weight, reverse=True)
    if len(blobs) > expected:
        blobs = blobs[:expected]
    for idx, blob in enumerate(blobs):
        blob.index = idx
    return blobs


def grid_dimensions_from_dots(dots: Sequence[ScreenDot]) -> Tuple[int, int]:
    if not dots:
        return 0, 0
    rows = max(dot.row for dot in dots) + 1
    cols = max(dot.col for dot in dots) + 1
    return rows, cols


def _apply_homography(points: np.ndarray, homography: np.ndarray) -> np.ndarray:
    """Map an (N, 2) array of points through a 3x3 homography."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    mapped = cv.perspectiveTransform(pts, homography)
    return mapped.reshape(-1, 2)


def grid_corner_cycle(points: np.ndarray) -> List[int]:
    """Indices of the four grid corners, ordered around the rectangle.

    Ranking is done in the point cloud's own principal-axis frame, so the result
    is stable under arbitrary in-plane rotation of the grid in the image.
    """
    centered = points - points.mean(axis=0)
    covariance = centered.T @ centered
    _, eigvecs = np.linalg.eigh(covariance)
    axes = eigvecs[:, ::-1].T  # major principal axis first
    projected = centered @ axes.T
    u = projected[:, 0]
    v = projected[:, 1]
    return [
        int(np.argmin(u + v)),  # (min u, min v)
        int(np.argmax(u - v)),  # (max u, min v)
        int(np.argmax(u + v)),  # (max u, max v)
        int(np.argmin(u - v)),  # (min u, max v)
    ]


def polygon_area(points: np.ndarray, indices: Sequence[int]) -> float:
    polygon = points[list(indices)]
    x = polygon[:, 0]
    y = polygon[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def order_cycle_around_centroid(points: np.ndarray, indices: Sequence[int]) -> List[int]:
    coords = points[list(indices)]
    center = np.mean(coords, axis=0)
    angles = np.arctan2(coords[:, 1] - center[1], coords[:, 0] - center[0])
    return [int(idx) for _, idx in sorted(zip(angles.tolist(), indices))]


def is_convex_quadrilateral(points: np.ndarray, indices: Sequence[int]) -> bool:
    if len(indices) != 4 or len(set(indices)) != 4:
        return False
    polygon = points[list(indices)]
    cross_values = []
    for idx in range(4):
        a = polygon[idx]
        b = polygon[(idx + 1) % 4]
        c = polygon[(idx + 2) % 4]
        ab = b - a
        bc = c - b
        cross_values.append(float(ab[0] * bc[1] - ab[1] * bc[0]))
    nonzero = [value for value in cross_values if abs(value) > 1e-9]
    if len(nonzero) < 4:
        return False
    return all(value > 0.0 for value in nonzero) or all(value < 0.0 for value in nonzero)


def corner_candidate_cycles(points: np.ndarray) -> List[List[int]]:
    """Return plausible four-corner cycles for a projected dot grid.

    A single PCA min/max cycle can collapse to duplicate corners on real
    trapezoidal views, especially when the grid is rolled and one side is
    compressed. The convex hull gives a stronger set of candidates while still
    staying small for clean blob detections.
    """
    if len(points) < 4:
        return []

    cycles: List[List[int]] = []
    seen: set = set()

    def add_cycle(indices: Sequence[int]) -> None:
        if len(indices) != 4 or len(set(indices)) != 4:
            return
        ordered = order_cycle_around_centroid(points, indices)
        if not is_convex_quadrilateral(points, ordered):
            return
        if polygon_area(points, ordered) <= 1e-6:
            return
        key = tuple(sorted(int(idx) for idx in ordered))
        if key in seen:
            return
        seen.add(key)
        cycles.append(ordered)

    add_cycle(grid_corner_cycle(points))

    hull = cv.convexHull(points.astype(np.float32), returnPoints=False)
    if hull is not None:
        hull_indices = [int(idx) for idx in hull.reshape(-1).tolist()]
        if len(hull_indices) == 4:
            add_cycle(hull_indices)
        elif len(hull_indices) > 4:
            # A clean grid often has extra hull vertices along the same border.
            # Trying all hull quadrilaterals is cheap here and avoids relying on
            # one brittle definition of "the" four extremes.
            max_hull = 16
            if len(hull_indices) > max_hull:
                step = len(hull_indices) / float(max_hull)
                sampled = [
                    hull_indices[int(round(i * step)) % len(hull_indices)]
                    for i in range(max_hull)
                ]
                hull_indices = sorted(set(sampled), key=hull_indices.index)
            for combo in itertools.combinations(hull_indices, 4):
                add_cycle(combo)

    cycles.sort(key=lambda cycle: polygon_area(points, cycle), reverse=True)
    return cycles


def median_neighbor_spacing(points: np.ndarray) -> float:
    """Median nearest-neighbor distance, used to gate assignment distances."""
    if len(points) < 2:
        return 0.0
    diff = points[:, None, :] - points[None, :, :]
    dist = np.linalg.norm(diff, axis=2)
    np.fill_diagonal(dist, np.inf)
    return float(np.median(np.min(dist, axis=1)))


def assign_dots_to_blobs(
    projected: np.ndarray, blob_points: np.ndarray, gate: float
) -> Optional[List[int]]:
    """Greedy unique nearest-neighbor match of projected dots to blobs.

    Returns dot_index -> blob_index, or None if any dot has no free blob within
    ``gate`` pixels (which rejects a wrong grid orientation).
    """
    dist = np.linalg.norm(projected[:, None, :] - blob_points[None, :, :], axis=2)
    n_dots, n_blobs = dist.shape
    pairs = [
        (float(dist[i, j]), i, j)
        for i in range(n_dots)
        for j in range(n_blobs)
        if dist[i, j] <= gate
    ]
    pairs.sort()
    dot_to_blob = [-1] * n_dots
    used_dots: set = set()
    used_blobs: set = set()
    for _, i, j in pairs:
        if i in used_dots or j in used_blobs:
            continue
        dot_to_blob[i] = j
        used_dots.add(i)
        used_blobs.add(j)
        if len(used_dots) == n_dots:
            break
    if len(used_dots) != n_dots:
        return None
    return dot_to_blob


def fit_grid_homography(
    obj_points: np.ndarray,
    blob_points: np.ndarray,
    obj_corners: Sequence[int],
    img_corners: Sequence[int],
    max_iterations: int = 6,
) -> Optional[Tuple[List[int], np.ndarray, float]]:
    """Seed a homography from four corner correspondences, then refine by ICP.

    Returns ``(dot_to_blob, homography, rms_px)`` on a full bijection, else None.
    """
    src = obj_points[list(obj_corners)].astype(np.float32)
    dst = blob_points[list(img_corners)].astype(np.float32)
    try:
        homography = cv.getPerspectiveTransform(src, dst)
    except cv.error:
        return None
    if homography is None or not np.all(np.isfinite(homography)):
        return None

    assignment: Optional[List[int]] = None
    previous: Optional[List[int]] = None
    for _ in range(max_iterations):
        projected = _apply_homography(obj_points, homography)
        spacing = median_neighbor_spacing(projected)
        if spacing <= 0.0:
            return None
        assignment = assign_dots_to_blobs(projected, blob_points, 0.75 * spacing)
        if assignment is None:
            return None
        if assignment == previous:
            break
        previous = assignment
        refined, _ = cv.findHomography(obj_points, blob_points[assignment], method=0)
        if refined is None or not np.all(np.isfinite(refined)):
            break
        homography = refined

    if assignment is None:
        return None
    projected = _apply_homography(obj_points, homography)
    residuals = np.linalg.norm(projected - blob_points[assignment], axis=1)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    return assignment, homography, rms


def associate_blobs_to_layout(blobs: Sequence[Blob], dots: Sequence[ScreenDot]) -> Tuple[List[Match], str]:
    """Match detected blobs to known dots, robust to rotation and perspective.

    A dot grid is a planar target, so its projection into the image is an exact
    homography. We seed that homography from the four grid corners, then refine
    it by iterated closest-point assignment. This handles any viewing angle,
    unlike a fixed row/column split of the image axes.
    """
    expected = len(dots)
    rows, _cols = grid_dimensions_from_dots(dots)
    if expected <= 0 or rows <= 0:
        return [], "empty target layout"
    if len(blobs) < expected:
        return [], f"not enough blobs: {len(blobs)}/{expected}"

    blob_points = np.array([[blob.x, blob.y] for blob in blobs], dtype=np.float64)
    obj_points = np.array(
        [[dot.object_x_mm, dot.object_y_mm] for dot in dots], dtype=np.float64
    )

    img_cycles = corner_candidate_cycles(blob_points)
    obj_cycles = corner_candidate_cycles(obj_points)
    if not img_cycles or not obj_cycles:
        return [], "could not isolate grid corners"

    # The anchor dot is drawn larger, so it is the highest-activity blob. It
    # resolves the 180-degree symmetry of a full grid.
    anchor_dot_idx = next(
        (i for i, dot in enumerate(dots) if dot.anchor),
        min(range(len(dots)), key=lambda i: (dots[i].row, dots[i].col)),
    )
    anchor_blob_idx = int(np.argmax([blob.weight for blob in blobs]))

    best: Optional[Tuple[bool, float, List[int], np.ndarray]] = None
    # Pinhole projection preserves orientation, but corner cycles may differ in
    # start corner and handedness: try rotations and reversals for each plausible
    # hull quadrilateral.
    for obj_cycle in obj_cycles:
        for img_cycle in img_cycles:
            for img_order in (img_cycle, img_cycle[::-1]):
                for k in range(4):
                    obj_order = obj_cycle[k:] + obj_cycle[:k]
                    result = fit_grid_homography(obj_points, blob_points, obj_order, img_order)
                    if result is None:
                        continue
                    assignment, homography, rms = result
                    spacing = median_neighbor_spacing(_apply_homography(obj_points, homography))
                    if spacing <= 0.0 or rms > 0.55 * spacing:
                        continue
                    anchor_ok = assignment[anchor_dot_idx] == anchor_blob_idx
                    candidate = (not anchor_ok, rms, assignment, homography)
                    if best is None or candidate[:2] < best[:2]:
                        best = candidate

    if best is None:
        return [], "could not fit grid homography"

    _, _, assignment, homography = best
    projected = _apply_homography(obj_points, homography)
    matches: List[Match] = []
    for dot_idx, blob_idx in enumerate(assignment):
        error = float(np.linalg.norm(projected[dot_idx] - blob_points[blob_idx]))
        matches.append(Match(dot=dots[dot_idx], blob=blobs[blob_idx], reproj_error_px=error))

    if len(matches) != expected:
        return [], f"matched {len(matches)}/{expected}"
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
    rows, cols = grid_dimensions_from_dots([match.dot for match in matches])
    border = [
        i for i, m in enumerate(matches)
        if m.dot.row in (0, rows - 1) or m.dot.col in (0, cols - 1)
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


def normalize_activity_to_bgr(
    activity: np.ndarray,
    color: Tuple[int, int, int] = (255, 255, 255),
    background_color: Tuple[int, int, int] = (0, 0, 0),
    gamma: float = 0.5,
) -> np.ndarray:
    h, w = activity.shape if activity.ndim == 2 else (480, 640)
    max_value = float(np.max(activity)) if activity.size > 0 else 0.0
    if max_value <= 0.0:
        canvas = np.empty((h, w, 3), dtype=np.uint8)
        canvas[:, :] = background_color
        return canvas

    # Normalize against a high percentile rather than the single hottest pixel so
    # that a whole blob reaches the full chosen color (true white, etc.) instead
    # of only a one-pixel peak fading into the background.
    nonzero = activity[activity > 0.0]
    robust_max = max(float(np.percentile(nonzero, 92)), 1e-6)
    ratio = np.clip(activity / robust_max, 0.0, 1.0)
    boosted = np.power(ratio, gamma, dtype=np.float32)[:, :, None]
    background_arr = np.array(background_color, dtype=np.float32)[None, None, :]
    color_arr = np.array(color, dtype=np.float32)[None, None, :]
    blended = background_arr * (1.0 - boosted) + color_arr * boosted
    return np.clip(blended, 0, 255).astype(np.uint8)


def make_overlay(
    activity: np.ndarray,
    blobs: Sequence[Blob],
    matches: Sequence[Match],
    event_color: Tuple[int, int, int] = (255, 255, 255),
    marker_color: Tuple[int, int, int] = (0, 255, 0),
    background_color: Tuple[int, int, int] = (0, 0, 0),
) -> np.ndarray:
    image = normalize_activity_to_bgr(activity, event_color, background_color)
    draw_blob_indicators(image, blobs, matches, color=marker_color)
    return image


def draw_blob_indicators(
    image: np.ndarray,
    blobs: Sequence[Blob],
    matches: Sequence[Match] = (),
    color: Tuple[int, int, int] = (0, 255, 0),
) -> np.ndarray:
    for blob in blobs:
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


def event_store_size(events: object) -> int:
    size_method = getattr(events, "size", None)
    if callable(size_method):
        return int(size_method())
    try:
        return len(events)  # type: ignore[arg-type]
    except TypeError:
        return 0


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


# ---------------------------------------------------------------------------
# Hand-eye collection support (external phone mire + tf2)
# Plan step 2 of docs/Robot_Control/ur3e_camera_base_calibration.md:
# the mire is the phone screen mounted on tool0, served by serve_phone_mire.py;
# each capture pairs TF base->tool0 with solvePnP camera->mire in one JSON.
# ---------------------------------------------------------------------------

UR_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def rotation_matrix_to_quat_xyzw(rotation: np.ndarray) -> List[float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = [
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        ]
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        quat = [
            0.25 * s,
            (rotation[0, 1] + rotation[1, 0]) / s,
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[2, 1] - rotation[1, 2]) / s,
        ]
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        quat = [
            (rotation[0, 1] + rotation[1, 0]) / s,
            0.25 * s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
        ]
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        quat = [
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            0.25 * s,
            (rotation[1, 0] - rotation[0, 1]) / s,
        ]
    norm = math.sqrt(sum(v * v for v in quat))
    return [v / norm for v in quat]


def quat_angle_deg(quat_a: Sequence[float], quat_b: Sequence[float]) -> float:
    dot = abs(sum(a * b for a, b in zip(quat_a, quat_b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def compute_stationarity(
    tf_start: Tuple[Sequence[float], Sequence[float]],
    tf_end: Tuple[Sequence[float], Sequence[float]],
) -> Dict[str, float]:
    """TF drift between the start and end of the accumulation window."""
    xyz_start, quat_start = tf_start
    xyz_end, quat_end = tf_end
    trans_delta_mm = 1000.0 * math.sqrt(
        sum((a - b) ** 2 for a, b in zip(xyz_start, xyz_end))
    )
    return {
        "trans_delta_mm": trans_delta_mm,
        "rot_delta_deg": quat_angle_deg(quat_start, quat_end),
    }


def fetch_external_layout(
    base_url: str, timeout_s: float = 3.0
) -> Tuple[List[ScreenDot], Dict[str, object]]:
    """Read the layout currently displayed by the phone from serve_phone_mire.py.

    Refuses any layout that is not explicitly landscape and real-fullscreen:
    otherwise the px->mm mapping and screen-center frame can change silently
    between sessions.
    """
    url = base_url.rstrip("/") + "/api/current_layout"
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            payload = json.load(response)
    except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"mire server unreachable at {url}: {exc}") from exc
    if "error" in payload:
        raise RuntimeError(f"mire server: {payload['error']} (ouvrir la page sur le telephone)")
    screen = payload.get("screen", {})
    screen_error = external_layout_screen_error(screen)
    if screen_error is not None:
        raise RuntimeError(screen_error)
    dots = [
        ScreenDot(
            row=int(dot["row"]),
            col=int(dot["col"]),
            anchor=bool(dot["anchor"]),
            screen_x_px=float(dot["screen_px"]["x"]),
            screen_y_px=float(dot["screen_px"]["y"]),
            radius_px=float(dot["radius_px"]),
            object_x_mm=float(dot["object_mm"]["x"]),
            object_y_mm=float(dot["object_mm"]["y"]),
            object_z_mm=float(dot["object_mm"].get("z", 0.0)),
        )
        for dot in payload.get("dots", [])
    ]
    layout = payload.get("layout", {})
    expected = EXPECTED_DOTS
    if isinstance(layout, dict):
        expected = int(layout.get("expected_dots", expected))
    if len(dots) != expected:
        raise RuntimeError(f"layout invalide: {len(dots)} points au lieu de {expected}")
    return dots, payload


def external_layout_screen_error(screen: object) -> Optional[str]:
    """Return why a phone screen geometry is unsafe for hand-eye capture."""
    if not isinstance(screen, dict):
        return "geometrie ecran absente dans le layout de la mire"
    if screen.get("landscape_ok") is not True:
        return (
            "format paysage requis pour la mire telephone "
            f"(viewport {screen.get('viewport_px')}, panneau {screen.get('panel_px')})"
        )
    if screen.get("fullscreen_ok") is not True:
        return (
            "le telephone n'est pas en vrai plein ecran "
            f"(viewport {screen.get('viewport_px')} vs panneau {screen.get('panel_px')})"
        )
    return None


def solve_mire_pose_with_ambiguity(
    matches: Sequence[Match],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Dict[str, object]:
    """solvePnPGeneric(IPPE) keeping the planar-ambiguity error ratio.

    Returns rvec/tvec (mm), reprojection RMS, tilt and the ratio between the
    two IPPE solutions (high ratio = unambiguous; ~1 with low tilt = reject).
    """
    object_points = np.array(
        [[m.dot.object_x_mm, m.dot.object_y_mm, m.dot.object_z_mm] for m in matches],
        dtype=np.float64,
    )
    image_points = np.array([[m.blob.x, m.blob.y] for m in matches], dtype=np.float64)

    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    ambiguity_ratio: Optional[float] = None
    try:
        count, rvecs, tvecs, errors = cv.solvePnPGeneric(
            object_points, image_points, camera_matrix, dist_coeffs,
            flags=cv.SOLVEPNP_IPPE,
        )
    except cv.error:
        count = 0
    if count and len(rvecs) >= 1:
        flat_errors = np.asarray(errors, dtype=np.float64).reshape(-1)
        order = np.argsort(flat_errors)
        rvec = np.asarray(rvecs[order[0]], dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvecs[order[0]], dtype=np.float64).reshape(3, 1)
        if len(order) >= 2 and flat_errors[order[0]] > 1e-9:
            ambiguity_ratio = float(flat_errors[order[1]] / flat_errors[order[0]])
    else:
        ok, rvec, tvec, reason = solve_pose_from_matches(matches, camera_matrix, dist_coeffs)
        if not ok:
            return {"valid": False, "reason": reason}

    projected, _ = cv.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    residuals = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    result: Dict[str, object] = {
        "valid": True,
        "rvec": rvec,
        "tvec": tvec,
        "rms_px": float(np.sqrt(np.mean(residuals**2))),
        "max_px": float(np.max(residuals)),
        "ambiguity_ratio": ambiguity_ratio,
    }
    result.update(pose_summary(rvec, tvec))
    return result


def handeye_rejection_reason(
    stationarity: Dict[str, float],
    matched_dots: int,
    min_matched: int,
    ambiguity_ratio: Optional[float],
    tilt_deg: float,
    trans_limit_mm: float,
    rot_limit_deg: float,
    ambiguity_min_ratio: float,
    ambiguity_min_tilt_deg: float,
) -> Optional[str]:
    """Auto-validation gates of plan section 7; None means the sample is kept."""
    if matched_dots < min_matched:
        return f"associations insuffisantes ({matched_dots}/{min_matched})"
    if stationarity["trans_delta_mm"] > trans_limit_mm or stationarity["rot_delta_deg"] > rot_limit_deg:
        return (
            "robot non immobile pendant l'accumulation "
            f"({stationarity['trans_delta_mm']:.3f} mm / {stationarity['rot_delta_deg']:.4f} deg)"
        )
    if (
        ambiguity_ratio is not None
        and ambiguity_ratio < ambiguity_min_ratio
        and tilt_deg < ambiguity_min_tilt_deg
    ):
        return (
            f"ambiguite planaire IPPE (ratio {ambiguity_ratio:.2f} "
            f"avec tilt {tilt_deg:.1f} deg): incliner davantage l'ecran"
        )
    return None


def build_handeye_sample(
    index: int,
    tf_start: Tuple[Sequence[float], Sequence[float]],
    tf_end: Tuple[Sequence[float], Sequence[float]],
    joint_positions_rad: Optional[Sequence[float]],
    pnp: Dict[str, object],
    matches: Sequence[Match],
) -> Dict[str, object]:
    """One entry of the multi-sample JSON (plan section 5 schema, meters)."""
    xyz, quat = tf_start
    rvec = np.asarray(pnp["rvec"], dtype=np.float64).reshape(3)
    tvec_mm = np.asarray(pnp["tvec"], dtype=np.float64).reshape(3)
    rotation, _ = cv.Rodrigues(np.asarray(pnp["rvec"], dtype=np.float64))
    return {
        "index": index,
        "stamp": datetime.now().isoformat(timespec="milliseconds"),
        "T_base_tool0": {
            "xyz": [float(v) for v in xyz],
            "quat_xyzw": [float(v) for v in quat],
        },
        "T_camera_mire": {
            "xyz": [float(v) / 1000.0 for v in tvec_mm],
            "quat_xyzw": rotation_matrix_to_quat_xyzw(rotation),
            "rvec": [float(v) for v in rvec],
            "tvec_mm": [float(v) for v in tvec_mm],
        },
        "joint_positions_rad": (
            [float(v) for v in joint_positions_rad] if joint_positions_rad else None
        ),
        "stationarity": compute_stationarity(tf_start, tf_end),
        "reproj_rms_px": float(pnp["rms_px"]),
        "matched_dots": len(matches),
        "tilt_deg": float(pnp["tilt_deg"]),
        "ippe_ambiguity_ratio": (
            float(pnp["ambiguity_ratio"]) if pnp.get("ambiguity_ratio") is not None else None
        ),
        "matches": [match.to_json() for match in matches],
    }


class TfPoseReader:
    """Background rclpy node: TF base->tool0 lookups and /joint_states.

    The web-UI stack already publishes TF; only a shared ROS_DOMAIN_ID is
    needed (plan section 9: no custom service, no backend change).
    """

    def __init__(self, base_frame: str, tool_frame: str) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from sensor_msgs.msg import JointState
            from tf2_ros import Buffer, TransformListener
        except ImportError as exc:
            raise RuntimeError(
                "rclpy/tf2_ros indisponibles: sourcer ROS 2 avant de lancer "
                "(source /opt/ros/humble/setup.bash)"
            ) from exc
        self._rclpy = rclpy
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("handeye_collector")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self._joint_lock = threading.Lock()
        self._joint_names: List[str] = []
        self._joint_positions: List[float] = []
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._executor.spin, name="handeye-tf-spin", daemon=True
        )
        self._thread.start()

    def _on_joint_state(self, msg) -> None:
        with self._joint_lock:
            self._joint_names = list(msg.name)
            self._joint_positions = list(msg.position)

    def joint_positions(self) -> Optional[List[float]]:
        """Positions reordered into physical UR order (the driver's
        /joint_states order is NOT canonical: shoulder_lift arrives first)."""
        with self._joint_lock:
            names = list(self._joint_names)
            positions = list(self._joint_positions)
        if not names:
            return None
        by_name = dict(zip(names, positions))
        try:
            return [float(by_name[name]) for name in UR_JOINT_ORDER]
        except KeyError:
            return [float(value) for value in positions]

    def lookup_tool_pose(self) -> Tuple[List[float], List[float]]:
        """Latest TF base->tool0 as (xyz meters, quat xyzw)."""
        from rclpy.time import Time

        transform = self.buffer.lookup_transform(self.base_frame, self.tool_frame, Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            [translation.x, translation.y, translation.z],
            [rotation.x, rotation.y, rotation.z, rotation.w],
        )

    def close(self) -> None:
        try:
            self._executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


class MireWindow(QtWidgets.QWidget):
    calibration_started = QtCore.pyqtSignal()
    calibration_done = QtCore.pyqtSignal()

    def __init__(
        self,
        blink_hz: float,
        gradient_softness: int,
        pattern_id: str = DEFAULT_PATTERN_ID,
    ) -> None:
        super().__init__()
        self.setWindowTitle("Mire calibration")
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
        self.setStyleSheet("background: black;")
        self.monitor = MonitorInfo("unknown", 0, 0, 640, 480, 0.0, 0.0, 0.0, 0.0, 1.0, "none")
        self.dots: List[ScreenDot] = []
        self.layout_meta: Dict[str, object] = {}
        self.pattern = pattern_by_id(pattern_id).pattern_id
        self.square_variant: Dict[str, object] = dict(SQUARE_SEQUENCE[0])
        self.lit = False
        self.blink_hz = blink_hz
        self.gradient_softness = int(np.clip(gradient_softness, 0, 100))
        self.blink_timer = QtCore.QTimer(self)
        self.blink_timer.timeout.connect(self.toggle_lit)
        self.calibration_timer = QtCore.QTimer(self)
        self.calibration_timer.setSingleShot(True)
        self.calibration_timer.timeout.connect(self.finish_calibration_blink)
        self.update_layout()
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
        if pattern != "square4":
            pattern = pattern_by_id(pattern).pattern_id
        if self.pattern == pattern:
            return
        self.pattern = pattern
        self.update_layout()

    def current_dot_grid_pattern(self) -> DotGridPattern:
        return pattern_by_id(self.pattern)

    def expected_dot_count(self) -> int:
        if self.pattern == "square4":
            return SQUARE_EXPECTED_DOTS
        return self.current_dot_grid_pattern().expected_dots

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
                self.pattern,
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


SHORTCUTS_HELP: List[Tuple[str, str, str]] = [
    ("F1", "Aide / raccourcis", "Ouvre cette page d'aide."),
    (
        "F2",
        "Afficher mire",
        "Affiche la mire en plein ecran sur l'ecran selectionne dans la liste deroulante.",
    ),
    ("F3 / Echap", "Masquer mire", "Masque la fenetre de la mire."),
    (
        "F4",
        "Reconnecter camera",
        "Ferme puis rouvre la connexion a la camera evenementielle DVXplorer.",
    ),
    (
        "F5",
        "Rafraichir",
        "Recharge la liste des fichiers de calibration XML disponibles dans le dossier de sortie.",
    ),
    (
        "F6",
        "Calib",
        "Lance une sequence de capture (mire noire puis clignotante), associe les blobs "
        "detectes a la mire, et exporte l'observation (JSON + PNG) pour la calibration "
        "des intrinseques.",
    ),
    (
        "F7",
        "Erase",
        "Efface l'accumulation d'evenements en cours et supprime les derniers fichiers exportes "
        "(Calib/Test).",
    ),
    (
        "F8",
        "Reset",
        "Reinitialise completement (Erase + fermeture de la camera + remise a zero de la mire). "
        "Il faut ensuite cliquer sur \"Reconnecter camera\".",
    ),
    (
        "F9",
        "Test calib",
        "Lance une capture et evalue la calibration XML selectionnee sur l'observation obtenue "
        "(RMS de reprojection, pose, distance).",
    ),
    (
        "F10",
        "Test carre",
        "Lance la sequence de test \"carres\" (estimation de pose avec la mire 19 points, puis "
        "4 motifs carres a differentes tailles/positions) pour valider la calibration de facon "
        "independante.",
    ),
    (
        "F11",
        "Capture hand-eye",
        "(mode --external-mire) Enregistre un echantillon hand-eye combinant la pose camera-mire "
        "(solvePnP) et la transformee TF base-outil au meme instant.",
    ),
    (
        "Shift+F11",
        "Supprimer dernier",
        "(mode --external-mire) Supprime le dernier echantillon hand-eye enregistre dans la "
        "session en cours.",
    ),
    (
        "F12",
        "Recharger mire/TF",
        "(mode --external-mire) Recharge le layout de mire externe (telephone) et relance la "
        "lecture des transformees TF.",
    ),
]


class ShortcutsHelpDialog(QtWidgets.QDialog):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Aide - raccourcis et boutons")
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
        self.resize(720, 560)

        layout = QtWidgets.QVBoxLayout(self)
        text = QtWidgets.QTextEdit()
        text.setReadOnly(True)
        text.setHtml(self._build_html())
        layout.addWidget(text)

        close_button = QtWidgets.QPushButton("Fermer")
        close_button.clicked.connect(self.accept)
        button_row = QtWidgets.QHBoxLayout()
        button_row.addStretch(1)
        button_row.addWidget(close_button)
        layout.addLayout(button_row)

    @staticmethod
    def _build_html() -> str:
        rows = "".join(
            f"<tr><td><b>{key}</b></td><td>{button}</td><td>{description}</td></tr>"
            for key, button, description in SHORTCUTS_HELP
        )
        return (
            "<h2>Raccourcis clavier</h2>"
            "<p>Chaque raccourci declenche la meme action que le clic sur le bouton "
            "correspondant (et ne fait rien si ce bouton est desactive, par exemple "
            "pendant une capture en cours).</p>"
            "<table border='1' cellpadding='6' cellspacing='0' width='100%'>"
            "<tr><th>Touche</th><th>Bouton</th><th>Action</th></tr>"
            f"{rows}"
            "</table>"
            "<h2>Taille physique de l'ecran</h2>"
            "<p>La taille en mm de l'ecran de la mire est detectee automatiquement (EDID), "
            "mais cette detection n'est pas toujours fiable: elle peut etre absente ou fausse "
            "de quelques millimetres. Il est preferable de mesurer l'ecran et de saisir la "
            "largeur/hauteur dans les champs \"Taille ecran mesuree (mm)\" puis de cliquer sur "
            "\"Appliquer taille\".</p>"
            "<h2>Filtre bruit de fond</h2>"
            "<p>Le filtre BackgroundActivityNoiseFilter supprime les evenements isoles qui "
            "n'ont pas de voisin recent. Il ne change pas le mode d'accumulation: les evenements "
            "ON et OFF conserves sont toujours additionnes positivement dans la meme image.</p>"
        )


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
        self.external_dots: List[ScreenDot] = []
        self.external_layout: Optional[Dict[str, object]] = None
        self.tf_reader: Optional[TfPoseReader] = None
        self.handeye_session: Optional[Dict[str, object]] = None
        self.handeye_json_path: Optional[Path] = None
        self.handeye_tf_start: Optional[Tuple[List[float], List[float]]] = None
        self.handeye_capture_pending = False
        self._shortcuts: List[QtWidgets.QShortcut] = []
        self._help_dialog: Optional[ShortcutsHelpDialog] = None
        self.event_color: Tuple[int, int, int] = (255, 255, 255)
        self.marker_color: Tuple[int, int, int] = (0, 255, 0)
        self.background_color: Tuple[int, int, int] = (0, 0, 0)
        self.noise_filter_enabled = bool(getattr(args, "noise_filter", False))
        self.noise_filter_cutoff_hz = min(
            5000.0,
            max(1.0, float(getattr(args, "noise_cutoff_hz", 500.0))),
        )
        self._noise_filter: Optional[object] = None
        self.noise_filter_incoming_events = 0
        self.noise_filter_outgoing_events = 0
        self.noise_filter_error: Optional[str] = None

        self.mire = MireWindow(args.blink_hz, args.gradient_softness, args.pattern)
        self.mire.calibration_started.connect(self.begin_accumulation)
        self.mire.calibration_done.connect(self.finish_calibration)

        self.setWindowTitle("Calibration mire evenementielle")
        self.resize(1060, 760)
        self._build_ui()
        self._connect_ui()
        self._install_shortcuts()
        self.refresh_monitor_labels()
        self.place_control_window()

        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_camera)
        self.poll_timer.start(5)

        if self.args.external_mire:
            QtCore.QTimer.singleShot(0, self.setup_external_mire)
        elif self.selected_monitor_index >= 0:
            self.show_mire_on_selected_monitor()
        QtCore.QTimer.singleShot(0, self.ensure_camera)
        QtCore.QTimer.singleShot(0, self.show_screen_size_warning)
        QtCore.QTimer.singleShot(0, self.show_help_dialog)

    def _build_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)

        monitor_row = QtWidgets.QHBoxLayout()
        self.monitor_combo = QtWidgets.QComboBox()
        self.monitor_combo.setMinimumWidth(560)
        self.show_mire_button = QtWidgets.QPushButton("Afficher mire")
        self.hide_mire_button = QtWidgets.QPushButton("Masquer mire")
        self.help_button = QtWidgets.QPushButton("Aide (F1)")
        monitor_row.addWidget(QtWidgets.QLabel("Ecran mire:"))
        monitor_row.addWidget(self.monitor_combo, 1)
        monitor_row.addWidget(self.show_mire_button)
        monitor_row.addWidget(self.hide_mire_button)
        monitor_row.addWidget(self.help_button)
        root.addLayout(monitor_row)

        self.monitor_details = QtWidgets.QLabel()
        self.monitor_details.setWordWrap(True)
        root.addWidget(self.monitor_details)

        pattern_row = QtWidgets.QHBoxLayout()
        pattern_row.addWidget(QtWidgets.QLabel("Type de mire:"))
        self.pattern_combo = QtWidgets.QComboBox()
        for pattern in DOT_GRID_PATTERNS:
            self.pattern_combo.addItem(pattern.label, pattern.pattern_id)
        pattern_index = self.pattern_combo.findData(pattern_by_id(self.args.pattern).pattern_id)
        if pattern_index >= 0:
            self.pattern_combo.setCurrentIndex(pattern_index)
        self.pattern_combo.setEnabled(not self.args.external_mire)
        pattern_row.addWidget(self.pattern_combo)
        pattern_row.addStretch(1)
        root.addLayout(pattern_row)

        screen_size_row = QtWidgets.QHBoxLayout()
        screen_size_row.addWidget(QtWidgets.QLabel("Taille ecran mesuree (mm):"))
        self.screen_width_edit = QtWidgets.QLineEdit()
        self.screen_width_edit.setPlaceholderText("largeur (auto EDID)")
        self.screen_width_edit.setMaximumWidth(140)
        self.screen_width_edit.setValidator(QtGui.QDoubleValidator(1.0, 100000.0, 2))
        if self.args.screen_width_mm is not None:
            self.screen_width_edit.setText(str(self.args.screen_width_mm))
        screen_size_row.addWidget(self.screen_width_edit)
        screen_size_row.addWidget(QtWidgets.QLabel("x"))
        self.screen_height_edit = QtWidgets.QLineEdit()
        self.screen_height_edit.setPlaceholderText("hauteur (auto EDID)")
        self.screen_height_edit.setMaximumWidth(140)
        self.screen_height_edit.setValidator(QtGui.QDoubleValidator(1.0, 100000.0, 2))
        if self.args.screen_height_mm is not None:
            self.screen_height_edit.setText(str(self.args.screen_height_mm))
        screen_size_row.addWidget(self.screen_height_edit)
        self.apply_screen_size_button = QtWidgets.QPushButton("Appliquer taille")
        screen_size_row.addWidget(self.apply_screen_size_button)
        self.screen_size_info_button = QtWidgets.QPushButton("?")
        self.screen_size_info_button.setMaximumWidth(28)
        self.screen_size_info_button.setToolTip("Pourquoi mesurer l'ecran soi-meme ?")
        screen_size_row.addWidget(self.screen_size_info_button)
        screen_size_row.addStretch(1)
        root.addLayout(screen_size_row)

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

        colors_row = QtWidgets.QHBoxLayout()
        colors_row.addWidget(QtWidgets.QLabel("Couleur evenements:"))
        self.event_color_combo = QtWidgets.QComboBox()
        self._populate_color_combo(self.event_color_combo, self.event_color)
        colors_row.addWidget(self.event_color_combo)
        colors_row.addWidget(QtWidgets.QLabel("Couleur marqueurs:"))
        self.marker_color_combo = QtWidgets.QComboBox()
        self._populate_color_combo(self.marker_color_combo, self.marker_color)
        colors_row.addWidget(self.marker_color_combo)
        colors_row.addWidget(QtWidgets.QLabel("Couleur arriere-plan (visualisation):"))
        self.background_color_combo = QtWidgets.QComboBox()
        self._populate_color_combo(self.background_color_combo, self.background_color)
        colors_row.addWidget(self.background_color_combo)
        colors_row.addStretch(1)
        root.addLayout(colors_row)

        noise_filter_row = QtWidgets.QHBoxLayout()
        self.noise_filter_checkbox = QtWidgets.QCheckBox("Filtre bruit de fond (BackgroundActivityNoiseFilter)")
        self.noise_filter_checkbox.setToolTip(
            "Ne garde un evenement que s'il a un evenement voisin recent (a la frequence de "
            "cutoff choisie). Elimine le bruit de fond isole de la camera evenementielle."
        )
        self.noise_filter_checkbox.setChecked(self.noise_filter_enabled)
        noise_filter_row.addWidget(self.noise_filter_checkbox)
        noise_filter_row.addWidget(QtWidgets.QLabel("Frequence cutoff (Hz):"))
        self.noise_filter_cutoff_spin = QtWidgets.QDoubleSpinBox()
        self.noise_filter_cutoff_spin.setRange(1.0, 5000.0)
        self.noise_filter_cutoff_spin.setDecimals(1)
        self.noise_filter_cutoff_spin.setSingleStep(1.0)
        self.noise_filter_cutoff_spin.setValue(self.noise_filter_cutoff_hz)
        self.noise_filter_cutoff_spin.setEnabled(self.noise_filter_enabled)
        noise_filter_row.addWidget(self.noise_filter_cutoff_spin)
        noise_filter_row.addStretch(1)
        root.addLayout(noise_filter_row)

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

        if self.args.external_mire:
            handeye_row = QtWidgets.QHBoxLayout()
            self.handeye_status_label = QtWidgets.QLabel("Hand-eye: initialisation...")
            self.handeye_status_label.setMinimumHeight(28)
            self.handeye_refresh_button = QtWidgets.QPushButton("Recharger mire/TF")
            self.handeye_capture_button = QtWidgets.QPushButton("Capture hand-eye")
            self.handeye_capture_button.setMinimumHeight(38)
            self.handeye_undo_button = QtWidgets.QPushButton("Supprimer dernier")
            handeye_row.addWidget(self.handeye_status_label, 1)
            handeye_row.addWidget(self.handeye_refresh_button)
            handeye_row.addWidget(self.handeye_capture_button)
            handeye_row.addWidget(self.handeye_undo_button)
            root.addLayout(handeye_row)

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
        self.help_button.clicked.connect(self.show_help_dialog)
        self.pattern_combo.currentIndexChanged.connect(self.on_pattern_changed)
        self.apply_screen_size_button.clicked.connect(self.apply_manual_screen_size)
        self.screen_size_info_button.clicked.connect(self.show_screen_size_warning)
        self.event_color_combo.currentIndexChanged.connect(self.on_event_color_changed)
        self.marker_color_combo.currentIndexChanged.connect(self.on_marker_color_changed)
        self.background_color_combo.currentIndexChanged.connect(self.on_background_color_changed)
        self.noise_filter_checkbox.toggled.connect(self.on_noise_filter_toggled)
        self.noise_filter_cutoff_spin.valueChanged.connect(self.on_noise_filter_cutoff_changed)
        if self.args.external_mire:
            self.handeye_refresh_button.clicked.connect(self.setup_external_mire)
            self.handeye_capture_button.clicked.connect(self.start_handeye_capture)
            self.handeye_undo_button.clicked.connect(self.undo_last_handeye_sample)

    def _install_shortcuts(self) -> None:
        bindings: List[Tuple[str, Callable[[], None]]] = [
            ("F1", self.show_help_dialog),
            ("F2", self.show_mire_button.click),
            ("F3", self.hide_mire_button.click),
            ("Esc", self.hide_mire_button.click),
            ("F4", self.reconnect_camera_button.click),
            ("F5", self.refresh_calib_button.click),
            ("F6", self.calib_button.click),
            ("F7", self.erase_button.click),
            ("F8", self.reset_button.click),
            ("F9", self.test_button.click),
            ("F10", self.square_test_button.click),
        ]
        if self.args.external_mire:
            bindings.extend(
                [
                    ("F11", self.handeye_capture_button.click),
                    ("Shift+F11", self.handeye_undo_button.click),
                    ("F12", self.handeye_refresh_button.click),
                ]
            )
        for sequence, callback in bindings:
            shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(sequence), self)
            shortcut.setContext(QtCore.Qt.ApplicationShortcut)
            shortcut.activated.connect(callback)
            self._shortcuts.append(shortcut)

    def show_help_dialog(self) -> None:
        if self._help_dialog is None:
            self._help_dialog = ShortcutsHelpDialog(self)
        self._help_dialog.show()
        self._help_dialog.raise_()
        self._help_dialog.activateWindow()

    def show_screen_size_warning(self) -> None:
        box = QtWidgets.QMessageBox(self)
        box.setIcon(QtWidgets.QMessageBox.Warning)
        box.setWindowTitle("Taille physique de l'ecran")
        box.setText(
            "La detection automatique de la taille physique de l'ecran (en mm) n'est pas "
            "toujours fiable.\n\n"
            "Elle depend des informations EDID transmises par l'ecran (cable, adaptateur ou "
            "moniteur ne les fournissant pas forcement). Quand l'EDID n'est pas disponible, la "
            "taille utilisee peut etre completement erronee.\n\n"
            "Meme quand l'EDID est disponible, la valeur annoncee peut etre fausse de quelques "
            "millimetres, ce qui suffit a fausser le calcul mm/px et donc la calibration.\n\n"
            "Il est donc preferable de mesurer soi-meme la zone active de l'ecran (largeur et "
            "hauteur en mm) et de saisir ces valeurs dans les champs \"Taille ecran mesuree (mm)\" "
            "ci-dessus, puis de cliquer sur \"Appliquer taille\"."
        )
        box.setWindowFlags(box.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
        box.exec_()

    def apply_manual_screen_size(self) -> None:
        width_text = self.screen_width_edit.text().strip()
        height_text = self.screen_height_edit.text().strip()
        try:
            width_mm = float(width_text) if width_text else None
            height_mm = float(height_text) if height_text else None
        except ValueError:
            self.append_status("Taille ecran invalide: entrer des nombres en mm.")
            return
        self.args.screen_width_mm = width_mm
        self.args.screen_height_mm = height_mm
        self.update_monitor_details()
        if self.mire.isVisible():
            self.show_mire_on_selected_monitor()
        if width_mm is None and height_mm is None:
            self.append_status("Taille ecran manuelle effacee: retour a la detection automatique.")
        else:
            self.append_status(
                f"Taille ecran manuelle appliquee: {width_mm} x {height_mm} mm."
            )

    @staticmethod
    def _populate_color_combo(combo: QtWidgets.QComboBox, current: Tuple[int, int, int]) -> None:
        for name, bgr in PRESET_COLORS:
            combo.addItem(name, bgr)
        for index in range(combo.count()):
            if combo.itemData(index) == current:
                combo.setCurrentIndex(index)
                break

    def on_event_color_changed(self, index: int) -> None:
        bgr = self.event_color_combo.itemData(index)
        if bgr is None:
            return
        self.event_color = tuple(bgr)
        self.append_status(f"Couleur des evenements: {self.event_color_combo.currentText()}.")

    def on_marker_color_changed(self, index: int) -> None:
        bgr = self.marker_color_combo.itemData(index)
        if bgr is None:
            return
        self.marker_color = tuple(bgr)
        self.append_status(f"Couleur des marqueurs: {self.marker_color_combo.currentText()}.")

    def on_background_color_changed(self, index: int) -> None:
        bgr = self.background_color_combo.itemData(index)
        if bgr is None:
            return
        self.background_color = tuple(bgr)
        self.append_status(
            f"Couleur d'arriere-plan de la visualisation: {self.background_color_combo.currentText()} "
            "(n'affecte pas la mire)."
        )

    @staticmethod
    def _cutoff_to_duration(frequency_hz: float) -> timedelta:
        frequency_hz = max(float(frequency_hz), 0.1)
        microseconds = max(1, int(round(1_000_000.0 / frequency_hz)))
        return timedelta(microseconds=microseconds)

    def noise_filter_duration(self) -> timedelta:
        return self._cutoff_to_duration(self.noise_filter_cutoff_hz)

    def noise_filter_duration_us(self) -> int:
        return max(1, int(round(self.noise_filter_duration().total_seconds() * 1_000_000.0)))

    def _ensure_noise_filter(self) -> None:
        if self.camera is None:
            self._noise_filter = None
            return
        if self._noise_filter is None:
            self._noise_filter = self.camera.dv.noise.BackgroundActivityNoiseFilter(
                (self.camera.width, self.camera.height),
                self.noise_filter_duration(),
            )

    def reset_noise_filter_state(self, reset_counters: bool = True) -> None:
        self._noise_filter = None
        self.noise_filter_error = None
        if reset_counters:
            self.noise_filter_incoming_events = 0
            self.noise_filter_outgoing_events = 0
        if self.noise_filter_enabled and self.camera is not None:
            try:
                self._ensure_noise_filter()
            except Exception as exc:  # noqa: BLE001
                self.disable_noise_filter_after_error(exc)

    def disable_noise_filter_after_error(self, exc: Exception) -> None:
        self._noise_filter = None
        self.noise_filter_enabled = False
        self.noise_filter_error = str(exc)
        if hasattr(self, "noise_filter_checkbox"):
            self.noise_filter_checkbox.blockSignals(True)
            self.noise_filter_checkbox.setChecked(False)
            self.noise_filter_checkbox.blockSignals(False)
        if hasattr(self, "noise_filter_cutoff_spin"):
            self.noise_filter_cutoff_spin.setEnabled(False)
        self.append_status(f"Filtre bruit de fond desactive: {exc}")

    def filter_background_noise(self, events: object) -> object:
        if not self.noise_filter_enabled:
            return events
        incoming = event_store_size(events)
        if incoming <= 0:
            return events
        try:
            self._ensure_noise_filter()
            if self._noise_filter is None:
                return events
            self._noise_filter.accept(events)
            filtered = self._noise_filter.generateEvents()
        except Exception as exc:  # noqa: BLE001
            self.disable_noise_filter_after_error(exc)
            return events

        outgoing = event_store_size(filtered)
        self.noise_filter_incoming_events += incoming
        self.noise_filter_outgoing_events += outgoing
        return filtered

    def noise_filter_snapshot(self) -> EventFilterSnapshot:
        return EventFilterSnapshot(
            enabled=self.noise_filter_enabled,
            name="dv_processing.noise.BackgroundActivityNoiseFilter",
            support_duration_us=self.noise_filter_duration_us(),
            cutoff_hz=self.noise_filter_cutoff_hz,
            incoming_events=self.noise_filter_incoming_events,
            outgoing_events=self.noise_filter_outgoing_events,
            error=self.noise_filter_error,
        )

    def noise_filter_summary(self) -> str:
        return self.noise_filter_snapshot().summary()

    def on_noise_filter_toggled(self, checked: bool) -> None:
        self.noise_filter_enabled = bool(checked)
        self.noise_filter_cutoff_spin.setEnabled(self.noise_filter_enabled)
        if self.noise_filter_enabled:
            self.reset_noise_filter_state(reset_counters=True)
            if self.noise_filter_enabled:
                self.append_status(
                    f"Filtre bruit de fond active: cutoff {self.noise_filter_cutoff_hz:.1f} Hz "
                    f"(support {self.noise_filter_duration_us() / 1000.0:.3f} ms)."
                )
        else:
            self._noise_filter = None
            self.noise_filter_error = None
            self.noise_filter_incoming_events = 0
            self.noise_filter_outgoing_events = 0
            self.append_status("Filtre bruit de fond desactive.")

    def on_noise_filter_cutoff_changed(self, value: float) -> None:
        self.noise_filter_cutoff_hz = min(5000.0, max(1.0, float(value)))
        if self._noise_filter is not None:
            try:
                self._noise_filter.setBackgroundActivityDuration(self.noise_filter_duration())
            except Exception as exc:  # noqa: BLE001
                self.disable_noise_filter_after_error(exc)
                return
        self.append_status(
            f"Cutoff filtre bruit de fond: {self.noise_filter_cutoff_hz:.1f} Hz "
            f"(support {self.noise_filter_duration_us() / 1000.0:.3f} ms)."
        )

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
        if hasattr(self, "pattern_combo"):
            self.pattern_combo.setEnabled(enabled and not self.args.external_mire)
        if hasattr(self, "handeye_capture_button"):
            self.handeye_capture_button.setEnabled(enabled)

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

    def selected_pattern_id(self) -> str:
        if not hasattr(self, "pattern_combo"):
            return pattern_by_id(self.args.pattern).pattern_id
        data = self.pattern_combo.currentData()
        return pattern_by_id(str(data) if data is not None else self.args.pattern).pattern_id

    def selected_pattern(self) -> DotGridPattern:
        return pattern_by_id(self.selected_pattern_id())

    def on_pattern_changed(self, index: int) -> None:
        data = self.pattern_combo.itemData(index)
        if data is None:
            return
        pattern = pattern_by_id(str(data))
        self.args.pattern = pattern.pattern_id
        self.mire.set_pattern(pattern.pattern_id)
        self.preview_blobs = []
        self.last_preview_blob_update = 0.0
        self.append_status(f"Type de mire: {pattern.label}.")

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
        pattern_text = (
            self.mire.current_dot_grid_pattern().label
            if self.mire.pattern != "square4"
            else "Carre 4 points"
        )
        self.append_status(f"Mire affichee sur {monitor.label()} | {pattern_text}")

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
        self.reset_noise_filter_state(reset_counters=True)
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
        self.reset_noise_filter_state(reset_counters=True)
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

        events = self.filter_background_noise(events)

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

    def active_expected_dots(self) -> int:
        if self.args.external_mire and self.external_dots:
            return len(self.external_dots)
        if self.square_phase == "validation":
            return SQUARE_EXPECTED_DOTS
        return self.mire.expected_dot_count()

    def effective_min_matched(self, expected: int) -> int:
        configured = int(getattr(self.args, "min_matched", 0))
        if configured <= 0:
            return expected
        return min(configured, expected)

    def update_live_preview(self) -> None:
        now = time.time()
        if now - self.last_preview_update < 0.04:
            return
        self.last_preview_update = now
        expected = self.active_expected_dots()

        if self.accumulating and self.activity is not None:
            source_activity = self.activity
            if now - self.last_preview_blob_update >= 0.12:
                self.preview_blobs = detect_blobs(source_activity, expected)
                self.last_preview_blob_update = now
            image = normalize_activity_to_bgr(self.activity, self.event_color, self.background_color)
            draw_blob_indicators(image, self.preview_blobs, color=self.marker_color)
            banner = (
                f"ACCUM {self.event_count} events | "
                f"blobs {len(self.preview_blobs)}/{expected} | "
                f"fenetre {self.current_capture_duration_ms} ms"
            )
            image = draw_preview_banner(image, banner, (0, 255, 255))
        elif self.live_activity is not None:
            source_activity = self.live_activity
            if now - self.last_preview_blob_update >= 0.12:
                self.preview_blobs = detect_blobs(source_activity, expected)
                self.last_preview_blob_update = now
            image = normalize_activity_to_bgr(self.live_activity, self.event_color, self.background_color)
            draw_blob_indicators(image, self.preview_blobs, color=self.marker_color)
            image = draw_preview_banner(
                image,
                f"LIVE {self.live_event_count} events | blobs {len(self.preview_blobs)}/{expected}",
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
            "pose_pattern": self.selected_pattern().to_json(),
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
        self.mire.set_pattern("square4" if square_phase == "validation" else self.selected_pattern_id())
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
            message = (
                f"Test carre phase 1/2: estimation pose avec "
                f"{self.mire.expected_dot_count()} points."
            )
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
        self.reset_noise_filter_state(reset_counters=True)
        self.accumulating = True
        self.accum_started_at = time.time()
        self.append_status(
            f"Accumulation active pour {self.current_capture_duration_ms} ms "
            f"(polarites ON/OFF additionnees, {self.noise_filter_summary()})."
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

        expected = SQUARE_EXPECTED_DOTS if square_phase == "validation" else self.mire.expected_dot_count()
        min_matched = self.effective_min_matched(expected)
        self.last_blobs = detect_blobs(self.activity, expected)
        if square_phase == "validation":
            self.finish_square_validation(elapsed_ms)
            return

        self.last_matches, reason = associate_blobs_to_layout(self.last_blobs, self.mire.dots)
        overlay = make_overlay(
            self.activity, self.last_blobs, self.last_matches,
            event_color=self.event_color, marker_color=self.marker_color,
            background_color=self.background_color,
        )
        self.preview_label.setPixmap(pixmap_from_bgr(overlay, self.preview_label.size()))

        self.append_status(
            f"Detection: {len(self.last_blobs)} blobs, "
            f"{len(self.last_matches)}/{expected} associations, "
            f"{self.event_count} events, {elapsed_ms:.0f} ms, "
            f"{self.noise_filter_summary()}. {reason}"
        )
        if self.test_mode:
            self.test_mode = False
            if len(self.last_matches) >= min_matched:
                self.run_calibration_test(overlay, elapsed_ms)
            else:
                self.append_status(
                    f"Test ignore: associations insuffisantes "
                    f"({len(self.last_matches)}/{min_matched})."
                )
            return
        if square_phase == "pose":
            if len(self.last_matches) >= min_matched:
                self.finish_square_pose(overlay, elapsed_ms)
            else:
                self.square_phase = None
                self.square_test_context = None
                self.set_capture_buttons_enabled(True)
                self.append_status(
                    f"Test carre ignore: associations phase 1 insuffisantes "
                    f"({len(self.last_matches)}/{min_matched})."
                )
            return
        if len(self.last_matches) >= min_matched:
            self.export_observation(overlay, elapsed_ms)
        else:
            self.append_status(
                f"Export ignore: associations insuffisantes "
                f"({len(self.last_matches)}/{min_matched})."
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
                "pose_background_noise_filter": self.noise_filter_snapshot().to_json(),
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
        overlay = make_overlay(
            self.activity, self.last_blobs, self.last_matches,
            event_color=self.event_color, marker_color=self.marker_color,
            background_color=self.background_color,
        )
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
            "polarity_mode": "positive_count + negative_count",
            "background_noise_filter": self.noise_filter_snapshot().to_json(),
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
                "polarity_mode": "positive_count + negative_count",
                "background_noise_filter": context.get("pose_background_noise_filter"),
                "pattern": context.get("pose_pattern"),
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
        self.mire.set_pattern(self.selected_pattern_id())
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
        pattern = self.mire.current_dot_grid_pattern()
        min_matched = self.effective_min_matched(pattern.expected_dots)

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
                "pattern": pattern.pattern_id,
                "pattern_type": "dot_grid",
                "pattern_label": pattern.label,
                "rows": pattern.rows,
                "cols": pattern.cols,
                "expected_dots": pattern.expected_dots,
                "missing_dot": (
                    {"row": pattern.missing_dot[0], "col": pattern.missing_dot[1]}
                    if pattern.missing_dot is not None
                    else None
                ),
                "anchor_dot": {"row": pattern.anchor_dot[0], "col": pattern.anchor_dot[1]},
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
                "background_noise_filter": self.noise_filter_snapshot().to_json(),
            },
            "detection": {
                "expected_dots": pattern.expected_dots,
                "min_matched": min_matched,
                "configured_min_matched": int(getattr(self.args, "min_matched", 0)),
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
                "polarity_mode": "positive_count + negative_count",
                "background_noise_filter": self.noise_filter_snapshot().to_json(),
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

    # ------------------------------------------------------------------
    # Hand-eye collection (--external-mire)
    # ------------------------------------------------------------------

    def setup_external_mire(self) -> None:
        try:
            self.external_dots, self.external_layout = fetch_external_layout(
                self.args.external_mire
            )
            spacing = float(self.external_layout["layout"]["spacing_x_mm"])
            source = str(self.external_layout.get("layout_source", "phone-live"))
            source_label = {
                "cache": "cache plein ecran",
                "poco-default": "profil Poco par defaut",
                "phone-live": "telephone connecte",
            }.get(source, source)
            self.append_status(
                f"Mire externe chargee: {len(self.external_dots)} points, "
                f"espacement {spacing:.2f} mm ({source_label})."
            )
        except RuntimeError as exc:
            self.external_dots = []
            self.external_layout = None
            self.append_status(f"Mire externe indisponible: {exc}")
        if self.tf_reader is None:
            try:
                self.tf_reader = TfPoseReader(
                    self.args.robot_base_frame, self.args.robot_tool_frame
                )
                self.append_status(
                    f"Listener tf2 actif ({self.args.robot_base_frame} -> "
                    f"{self.args.robot_tool_frame})."
                )
            except RuntimeError as exc:
                self.append_status(str(exc))
        self.update_handeye_status()

    def update_handeye_status(self) -> None:
        if not hasattr(self, "handeye_status_label"):
            return
        mire_text = (
            f"mire {len(self.external_dots)} pts" if self.external_dots else "mire ABSENTE"
        )
        tf_ok = False
        if self.tf_reader is not None:
            try:
                self.tf_reader.lookup_tool_pose()
                tf_ok = True
            except Exception:  # noqa: BLE001
                tf_ok = False
        count = len(self.handeye_session["samples"]) if self.handeye_session else 0
        path_text = f" | {self.handeye_json_path.name}" if self.handeye_json_path else ""
        self.handeye_status_label.setText(
            f"Hand-eye: {mire_text} | TF {'ok' if tf_ok else 'ABSENT'} | "
            f"{count} echantillons{path_text}"
        )

    def ensure_handeye_session(self, intrinsics_path: Path) -> None:
        if self.handeye_session is not None:
            return
        output_dir = Path(self.args.output_dir) / "handeye"
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.handeye_json_path = output_dir / f"handeye_samples_{stamp}.json"
        self.handeye_session = {
            "created_at": datetime.now().isoformat(timespec="milliseconds"),
            "units": "meters",
            "frames": {
                "robot_parent": self.args.robot_base_frame,
                "robot_child": self.args.robot_tool_frame,
                "camera": self.args.camera_frame,
                "mire": self.args.mire_frame,
            },
            "intrinsics_xml": str(intrinsics_path),
            "mire_source": {
                "url": self.args.external_mire,
                "screen": self.external_layout.get("screen") if self.external_layout else None,
                "layout": self.external_layout.get("layout") if self.external_layout else None,
            },
            "samples": [],
        }

    def save_handeye_session(self) -> None:
        if self.handeye_session is None or self.handeye_json_path is None:
            return
        with self.handeye_json_path.open("w", encoding="utf-8") as handle:
            json.dump(self.handeye_session, handle, indent=2)

    def start_handeye_capture(self) -> None:
        if self.accumulating or self.handeye_capture_pending:
            return
        if not self.ensure_camera():
            return
        if not self.external_dots or self.tf_reader is None:
            self.setup_external_mire()
        if not self.external_dots:
            self.append_status("Capture annulee: mire externe indisponible.")
            return
        if self.tf_reader is None:
            return
        selected = self.selected_calibration_path()
        if selected is None:
            self.append_status(
                "Capture annulee: aucune calibration XML (intrinseques requis pour solvePnP)."
            )
            return
        try:
            self.handeye_tf_start = self.tf_reader.lookup_tool_pose()
        except Exception as exc:  # noqa: BLE001
            self.append_status(f"Capture annulee: TF indisponible ({exc}).")
            self.update_handeye_status()
            return

        self.activity = np.zeros((self.camera.height, self.camera.width), dtype=np.float32)
        self.event_count = 0
        self.last_blobs = []
        self.last_matches = []
        self.preview_blobs = []
        self.last_preview_blob_update = 0.0
        self.current_capture_duration_ms = int(self.args.accum_ms)
        self.reset_noise_filter_state(reset_counters=True)
        self.accumulating = True
        self.accum_started_at = time.time()
        self.handeye_capture_pending = True
        self.set_capture_buttons_enabled(False)
        self.append_status(
            f"Capture hand-eye: accumulation {self.current_capture_duration_ms} ms "
            f"(mire telephone en clignotement libre, robot immobile, {self.noise_filter_summary()})."
        )
        QtCore.QTimer.singleShot(self.current_capture_duration_ms, self.finish_handeye_capture)

    def finish_handeye_capture(self) -> None:
        if not self.handeye_capture_pending:
            return
        self.handeye_capture_pending = False
        self.accumulating = False
        self.set_capture_buttons_enabled(True)
        elapsed_ms = (time.time() - self.accum_started_at) * 1000.0

        try:
            tf_end = self.tf_reader.lookup_tool_pose()
        except Exception as exc:  # noqa: BLE001
            self.append_status(f"Echantillon rejete: TF de fin indisponible ({exc}).")
            return
        if self.activity is None or self.handeye_tf_start is None:
            self.append_status("Echantillon rejete: accumulation manquante.")
            return

        expected = len(self.external_dots)
        min_matched = self.effective_min_matched(expected)
        self.last_blobs = detect_blobs(self.activity, expected)
        self.last_matches, reason = associate_blobs_to_layout(self.last_blobs, self.external_dots)
        overlay = make_overlay(
            self.activity, self.last_blobs, self.last_matches,
            event_color=self.event_color, marker_color=self.marker_color,
            background_color=self.background_color,
        )
        self.preview_label.setPixmap(pixmap_from_bgr(overlay, self.preview_label.size()))
        self.append_status(
            f"Detection: {len(self.last_blobs)} blobs, "
            f"{len(self.last_matches)}/{expected} associations, "
            f"{self.event_count} events, {elapsed_ms:.0f} ms, "
            f"{self.noise_filter_summary()}. {reason}"
        )

        selected = self.selected_calibration_path()
        try:
            camera_matrix, dist_coeffs, _ = load_calibration_xml(selected)
        except (RuntimeError, cv.error) as exc:
            self.append_status(f"Echantillon rejete: intrinseques illisibles ({exc}).")
            return

        pnp: Dict[str, object] = {"valid": False, "reason": "associations insuffisantes"}
        if len(self.last_matches) >= min_matched:
            pnp = solve_mire_pose_with_ambiguity(self.last_matches, camera_matrix, dist_coeffs)
        if not pnp.get("valid"):
            self.append_status(f"Echantillon rejete: {pnp.get('reason')}.")
            return

        stationarity = compute_stationarity(self.handeye_tf_start, tf_end)
        rejection = handeye_rejection_reason(
            stationarity,
            len(self.last_matches),
            min_matched,
            pnp.get("ambiguity_ratio"),
            float(pnp["tilt_deg"]),
            self.args.stationarity_trans_mm,
            self.args.stationarity_rot_deg,
            self.args.ambiguity_min_ratio,
            self.args.ambiguity_min_tilt_deg,
        )
        if rejection is not None:
            self.append_status(f"Echantillon rejete: {rejection}.")
            banner = draw_preview_banner(overlay.copy(), f"REJET: {rejection}", (0, 180, 255))
            self.preview_label.setPixmap(pixmap_from_bgr(banner, self.preview_label.size()))
            return

        self.ensure_handeye_session(selected)
        sample = build_handeye_sample(
            len(self.handeye_session["samples"]),
            self.handeye_tf_start,
            tf_end,
            self.tf_reader.joint_positions(),
            pnp,
            self.last_matches,
        )
        sample["polarity_mode"] = "positive_count + negative_count"
        sample["background_noise_filter"] = self.noise_filter_snapshot().to_json()
        self.handeye_session["samples"].append(sample)
        self.save_handeye_session()

        ambiguity = sample["ippe_ambiguity_ratio"]
        ambiguity_text = f", ambiguite {ambiguity:.1f}" if ambiguity is not None else ""
        self.append_status(
            f"Echantillon {sample['index']} enregistre: rms {sample['reproj_rms_px']:.2f} px, "
            f"tilt {sample['tilt_deg']:.1f} deg{ambiguity_text}"
        )
        self.append_status(
            f"  stationnarite {sample['stationarity']['trans_delta_mm']:.3f} mm / "
            f"{sample['stationarity']['rot_delta_deg']:.4f} deg | "
            f"distance {float(pnp['distance_norm_mm']):.0f} mm | {self.handeye_json_path}"
        )
        banner = draw_preview_banner(
            overlay.copy(),
            f"HANDEYE #{sample['index']} rms {sample['reproj_rms_px']:.2f}px "
            f"tilt {sample['tilt_deg']:.0f}deg",
            (0, 255, 0),
        )
        self.preview_label.setPixmap(pixmap_from_bgr(banner, self.preview_label.size()))
        self.update_handeye_status()

    def undo_last_handeye_sample(self) -> None:
        if not self.handeye_session or not self.handeye_session["samples"]:
            self.append_status("Aucun echantillon hand-eye a supprimer.")
            return
        removed = self.handeye_session["samples"].pop()
        self.save_handeye_session()
        self.append_status(f"Echantillon {removed['index']} supprime.")
        self.update_handeye_status()

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
        self.reset_noise_filter_state(reset_counters=True)
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
        self.mire.set_pattern(self.selected_pattern_id())
        self.mire.restart_blink()
        self.mire.update()
        self.set_camera_status("Camera: reset, appuyer sur Reconnecter camera", "warn")
        self.append_status("Reset complet.")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        if self.camera is not None:
            self.camera.close()
        if self.tf_reader is not None:
            self.tf_reader.close()
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


def make_synthetic_activity(
    pattern_id: str = DEFAULT_PATTERN_ID,
) -> Tuple[np.ndarray, List[ScreenDot], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    width, height = 640, 480
    dots, _ = build_mire_layout(width, height, 1.0, 1.0, pattern_id)
    camera_matrix = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.zeros(5, dtype=np.float64)
    rvec = np.array([[0.05], [-0.18], [0.03]], dtype=np.float64)
    tvec = np.array([[15.0], [5.0], [680.0]], dtype=np.float64)
    activity = render_synthetic_activity(width, height, dots, camera_matrix, dist_coeffs, rvec, tvec)
    return activity, dots, camera_matrix, dist_coeffs, rvec, tvec


def run_background_filter_self_test() -> bool:
    try:
        dv = load_dv_processing()
    except RuntimeError as exc:
        print(f"background filter self-test skipped: {exc}")
        return True

    isolated = dv.EventStore()
    for index in range(12):
        isolated.push_back(index * 1000, (index * 7) % 64, (index * 11) % 64, True)

    coherent = dv.EventStore()
    for index in range(12):
        coherent.push_back(
            index * 100,
            30 + (index % 3),
            30 + ((index // 3) % 3),
            True,
        )

    duration = timedelta(milliseconds=2)
    isolated_filter = dv.noise.BackgroundActivityNoiseFilter((64, 64), duration)
    isolated_filter.accept(isolated)
    isolated_out = isolated_filter.generateEvents()

    coherent_filter = dv.noise.BackgroundActivityNoiseFilter((64, 64), duration)
    coherent_filter.accept(coherent)
    coherent_out = coherent_filter.generateEvents()

    print(f"background filter isolated: {len(isolated_out)}/{len(isolated)}")
    print(f"background filter coherent: {len(coherent_out)}/{len(coherent)}")

    if len(coherent_out) < 8 or len(coherent_out) <= len(isolated_out):
        print("background filter self-test failed")
        return False
    return True


def run_blob_center_self_test() -> bool:
    height, width = 28, 32
    true_cx, true_cy = 15.3, 13.6
    yy, xx = np.ogrid[:height, :width]
    mask = (xx - true_cx) ** 2 + (yy - true_cy) ** 2 <= 9.0 ** 2
    activity = np.zeros((height, width), dtype=np.float32)
    activity[mask] = 60.0
    activity[mask & (xx > true_cx + 4.0) & (yy < true_cy + 3.0)] += 500.0

    mask_ys, mask_xs = np.nonzero(mask)
    component_centroid = (float(np.mean(mask_xs)), float(np.mean(mask_ys)))
    center_x, center_y, method, agreement = estimate_blob_center(
        mask,
        activity,
        component_centroid,
        0,
        0,
    )

    raw_weights = activity[mask]
    raw_x = float(np.sum(mask_xs * raw_weights) / np.sum(raw_weights))
    raw_y = float(np.sum(mask_ys * raw_weights) / np.sum(raw_weights))
    robust_error = math.hypot(center_x - true_cx, center_y - true_cy)
    raw_error = math.hypot(raw_x - true_cx, raw_y - true_cy)
    print(
        f"blob center robust: {robust_error:.3f}px via {method} "
        f"(weighted {raw_error:.3f}px, agreement {agreement})"
    )
    if robust_error > 1.0 or robust_error >= raw_error:
        print("blob center self-test failed")
        return False
    return True


def run_self_test() -> int:
    activity, dots, camera_matrix, dist_coeffs, rvec, tvec = make_synthetic_activity()
    blobs = detect_blobs(activity, len(dots))
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
    if not run_blob_center_self_test():
        return 1
    if not run_background_filter_self_test():
        return 1

    for pattern in DOT_GRID_PATTERNS:
        activity_p, dots_p, _, _, _, _ = make_synthetic_activity(pattern.pattern_id)
        blobs_p = detect_blobs(activity_p, len(dots_p))
        matches_p, reason_p = associate_blobs_to_layout(blobs_p, dots_p)
        print(
            f"synthetic pattern {pattern.pattern_id}: "
            f"{len(blobs_p)} blobs, {len(matches_p)} matches ({reason_p})"
        )
        if len(dots_p) != pattern.expected_dots:
            print(f"layout failed for {pattern.pattern_id}: {len(dots_p)} dots")
            return 1
        if len(blobs_p) != pattern.expected_dots:
            print(f"blob detection failed for {pattern.pattern_id}")
            return 1
        if len(matches_p) != pattern.expected_dots:
            print(f"association failed for {pattern.pattern_id}")
            return 1

    pca_collapse_dots, _ = build_mire_layout(2560, 1440, 1.0, 1.0, "mire")
    pca_collapse_points = np.array(
        [
            [
                160.0 + 45.0 * dot.col - 2.0 * dot.row - 1.0 * dot.row * dot.col,
                150.0 + 60.0 * dot.row - 6.0 * dot.col - 3.0 * dot.row * dot.col,
            ]
            for dot in pca_collapse_dots
        ],
        dtype=np.float64,
    )
    pca_collapse_cycle = grid_corner_cycle(pca_collapse_points)
    pca_collapse_blobs = [
        Blob(
            index=idx,
            x=float(point[0]),
            y=float(point[1]),
            area_px=24,
            weight=5000.0 if dot.anchor else 1000.0 - idx,
            peak=1.0,
            bbox=(int(round(point[0])) - 5, int(round(point[1])) - 5, 10, 10),
        )
        for idx, (dot, point) in enumerate(zip(pca_collapse_dots, pca_collapse_points))
    ]
    pca_collapse_matches, pca_collapse_reason = associate_blobs_to_layout(
        pca_collapse_blobs, pca_collapse_dots
    )
    print(
        "pca-collapse mire: "
        f"pca corners {len(set(pca_collapse_cycle))}/4, "
        f"{len(pca_collapse_matches)} matches ({pca_collapse_reason})"
    )
    if len(set(pca_collapse_cycle)) >= 4 or len(pca_collapse_matches) != len(pca_collapse_dots):
        print("pca-collapse association failed")
        return 1

    # Tilted / rolled poses: the whole point of the homography association is to
    # accept oblique calibration views, so lock that in with strong off-axis
    # rotations that the old row/column split could not handle.
    tilt_camera = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    tilt_dist = np.zeros(5, dtype=np.float64)
    tilt_tvec = np.array([[10.0], [8.0], [820.0]], dtype=np.float64)
    challenge_rvecs = [
        np.array([[0.35], [0.32], [0.45]], dtype=np.float64),   # tilt + ~26 deg roll
        np.array([[-0.30], [0.40], [-0.55]], dtype=np.float64),  # opposite tilt + roll
        np.array([[0.18], [-0.48], [0.60]], dtype=np.float64),   # strong y tilt + ~34 deg roll
    ]
    for pattern_id in ("mire", "grid_5x4", "grid_7x5"):
        tilt_dots, _ = build_mire_layout(640, 480, 1.0, 1.0, pattern_id)
        for pose_idx, tilt_rvec in enumerate(challenge_rvecs):
            tilt_activity = render_synthetic_activity(
                640, 480, tilt_dots, tilt_camera, tilt_dist, tilt_rvec, tilt_tvec
            )
            tilt_blobs = detect_blobs(tilt_activity, len(tilt_dots))
            tilt_matches, tilt_reason = associate_blobs_to_layout(tilt_blobs, tilt_dots)
            print(
                f"tilt {pattern_id} pose {pose_idx + 1}: "
                f"{len(tilt_blobs)} blobs, {len(tilt_matches)} matches ({tilt_reason})"
            )
            if len(tilt_matches) != len(tilt_dots):
                print(f"tilted association failed for {pattern_id} pose {pose_idx + 1}")
                return 1
            tilt_pose = evaluate_calibration_on_matches(tilt_matches, tilt_camera, tilt_dist)
            if not tilt_pose.get("valid") or float(tilt_pose.get("rms_px", 1e9)) > 2.0:
                print(
                    f"tilted pose check failed for {pattern_id} pose {pose_idx + 1}: "
                    f"{tilt_pose.get('reason')} rms={tilt_pose.get('rms_px')}"
                )
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

    # Hand-eye collection units (plan step 2).
    pnp = solve_mire_pose_with_ambiguity(matches, camera_matrix, dist_coeffs)
    print(
        f"handeye pnp valid: {pnp.get('valid')} rms={pnp.get('rms_px')} "
        f"ambiguity={pnp.get('ambiguity_ratio')}"
    )
    if not pnp.get("valid") or float(pnp["rms_px"]) > 1.5:
        print("handeye solvePnP failed")
        return 1
    true_distance = float(np.linalg.norm(tvec))
    est_distance = float(pnp["distance_norm_mm"])
    if abs(est_distance - true_distance) > 0.02 * true_distance:
        print(f"handeye distance off: {est_distance} vs {true_distance}")
        return 1

    rotation, _ = cv.Rodrigues(rvec)
    quat = rotation_matrix_to_quat_xyzw(rotation)
    rotation_back = np.zeros((3, 3))
    x, y, z, w = quat
    rotation_back = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    if float(np.max(np.abs(rotation_back - rotation))) > 1e-9:
        print("quaternion round-trip failed")
        return 1

    tf_start = ([0.30, -0.10, 0.40], [0.0, 0.0, 0.0, 1.0])
    moved_quat = [0.0, 0.0, math.sin(math.radians(0.05) / 2.0), math.cos(math.radians(0.05) / 2.0)]
    tf_end = ([0.30, -0.10 + 0.0002, 0.40], moved_quat)
    stationarity = compute_stationarity(tf_start, tf_end)
    print(
        f"handeye stationarity: {stationarity['trans_delta_mm']:.3f} mm "
        f"{stationarity['rot_delta_deg']:.4f} deg"
    )
    if abs(stationarity["trans_delta_mm"] - 0.2) > 1e-6 or abs(stationarity["rot_delta_deg"] - 0.05) > 1e-6:
        print("stationarity computation failed")
        return 1

    still = {"trans_delta_mm": 0.02, "rot_delta_deg": 0.005}
    cases = [
        (handeye_rejection_reason(still, 19, 19, 5.0, 30.0, 0.1, 0.02, 1.5, 15.0), None),
        (handeye_rejection_reason(stationarity, 19, 19, 5.0, 30.0, 0.1, 0.02, 1.5, 15.0), "moving"),
        (handeye_rejection_reason(still, 17, 19, 5.0, 30.0, 0.1, 0.02, 1.5, 15.0), "matches"),
        (handeye_rejection_reason(still, 19, 19, 1.1, 5.0, 0.1, 0.02, 1.5, 15.0), "ambiguity"),
        (handeye_rejection_reason(still, 19, 19, 1.1, 30.0, 0.1, 0.02, 1.5, 15.0), None),
    ]
    for idx, (reason_text, expectation) in enumerate(cases):
        ok = (reason_text is None) if expectation is None else (reason_text is not None)
        if not ok:
            print(f"handeye rejection case {idx} failed: {reason_text!r}")
            return 1
    print("handeye rejection gates ok")

    landscape_screen = {
        "viewport_px": {"width": 2712, "height": 1220},
        "panel_px": {"width": 2712, "height": 1220},
        "landscape_ok": True,
        "fullscreen_ok": True,
    }
    portrait_screen = {
        "viewport_px": {"width": 1220, "height": 2712},
        "panel_px": {"width": 1220, "height": 2712},
        "landscape_ok": False,
        "fullscreen_ok": True,
    }
    cropped_screen = {
        "viewport_px": {"width": 2712, "height": 1100},
        "panel_px": {"width": 2712, "height": 1220},
        "landscape_ok": True,
        "fullscreen_ok": False,
    }
    if external_layout_screen_error(landscape_screen) is not None:
        print("landscape phone layout rejected")
        return 1
    if external_layout_screen_error(portrait_screen) is None:
        print("portrait phone layout accepted")
        return 1
    if external_layout_screen_error(cropped_screen) is None:
        print("non-fullscreen phone layout accepted")
        return 1
    print("phone landscape/fullscreen gates ok")

    sample = build_handeye_sample(0, tf_start, tf_end, [0.1] * 6, pnp, matches)
    required_keys = {
        "index", "stamp", "T_base_tool0", "T_camera_mire", "joint_positions_rad",
        "stationarity", "reproj_rms_px", "matched_dots", "tilt_deg",
        "ippe_ambiguity_ratio", "matches",
    }
    if not required_keys.issubset(sample.keys()):
        print(f"sample schema missing keys: {required_keys - set(sample.keys())}")
        return 1
    xyz_m = sample["T_camera_mire"]["xyz"]
    tvec_mm = sample["T_camera_mire"]["tvec_mm"]
    if any(abs(m * 1000.0 - mm) > 1e-9 for m, mm in zip(xyz_m, tvec_mm)):
        print("sample mm->m conversion failed")
        return 1
    if sample["matched_dots"] != EXPECTED_DOTS or len(sample["matches"]) != EXPECTED_DOTS:
        print("sample matches incomplete")
        return 1
    print("handeye sample schema ok")

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
        "--pattern",
        choices=[pattern.pattern_id for pattern in DOT_GRID_PATTERNS],
        default=DEFAULT_PATTERN_ID,
        help="Initial blinking dot-grid target shown in the dropdown.",
    )
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
        "--noise-filter",
        action="store_true",
        help="Enable dv-processing BackgroundActivityNoiseFilter by default.",
    )
    parser.add_argument(
        "--noise-cutoff-hz",
        type=float,
        default=500.0,
        help="Background noise support cutoff in Hz (duration = 1 / cutoff, default 500 Hz = 2 ms).",
    )
    parser.add_argument(
        "--min-matched",
        type=int,
        default=0,
        help="Minimum associated centers required before export (0 = all dots in the active target).",
    )
    parser.add_argument("--self-test", action="store_true", help="Run synthetic blob/layout tests and exit.")
    parser.add_argument(
        "--external-mire",
        metavar="URL",
        help="Phone mire server URL (serve_phone_mire.py, e.g. http://127.0.0.1:8081). "
        "Enables hand-eye collection: no local mire window, layout fetched from "
        "the server, TF base->tool0 read at each capture.",
    )
    parser.add_argument("--robot-base-frame", default="base", help="TF parent frame (UR convention).")
    parser.add_argument("--robot-tool-frame", default="tool0", help="TF child frame (flange).")
    parser.add_argument("--camera-frame", default="camera_optical", help="Camera frame name in exports.")
    parser.add_argument("--mire-frame", default="screen_center", help="Mire frame name in exports.")
    parser.add_argument(
        "--stationarity-trans-mm",
        type=float,
        default=0.1,
        help="Max TF translation drift during accumulation before rejecting a sample.",
    )
    parser.add_argument(
        "--stationarity-rot-deg",
        type=float,
        default=0.02,
        help="Max TF rotation drift during accumulation before rejecting a sample.",
    )
    parser.add_argument(
        "--ambiguity-min-ratio",
        type=float,
        default=1.5,
        help="Reject a sample when the IPPE ambiguity ratio is below this with low tilt.",
    )
    parser.add_argument(
        "--ambiguity-min-tilt-deg",
        type=float,
        default=15.0,
        help="Tilt below which a low IPPE ambiguity ratio rejects the sample.",
    )
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
