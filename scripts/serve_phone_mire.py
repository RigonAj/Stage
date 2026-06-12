#!/usr/bin/env python3
"""Serve the blinking calibration mire as a web page for a smartphone screen.

The phone (e.g. Poco X7 Pro) opens this page over Wi-Fi and displays the same
19-dot mire as ``event_mire_calibration.py``. The dot layout is computed
server-side with ``build_mire_layout`` so there is a single source of truth:
the page only draws what the server returns, in physical screen pixels.

The collector (hand-eye step 2) reads ``/api/current_layout`` to use exactly
the geometry the phone is displaying.

Usage:
    python3 scripts/serve_phone_mire.py --host 0.0.0.0 --port 8081
    # on the phone (same network): http://<PC_IP>:8081/

Checklist phone side (AMOLED, see docs/ur3e_camera_base_calibration.md §2-3):
- brightness 100% (or DC dimming), auto-brightness off, refresh rate fixed,
  screen timeout "never", notifications off, real fullscreen.
- verify the printed expected dot spacing with a caliper (mode "measure").
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from event_mire_calibration import (  # noqa: E402
    ANCHOR_DOT,
    COLS,
    EXPECTED_DOTS,
    MISSING_DOT,
    ROWS,
    build_mire_layout,
)

# Poco X7 Pro active display, portrait reference (docs §2).
DEFAULT_SCREEN_WIDTH_MM = 69.55
DEFAULT_SCREEN_HEIGHT_MM = 154.50
PAGE_PATH = Path(__file__).resolve().parent / "phone_mire.html"


def oriented_screen_mm(
    width_px: int, height_px: int, portrait_width_mm: float, portrait_height_mm: float
) -> Tuple[float, float]:
    """Match the long mm side to the long pixel side (portrait vs landscape)."""
    short_mm, long_mm = sorted((portrait_width_mm, portrait_height_mm))
    if width_px >= height_px:
        return long_mm, short_mm
    return short_mm, long_mm


def compute_layout_payload(
    viewport_width_px: int,
    viewport_height_px: int,
    panel_width_px: Optional[int],
    panel_height_px: Optional[int],
    portrait_width_mm: float,
    portrait_height_mm: float,
    blink_hz: float,
    gradient_softness: int,
) -> Dict[str, object]:
    """Build the mire layout for the phone viewport, in physical pixels.

    mm-per-px is derived from the full panel resolution (the physical screen
    size in mm describes the whole panel). The layout itself is placed in the
    fullscreen viewport; if the viewport is smaller than the panel (browser
    bars, letterboxing) the page is NOT in real fullscreen and the px->mm
    mapping of dot positions no longer references the screen center, so the
    payload flags it.
    """
    if panel_width_px is None or panel_height_px is None:
        panel_width_px, panel_height_px = viewport_width_px, viewport_height_px
    width_mm, height_mm = oriented_screen_mm(
        panel_width_px, panel_height_px, portrait_width_mm, portrait_height_mm
    )
    mm_per_px_x = width_mm / panel_width_px
    mm_per_px_y = height_mm / panel_height_px

    dots, meta = build_mire_layout(
        viewport_width_px, viewport_height_px, mm_per_px_x, mm_per_px_y
    )
    fullscreen_ok = (
        abs(viewport_width_px - panel_width_px) <= max(2, 0.01 * panel_width_px)
        and abs(viewport_height_px - panel_height_px) <= max(2, 0.01 * panel_height_px)
    )
    return {
        "created_at": datetime.now().isoformat(timespec="milliseconds"),
        "pattern": {
            "rows": ROWS,
            "cols": COLS,
            "missing_dot": {"row": MISSING_DOT[0], "col": MISSING_DOT[1]},
            "anchor_dot": {"row": ANCHOR_DOT[0], "col": ANCHOR_DOT[1]},
            "expected_dots": EXPECTED_DOTS,
        },
        "screen": {
            "viewport_px": {"width": viewport_width_px, "height": viewport_height_px},
            "panel_px": {"width": panel_width_px, "height": panel_height_px},
            "size_mm": {"width": width_mm, "height": height_mm},
            "mm_per_px": {"x": mm_per_px_x, "y": mm_per_px_y},
            "fullscreen_ok": fullscreen_ok,
        },
        "render": {"blink_hz": blink_hz, "gradient_softness_percent": gradient_softness},
        "layout": meta,
        "dots": [dot.to_json() for dot in dots],
    }


class MireState:
    """Last layout actually served to the phone, shared with the collector."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._current: Optional[Dict[str, object]] = None

    def set(self, payload: Dict[str, object]) -> None:
        with self._lock:
            self._current = payload

    def get(self) -> Optional[Dict[str, object]]:
        with self._lock:
            return self._current


def make_handler(args: argparse.Namespace, state: MireState):
    class MireRequestHandler(BaseHTTPRequestHandler):
        def _send_json(self, payload: Dict[str, object], status: int = 200) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_page(self) -> None:
            try:
                body = PAGE_PATH.read_bytes()
            except OSError as exc:
                self._send_json({"error": f"cannot read {PAGE_PATH}: {exc}"}, 500)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                self._send_page()
                return
            if parsed.path == "/api/layout":
                query = parse_qs(parsed.query)

                def read_int(name: str) -> Optional[int]:
                    raw = query.get(name, [None])[0]
                    if raw is None:
                        return None
                    try:
                        value = int(round(float(raw)))
                    except ValueError:
                        return None
                    return value if value > 0 else None

                viewport_w = read_int("width_px")
                viewport_h = read_int("height_px")
                if viewport_w is None or viewport_h is None:
                    self._send_json(
                        {"error": "width_px and height_px (physical pixels) required"}, 400
                    )
                    return
                payload = compute_layout_payload(
                    viewport_w,
                    viewport_h,
                    read_int("panel_width_px"),
                    read_int("panel_height_px"),
                    args.screen_width_mm,
                    args.screen_height_mm,
                    args.blink_hz,
                    args.gradient_softness,
                )
                state.set(payload)
                self._send_json(payload)
                return
            if parsed.path == "/api/current_layout":
                payload = state.get()
                if payload is None:
                    self._send_json({"error": "no layout served yet"}, 404)
                    return
                self._send_json(payload)
                return
            self._send_json({"error": "not found"}, 404)

        def log_message(self, fmt: str, *log_args: object) -> None:
            sys.stderr.write(
                f"[{datetime.now().strftime('%H:%M:%S')}] {self.address_string()} "
                f"{fmt % log_args}\n"
            )

    return MireRequestHandler


def run_self_test() -> int:
    failures = 0

    def check(name: str, ok: bool) -> None:
        nonlocal failures
        print(f"{'ok ' if ok else 'FAIL'} {name}")
        if not ok:
            failures += 1

    # Landscape phone viewport: long mm side must follow the long px side.
    payload = compute_layout_payload(
        2712, 1220, 2712, 1220, DEFAULT_SCREEN_WIDTH_MM, DEFAULT_SCREEN_HEIGHT_MM, 6.0, 55
    )
    screen = payload["screen"]
    check("landscape maps 154.50 mm to width", screen["size_mm"]["width"] == 154.50)
    check("landscape fullscreen detected", screen["fullscreen_ok"] is True)
    check("19 dots served", len(payload["dots"]) == EXPECTED_DOTS)

    # Single source of truth: served dots == direct build_mire_layout output.
    mm_x = 154.50 / 2712
    mm_y = 69.55 / 1220
    dots, meta = build_mire_layout(2712, 1220, mm_x, mm_y)
    same = all(
        served["screen_px"]["x"] == dot.screen_x_px
        and served["screen_px"]["y"] == dot.screen_y_px
        and served["object_mm"]["x"] == dot.object_x_mm
        and served["object_mm"]["y"] == dot.object_y_mm
        for served, dot in zip(payload["dots"], dots)
    )
    check("served dots identical to build_mire_layout", same)
    check(
        "spacing_mm consistent",
        abs(payload["layout"]["spacing_x_mm"] - meta["spacing_x_mm"]) < 1e-12,
    )

    # Portrait orientation flips the mm mapping.
    portrait = compute_layout_payload(
        1220, 2712, 1220, 2712, DEFAULT_SCREEN_WIDTH_MM, DEFAULT_SCREEN_HEIGHT_MM, 6.0, 55
    )
    check("portrait maps 69.55 mm to width", portrait["screen"]["size_mm"]["width"] == 69.55)

    # Browser bar stealing pixels must be flagged as not fullscreen.
    cropped = compute_layout_payload(
        2712, 1100, 2712, 1220, DEFAULT_SCREEN_WIDTH_MM, DEFAULT_SCREEN_HEIGHT_MM, 6.0, 55
    )
    check("cropped viewport flagged", cropped["screen"]["fullscreen_ok"] is False)

    # mm/px must stay panel-based even when the viewport is cropped.
    check(
        "mm_per_px panel-based when cropped",
        abs(cropped["screen"]["mm_per_px"]["y"] - 69.55 / 1220) < 1e-12,
    )

    check("page file exists", PAGE_PATH.is_file())
    print("self-test " + ("ok" if failures == 0 else f"failed ({failures})"))
    return 0 if failures == 0 else 1


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (phone must reach it).")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument(
        "--screen-width-mm",
        type=float,
        default=DEFAULT_SCREEN_WIDTH_MM,
        help="Active display width in mm, portrait reference.",
    )
    parser.add_argument(
        "--screen-height-mm",
        type=float,
        default=DEFAULT_SCREEN_HEIGHT_MM,
        help="Active display height in mm, portrait reference.",
    )
    parser.add_argument("--blink-hz", type=float, default=6.0, help="Mire blink frequency.")
    parser.add_argument(
        "--gradient-softness",
        type=int,
        default=55,
        help="Dot radial gradient softness, 0 hard edge to 100 soft fade.",
    )
    parser.add_argument("--self-test", action="store_true", help="Run layout tests and exit.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return run_self_test()
    state = MireState()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args, state))
    print(
        f"Mire server on http://{args.host}:{args.port}/ "
        f"(screen {args.screen_width_mm} x {args.screen_height_mm} mm, "
        f"blink {args.blink_hz} Hz)"
    )
    print("Open this URL on the phone, then press 'Demarrer' for real fullscreen.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping mire server.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
