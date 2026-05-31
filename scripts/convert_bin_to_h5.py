#!/usr/bin/env python3
"""Convert project EVT1 .bin event files to .h5.

The input format is the one used by EventWriter:
32-byte header followed by packed 13-byte events:
  x:int16, y:int16, polarity:uint8, timestamp:int64.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

try:
    import h5py
    import numpy as np
except ModuleNotFoundError as exc:
    missing = exc.name or "h5py/numpy"
    print(
        f"Missing Python dependency: {missing}\n"
        "Install it with one of these commands:\n"
        "  python3 -m pip install --user h5py numpy\n"
        "  python3 -m venv .venv && . .venv/bin/activate && python -m pip install h5py numpy",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


HEADER_SIZE = 32
EVENT_SIZE = 13
HEADER_STRUCT = struct.Struct("<4sIQqq")
EVENT_DTYPE = np.dtype(
    [
        ("x", "<i2"),
        ("y", "<i2"),
        ("polarity", "u1"),
        ("timestamp", "<i8"),
    ],
    align=False,
)


def read_evt1_bin(path: Path) -> tuple[dict[str, int | bytes], np.ndarray]:
    size = path.stat().st_size
    if size < HEADER_SIZE:
        raise ValueError(f"file is too small for an EVT1 header ({size} bytes)")

    payload_size = size - HEADER_SIZE
    if payload_size % EVENT_SIZE != 0:
        raise ValueError(
            f"payload size {payload_size} is not divisible by {EVENT_SIZE}; "
            "the file may not be an EVT1 .bin recording"
        )

    with path.open("rb") as file:
        raw_header = file.read(HEADER_SIZE)
        magic, version, header_count, t_start, t_end = HEADER_STRUCT.unpack(raw_header)
        if magic != b"EVT1":
            raise ValueError(f"unexpected magic {magic!r}, expected b'EVT1'")

        events = np.fromfile(file, dtype=EVENT_DTYPE, count=payload_size // EVENT_SIZE)

    if len(events) > 0:
        t_start = int(events["timestamp"][0])
        t_end = int(events["timestamp"][-1])

    header = {
        "magic": magic,
        "version": int(version),
        "event_count": int(len(events)),
        "header_event_count": int(header_count),
        "t_start": int(t_start),
        "t_end": int(t_end),
    }
    return header, events


def write_h5(path: Path, header: dict[str, int | bytes], events: np.ndarray, overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"{path} already exists; pass --overwrite to replace it")

    with h5py.File(path, "w") as h5:
        h5.attrs["magic"] = np.bytes_(header["magic"])
        h5.attrs["version"] = np.uint32(header["version"])
        h5.attrs["event_count"] = np.uint64(header["event_count"])
        h5.attrs["source_header_event_count"] = np.uint64(header["header_event_count"])
        h5.attrs["t_start"] = np.int64(header["t_start"])
        h5.attrs["t_end"] = np.int64(header["t_end"])
        h5.attrs["timestamp_unit"] = "microseconds"
        h5.attrs["source_format"] = "EVT1 bin"

        h5.create_dataset(
            "events",
            data=events,
            maxshape=(None,),
            chunks=True,
            compression="gzip",
            compression_opts=4,
            shuffle=True,
        )


def convert_file(input_path: Path, output_dir: Path | None, overwrite: bool) -> Path:
    header, events = read_evt1_bin(input_path)
    output_path = (output_dir or input_path.parent) / f"{input_path.stem}.h5"
    write_h5(output_path, header, events, overwrite)
    return output_path


def iter_inputs(paths: list[Path]) -> list[Path]:
    files: list[Path] = []
    for path in paths:
        if path.is_dir():
            files.extend(sorted(path.glob("*.bin")))
        elif path.suffix.lower() == ".bin":
            files.append(path)
    return files


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    default_recordings_dir = project_root / "recordings"
    parser = argparse.ArgumentParser(description="Convert EVT1 .bin event recordings to .h5.")
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        default=[default_recordings_dir],
        help="Input .bin files or directories containing .bin files. Default: recordings/.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="Directory where .h5 files will be written. Default: next to each .bin file.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Replace existing .h5 files.")
    args = parser.parse_args()

    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)

    inputs = iter_inputs([path.resolve() for path in args.paths])
    if not inputs:
        print("No .bin files found.", file=sys.stderr)
        return 1

    failures = 0
    for input_path in inputs:
        try:
            output_path = convert_file(input_path, args.output_dir, args.overwrite)
            print(f"{input_path.name} -> {output_path} ({output_path.stat().st_size} bytes)")
        except Exception as exc:
            failures += 1
            print(f"FAILED {input_path}: {exc}", file=sys.stderr)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
