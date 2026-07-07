"""Wiring contract for the racket hold side (right/left).

Source-text asserts (style of test_ball_regression_contract.py): the launch
hold_side argument with its Isaac-matched hoop TF defaults, and the deployed
model metadata annotation.
"""

import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
VIRTUAL_LAUNCH = PACKAGE_ROOT / "launch" / "virtual_ball_robot.launch.py"


def test_virtual_ball_launch_declares_hold_side():
    launch = VIRTUAL_LAUNCH.read_text(encoding="utf-8")

    assert '"hold_side"' in launch
    assert 'default_value="right"' in launch
    # Isaac-matched hoop TF per side: left = right rotated 180 deg about
    # wrist_3 Z, so x flips sign and the quaternion becomes 180 deg about Y.
    assert '"right": ("-0.5 0.0 0.0", "1.0 0.0 0.0 0.0")' in launch
    assert '"left": ("0.5 0.0 0.0", "0.0 1.0 0.0 0.0")' in launch
    # Explicit hoop_xyz/hoop_quat still override the side-derived defaults.
    assert 'or default_xyz' in launch
    assert 'or default_quat' in launch


def test_deployed_models_declare_hold_side():
    model_root = WORKSPACE_ROOT / "data" / "models"
    metadata_paths = sorted(model_root.glob("*/policy_metadata.json"))
    metadata_paths.append(model_root / "policy_metadata.json")

    for path in metadata_paths:
        if not path.is_file():
            continue
        metadata = json.loads(path.read_text(encoding="utf-8"))
        hold_side = metadata.get("hold_side", "right")
        assert hold_side in ("right", "left"), f"{path}: hold_side={hold_side!r}"
        disk_offset = metadata.get("disk_offset_wrist_3_link_m")
        if isinstance(disk_offset, list) and len(disk_offset) == 3:
            expected_sign = -1.0 if hold_side == "right" else 1.0
            assert float(disk_offset[0]) * expected_sign > 0.0, (
                f"{path}: disk offset x={disk_offset[0]} vs hold_side={hold_side}"
            )
