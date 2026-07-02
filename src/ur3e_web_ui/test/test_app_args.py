from pathlib import Path

import pytest
from fastapi import HTTPException

from ur3e_web_ui.app import _strip_ros_args, build_parser
from ur3e_web_ui.app import (
    CatchBallConfigRequest,
    _active_model_name,
    _discover_catch_models,
    _validate_ball_config,
)


def test_strip_ros_args_keeps_web_ui_options():
    argv = [
        "--host",
        "127.0.0.1",
        "--port",
        "8080",
        "--ros-args",
        "-r",
        "__node:=ur3e_web_ui",
    ]

    args = build_parser().parse_args(_strip_ros_args(argv))

    assert args.host == "127.0.0.1"
    assert args.port == 8080


def test_validate_ball_config_accepts_launch_point_and_velocity():
    p0, v0, gravity, flight_s = _validate_ball_config(
        CatchBallConfigRequest(p0=[-0.4, 1.65, 0.85], v0=[-0.05, -4.25, 0.7]),
        require_all=True,
    )

    assert p0 == (-0.4, 1.65, 0.85)
    assert v0 == (-0.05, -4.25, 0.7)
    assert gravity is None
    assert flight_s is None  # omitted => leave the node's restart_after_s unchanged


def test_validate_ball_config_accepts_isaac_velocity_and_gravity():
    p0, v0, gravity, flight_s = _validate_ball_config(
        CatchBallConfigRequest(
            p0=[-0.5, 1.35, 0.825],
            v0=[-0.2, -5.0, 0.025],
            gravity=[0.0, 0.0, -9.81],
            flight_s=4.0,
        ),
        require_all=True,
    )

    assert p0 == (-0.5, 1.35, 0.825)
    assert v0 == (-0.2, -5.0, 0.025)
    assert gravity == (0.0, 0.0, -9.81)
    assert flight_s == 4.0


def test_catch_panel_isaac_random_matches_firsttraining_ranges():
    source = (Path(__file__).parents[1] / "ur3e_web_ui" / "static" / "js" / "catch_panel.js").read_text()
    assert "p0Ranges: [[-0.6, -0.2], [1.2, 2.1], [0.5, 1.2]]" in source
    assert "v0Ranges: [[-0.7, 0.6], [-5.0, -3.5], [-0.1, 1.5]]" in source
    assert "positionNoiseStd: 0.01" in source


def test_validate_ball_config_accepts_flight_time():
    _, _, _, flight_s = _validate_ball_config(
        CatchBallConfigRequest(p0=[-1.0, 1.5, 0.4], v0=[1.5, -1.0, 2.5], flight_s=3.5),
        require_all=True,
    )

    assert flight_s == 3.5


def test_validate_ball_config_rejects_out_of_bounds_values():
    with pytest.raises(HTTPException):
        _validate_ball_config(
            CatchBallConfigRequest(p0=[-1.0, 1.5, -0.1], v0=[1.5, -1.0, 2.5]),
            require_all=True,
        )


def test_validate_ball_config_rejects_out_of_bounds_flight_time():
    with pytest.raises(HTTPException):
        _validate_ball_config(
            CatchBallConfigRequest(p0=[-1.0, 1.5, 0.4], v0=[1.5, -1.0, 2.5], flight_s=99.0),
            require_all=True,
        )


def _write_model(root, name, *, onnx=True, torch=True):
    directory = root / "data" / "models" / name
    directory.mkdir(parents=True)
    if onnx:
        (directory / "policy_deterministic.onnx").write_bytes(b"onnx")
    if torch:
        (directory / "policy_deterministic.ts").write_bytes(b"torch")
    (directory / "policy_metadata.json").write_text(
        '{"observation_space": 33, "action_space": 6, "action_semantics": "test"}'
    )
    return directory


def test_discover_catch_models_prefers_onnx(tmp_path):
    latest_dir = _write_model(tmp_path, "latest")
    best_dir = _write_model(tmp_path, "best", onnx=False, torch=True)

    models = _discover_catch_models(tmp_path)
    by_name = {model["name"]: model for model in models}

    assert set(by_name) == {"latest", "best"}
    assert by_name["latest"]["available"]
    assert by_name["latest"]["model_path"] == str(latest_dir / "policy_deterministic.onnx")
    assert by_name["best"]["available"]
    assert by_name["best"]["model_path"] == str(best_dir / "policy_deterministic.ts")


def test_active_model_name_defaults_to_latest(tmp_path):
    _write_model(tmp_path, "latest")
    _write_model(tmp_path, "best")
    models = _discover_catch_models(tmp_path)

    assert _active_model_name(models, "") == "latest"


def test_active_model_name_accepts_any_file_in_model_directory(tmp_path):
    _write_model(tmp_path, "latest")
    best_dir = _write_model(tmp_path, "best")
    models = _discover_catch_models(tmp_path)

    assert _active_model_name(models, str(best_dir / "policy_deterministic.ts")) == "best"
