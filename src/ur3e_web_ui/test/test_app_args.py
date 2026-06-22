import pytest
from fastapi import HTTPException

from ur3e_web_ui.app import _strip_ros_args, build_parser
from ur3e_web_ui.app import CatchBallConfigRequest, _validate_ball_config


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
    p0, v0, flight_s = _validate_ball_config(
        CatchBallConfigRequest(p0=[-1.0, 1.5, 0.4], v0=[1.5, -1.0, 2.5]),
        require_all=True,
    )

    assert p0 == (-1.0, 1.5, 0.4)
    assert v0 == (1.5, -1.0, 2.5)
    assert flight_s is None  # omitted => leave the node's restart_after_s unchanged


def test_validate_ball_config_accepts_flight_time():
    _, _, flight_s = _validate_ball_config(
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
