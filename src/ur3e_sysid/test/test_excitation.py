import math

import pytest

from ur3e_sysid.excitation import ExcitationUnsafe, SweepParams, check_excitation

PI = math.pi


def test_safe_chirp_passes():
    p = SweepParams(signal="chirp", amplitude=0.02, f0=0.1, f1=3.0, duration=20.0)
    m = check_excitation(0.0, p, q_min=-PI, q_max=PI, velocity_limit=PI)
    assert m["peak_velocity"] < m["velocity_cap"]


def test_amplitude_outside_limits_rejected():
    p = SweepParams(signal="step", amplitude=0.5)
    with pytest.raises(ExcitationUnsafe):
        # center near the upper stop -> swing leaves the safe band
        check_excitation(PI - 0.2, p, q_min=-PI, q_max=PI, velocity_limit=PI, margin=0.1)


def test_chirp_peak_velocity_guard():
    # 2*pi*f1*A = 2*pi*8*0.1 ~= 5.03 rad/s >> 0.5*pi
    p = SweepParams(signal="chirp", amplitude=0.1, f0=0.1, f1=8.0, duration=10.0)
    with pytest.raises(ExcitationUnsafe):
        check_excitation(0.0, p, q_min=-PI, q_max=PI, velocity_limit=PI)


def test_ramp_velocity_guard():
    p = SweepParams(signal="ramp", velocity=2.0, distance=0.2)
    with pytest.raises(ExcitationUnsafe):
        check_excitation(0.0, p, q_min=-PI, q_max=PI, velocity_limit=PI)  # 2.0 > 0.5*pi


def test_params_validation():
    with pytest.raises(ExcitationUnsafe):
        SweepParams(signal="chirp", amplitude=0.02, f0=0.0, f1=3.0).validate()
    with pytest.raises(ExcitationUnsafe):
        SweepParams(signal="ramp", velocity=-1.0).validate()
    with pytest.raises(ExcitationUnsafe):
        SweepParams(signal="step", amplitude=0.0).validate()
