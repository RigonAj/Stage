import math

from ur3e_sysid import signals


def test_chirp_phase_starts_at_zero_and_increases():
    f0, f1, T = 0.2, 4.0, 20.0
    assert signals.chirp_phase(0.0, f0, f1, T) == 0.0
    prev = -1.0
    for k in range(1, 200):
        t = T * k / 200.0
        ph = signals.chirp_phase(t, f0, f1, T)
        assert ph > prev  # monotonic since f(t) > 0
        prev = ph


def test_chirp_instant_frequency_endpoints():
    f0, f1, T = 0.5, 6.0, 10.0
    assert math.isclose(signals.chirp_instant_frequency(0.0, f0, f1, T), f0)
    assert math.isclose(signals.chirp_instant_frequency(T, f0, f1, T), f1)


def test_peak_velocity_and_acceleration():
    assert math.isclose(signals.peak_velocity(0.1, 3.0), 2 * math.pi * 3.0 * 0.1)
    assert math.isclose(signals.peak_acceleration(0.1, 3.0), (2 * math.pi * 3.0) ** 2 * 0.1)


def test_chirp_value_at_zero_is_offset():
    assert math.isclose(signals.chirp_value(0.0, 1.234, 0.05, 0.1, 3.0, 20.0), 1.234)


def test_step_profile_shape():
    t, v = signals.step_profile(0.5, 0.05, dwell_s=1.0, rise_s=0.05, settle_s=2.0)
    assert v == [0.5, 0.5, 0.55, 0.55]
    assert t == [0.0, 1.0, 1.05, 3.05]
    assert all(t[i] < t[i + 1] for i in range(len(t) - 1))


def test_ramp_profile_move_time():
    t, v = signals.ramp_profile(0.0, 0.2, 0.1, dwell_s=0.5)
    assert math.isclose(t[-1] - t[1], 0.2 / 0.1)
    assert math.isclose(v[-1], 0.2)


def test_chirp_profile_length_and_rate():
    t, v = signals.chirp_profile(0.0, 0.05, 0.1, 3.0, 2.0, rate=60.0)
    assert len(t) == len(v) == 121  # 2 s * 60 Hz + 1
    assert math.isclose(t[1] - t[0], 1.0 / 60.0)


def test_embed_replaces_only_excited_joint():
    base = (0.0, -1.0, 1.0, -1.0, -1.0, 0.0)
    poses = signals.embed(base, 2, [1.1, 1.2])
    assert poses[0] == (0.0, -1.0, 1.1, -1.0, -1.0, 0.0)
    assert poses[1][2] == 1.2
    assert poses[1][0] == 0.0 and poses[1][5] == 0.0
