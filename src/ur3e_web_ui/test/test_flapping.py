"""FlapDetector rules (command_enabled telemetry sanity)."""

import pytest

from ur3e_web_ui.flapping import FlapDetector


def test_stable_signal_never_flaps():
    detector = FlapDetector(window_s=2.0, min_transitions=3)
    for i in range(100):
        assert detector.observe(True, now_s=i * 0.016) is False


def test_operator_toggle_does_not_flap():
    # One OFF->ON transition (the UI button) is normal.
    detector = FlapDetector(window_s=2.0, min_transitions=3)
    detector.observe(False, 0.0)
    assert detector.observe(True, 1.0) is False
    assert detector.observe(True, 1.5) is False


def test_interleaved_dual_node_telemetry_flaps():
    # Two 60 Hz nodes with opposite states => a transition almost every sample.
    detector = FlapDetector(window_s=2.0, min_transitions=3)
    verdicts = [detector.observe(i % 2 == 0, now_s=i / 60.0) for i in range(12)]
    assert verdicts[-1] is True


def test_flapping_decays_once_stable():
    detector = FlapDetector(window_s=2.0, min_transitions=3)
    for i in range(12):
        detector.observe(i % 2 == 0, now_s=i / 60.0)
    assert detector.flapping(0.5) is True
    # After the window slides past the burst, the verdict clears.
    assert detector.flapping(3.0) is False
    assert detector.observe(True, 3.1) is False


def test_first_sample_is_not_a_transition():
    detector = FlapDetector(window_s=2.0, min_transitions=1)
    assert detector.observe(True, 0.0) is False
    assert detector.observe(False, 0.1) is True


@pytest.mark.parametrize("kwargs", [{"window_s": 0.0}, {"min_transitions": 0}])
def test_invalid_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        FlapDetector(**kwargs)
