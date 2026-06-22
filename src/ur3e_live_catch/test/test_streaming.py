"""CommandStreamer: format / interpolate / hold (archi §4.3.6, step 6)."""

import pytest

from ur3e_live_catch.joint_order import JOINT_ORDER
from ur3e_live_catch.streaming import CommandStreamer


def test_format_returns_six_values_in_order():
    s = CommandStreamer()
    cmd = s.format([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert cmd == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert len(cmd) == len(JOINT_ORDER) == 6
    assert s.last_command == pytest.approx(cmd)


def test_format_rejects_wrong_length():
    s = CommandStreamer()
    with pytest.raises(ValueError):
        s.format([0.0, 0.0, 0.0])


def test_stream_substeps_one_forwards_target():
    s = CommandStreamer(substeps=1)
    cmds = s.stream([1.0] * 6)
    assert cmds == [pytest.approx([1.0] * 6)]


def test_stream_interpolates_linearly():
    s = CommandStreamer(substeps=4)
    s.stream([0.0] * 6)             # first call: single command at the target
    cmds = s.stream([4.0] * 6)      # ramp 0 -> 4 in 4 substeps
    assert len(cmds) == 4
    # last substep reaches the target; steps are evenly spaced
    assert cmds[0] == pytest.approx([1.0] * 6)
    assert cmds[1] == pytest.approx([2.0] * 6)
    assert cmds[2] == pytest.approx([3.0] * 6)
    assert cmds[3] == pytest.approx([4.0] * 6)


def test_stream_first_call_is_single_command():
    s = CommandStreamer(substeps=8)
    cmds = s.stream([2.0] * 6)  # no previous command -> never ramp from unknown
    assert cmds == [pytest.approx([2.0] * 6)]


def test_hold_repeats_last_command():
    s = CommandStreamer()
    s.format([0.5] * 6)
    assert s.hold() == pytest.approx([0.5] * 6)


def test_hold_uses_fallback_when_no_command_yet():
    s = CommandStreamer()
    assert s.hold(fallback=[0.7] * 6) == pytest.approx([0.7] * 6)
    # after a fallback hold, that becomes the last command
    assert s.hold() == pytest.approx([0.7] * 6)


def test_hold_without_command_or_fallback_raises():
    s = CommandStreamer()
    with pytest.raises(ValueError):
        s.hold()


def test_invalid_substeps():
    with pytest.raises(ValueError):
        CommandStreamer(substeps=0)
