import math

from ur3e_sysid.recorder import Recorder, load_csv


def test_csv_roundtrip_and_command_interpolation(tmp_path):
    rec = Recorder("elbow_joint")
    # states at 100 Hz, commands at 50 Hz on the same clock.
    for i in range(11):
        t = i * 0.01
        rec.add_state(t, q=0.1 * i, qd=0.1, effort=2.0)
    rec.add_command(0.0, 0.0)
    rec.add_command(0.1, 1.0)  # linear 0 -> 1 over the window
    path = rec.save_csv(tmp_path / "elbow_joint_step.csv", f0=0.0, f1=0.0)
    cols = load_csv(path)
    assert cols["t"].size == 11
    # q_cmd interpolated onto state grid: at t=0.05 -> 0.5
    mid = int((cols["t"] == 0.05).argmax())
    assert math.isclose(cols["q_cmd"][mid], 0.5, abs_tol=1e-6)
    assert math.isclose(cols["q"][-1], 1.0, abs_tol=1e-9)


def test_save_meta(tmp_path):
    rec = Recorder("wrist_1_joint")
    p = rec.save_meta(tmp_path / "m.json", {"joint": "wrist_1_joint", "signal": "chirp"})
    assert p.is_file()
    assert "wrist_1_joint" in p.read_text()
