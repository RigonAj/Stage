# Extrinsic Calibration Runbook (Eye-To-Hand Session)

> Sources: camera-base calibration reference, 2026-06-12; calibration scripts code review, 2026-07-06
> Raw: [Camera-base calibration](../../docs/Robot_Control/ur3e_camera_base_calibration.md); [Session launcher](../../scripts/run_handeye_session.sh); [Solver](../../scripts/solve_handeye.py); [Collector](../../scripts/event_mire_calibration.py); [TF publisher](../../scripts/publish_camera_tf.py)

## Overview

Operator checklist for the physical eye-to-hand session that estimates
`T_base_camera` (fixed DVXplorer in the UR3e `base` frame) with the phone mire
mounted on `tool0`. The code path was verified on 2026-07-06: solver
conventions are locked by `solve_handeye.py --self-test`, collector rejection
gates by `event_mire_calibration.py --self-test`, and the web UI calibration
tab by its pytest suite. A French condensed version of this procedure lives at
[procedure_calibration_extrinseque.md](../../docs/Robot_Control/procedure_calibration_extrinseque.md).

## Prerequisites

- **Robot stack up** (`source env.sh; run` → ur3e_stack) so TF `base → tool0`
  and `/joint_states` are published. Collector and stack must share the same
  `ROS_DOMAIN_ID`.
- **DVXplorer plugged in via USB.** The collector opens it directly through
  `dv_processing` — no ROS camera driver is involved.
- **Intrinsics validated first.** Existing XMLs
  (`recordings/mire_calibration/intrinsics_from_mire*.xml`) are approximate
  (~0.49 px); run "Test calib" (F9) and "Test carré" (F10) and require distance
  and spacing errors < 1 % before the hand-eye session. Hand-eye never beats
  the intrinsics it uses.
- **Phone mounted on tool0** (Poco X7 Pro in the printed support): brightness
  100 % or DC dimming (AMOLED PWM floods the event camera otherwise), fixed
  60 Hz refresh, auto-brightness/always-on/notifications off, screen timeout
  "never". First time: verify the displayed dot spacing with a caliper in the
  page's "Mode mesure".

## Pre-Flight Checks

```bash
cd ~/Dv-Rosws/Dv-Rosws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 scripts/solve_handeye.py --self-test
python3 scripts/event_mire_calibration.py --self-test
```

Both must print `self-test ok`. Never trust the solver conventions (both
measured inputs inverted, both outputs read directly) without this test.

## Capture Session

```bash
scripts/run_handeye_session.sh
```

This starts the phone mire server on `:8081` and the collector in
`--external-mire` mode. The server restores the last valid fullscreen layout
from `recordings/mire_calibration/phone_mire_layout.json`; if no cache exists,
it uses the documented Poco X7 Pro 2712×1220 landscape profile. Therefore a
phone that is already displaying the mire does not need to reconnect or reload
after the PC-side server restarts. The printed `http://<ip>:8081/` remains
available when the operator wants to start or refresh the phone display. A
phone-reported non-fullscreen layout is still rejected because its metric
geometry is unsafe for calibration.

Per pose (target 15–20 accepted samples):

1. Web UI Calibration tab: "Go to next pose" (poses are saved joint
   configurations in `calibration/calibration_poses.json`, replayed
   identically — never Cartesian targets).
2. Wait until the robot is fully stopped.
3. Collector window: "Capture hand-eye" (F11). Shift+F11 removes the last
   sample.

Pose diversity is what constrains the rotation: tilt the screen toward the
camera at varied ±25–40° angles on all three axes, cover the camera field of
view and 2–3 distances, mire fully visible and sharp at each pose.

Samples are auto-rejected when dots are missing, the robot moved during the
240 ms accumulation window (> 0.1 mm / 0.02° TF drift), or the IPPE planar
ambiguity ratio is too low at low tilt (< 1.5 below 15° tilt). Session JSON:
`recordings/mire_calibration/handeye/handeye_samples_<stamp>.json` (meters;
records frames, joint positions and the intrinsics XML path).

## Solve

```bash
python3 scripts/solve_handeye.py \
    recordings/mire_calibration/handeye/handeye_samples_*.json \
    --output-yaml calibration/handeye_result.yaml
```

`calibration/handeye_result.yaml` is the path the web UI viewer
(`GET /api/calibration/camera`) and `publish_camera_tf.py` expect; the solver
creates the directory. Accept the result only if the report validation is
green on all of:

- both OpenCV solvers agree (calibrateRobotWorldHandEye SHAH vs
  calibrateHandEye PARK, few mm / tenths of a degree);
- per-pose residuals and leave-one-out translation stable (< ~2–3 mm);
- rotation-axis diversity sufficient;
- pixel RMS of the end-to-end reprojection < ~1–2 px (same order as the
  per-pose solvePnP RMS);
- estimated mire normal points into the screen; `T_tool0_mire` consistent with
  the CAD (< ~5 mm, < ~2–3°) and `t_base_camera` with a tape measure
  (< 2–3 cm).

## Publish And Validate

```bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

Publishes the static TF `base → camera_optical` (default child frame matches
the perception adapter default). `--with-mire` also publishes
`tool0 → screen_center`; `--write-xacro` emits a URDF fragment;
`--print-only` shows the command without running it.

Then validate integration:

- The reference doc §9 mentions `camera_optical_frame`; the code default is
  `camera_optical` and perception consumes `camera_optical` — keep the default
  unless deliberately renaming with `--child-frame`.
- `base` vs `base_link`: calibration publishes under `base` (UR convention);
  live catch works in `base_link` (rotated π about Z). The robot stack must be
  up so TF resolves `camera_optical → base_link`.
- Run the parity gate: `publish_frame=base_link` vs `publish_frame=camera_optical`
  must agree before trusting real camera perception (see
  [Camera And Hand-Eye Calibration](camera-and-handeye-calibration.md)).
- Check the camera frame overlay in the web viewer (Calibration tab).
- The Support3D ghost on `tool0` in the viewer is **display-only**: its pose
  lives in `src/ur3e_web_ui/ur3e_web_ui/static/models/support_mount.json`
  (in-plane clocking about tool0 Z set +90° on 2026-07-06 to match the
  physical mount; flip the yaw/x-offset signs there if it is the other way).
  It has zero effect on the solve — `T_tool0_mire` is co-solved by hand-eye —
  but it is the reference for the CAD-consistency validation glance.
- Do not move the camera afterwards; any bump invalidates the result.

## See Also

- [Camera And Hand-Eye Calibration](camera-and-handeye-calibration.md)
- [Frames And Transforms](frames-and-transforms.md)
- [Real Robot Bring-Up Runbook](../operations/real-robot-bringup-runbook.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
