# Testing And Commands

> Sources: repository README quick start, 2026-07-16; project command reference, 2026-07-16; live-catch README, 2026-06-29; implementation status, 2026-06-30; web UI docs, 2026-06-30; user hardware report, 2026-07-02; hold-side variant and Isaac repo path check, 2026-07-06; stack --tracker option, 2026-07-09; real-ball perception diagnosis commands, 2026-07-16
> Raw: [README](../../README.md); [Command reference](../../docs/COMMANDS.md); [Live-catch README](../../src/ur3e_live_catch/README.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Web UI docs](../../docs/Robot_Control/ur3e_web_ui.md); [Stack script](../../scripts/launch_ur3e_virtual_ball_stack.sh)

## Root Command Reference

The root `README.md` is a short operator quick start with five entries:
standalone Trace perception, real UR3e + Web UI, publication/validation of the
hand-eye TF, the live-catch integration stack, and (since 2026-07-16) the
ordered **robot-disarmed real-ball perception test** — four terminals (TF,
`--tracker` stack with `--ball-radius 45.0`, raw boundary, fitted boundary),
startup log checks, heartbeat/validity acceptance criteria and the manual
REC recording with automatic timestamp archiving. The stack
distinguishes fake-hardware virtual-ball testing, virtual-ball testing on the
real UR3e, and real DVXplorer `--tracker` mode: `--tracker` replaces
`test_ball_node`, so the UI virtual-ball controls are intentionally disabled. The full command inventory
moved to `docs/COMMANDS.md`; it groups
environment/dependency setup, targeted builds, perception, intrinsic and
extrinsic calibration, robot/UI bring-up, single-stack live catch, dry-run
diagnostics, rollout replay, system identification, tests and wiki/report
maintenance. The topic runbooks linked in `## See Also` remain authoritative
for ordered physical procedures and acceptance gates.

The `env.sh` helper `build` is intentionally perception-only: it builds
`ur3e_catch_msgs`, then `ball_tracking_cpp` with GCC 13. The Python robot
packages (`ur3e_live_catch`, `ur3e_rollout_replay`, `ur3e_web_ui`,
`ur3e_sysid`) require the explicit `colcon build --symlink-install
--packages-select ...` command from the README.

## Environment

```bash
source env.sh
```

## Build

```bash
build
colcon build --packages-select ur3e_catch_msgs ur3e_live_catch
colcon build --symlink-install --packages-select ur3e_catch_msgs ur3e_live_catch ur3e_rollout_replay ur3e_web_ui
colcon build --packages-select ball_tracking_cpp
```

## Run

```bash
run
ur3e_stack
ur3e_catch_stack
```

### Commands From The 2026-07-16 Real-Ball Session

The first three README entries were launched separately during bring-up:

```bash
source env.sh
run

UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 ur3e_stack

python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

They start standalone perception, the real robot/Web UI and the hand-eye TF,
respectively. They do **not** start `live_catch_node`; the integrated inference
session must instead use the single live-catch stack below. Do not keep the
standalone tracker running beside `--tracker`, because that would create a
second perception producer.

The real-camera graph inspected later corresponds to this single-stack mode:

```bash
source env.sh
ur3e_catch_stop
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 \
  ur3e_catch_stack --real --tracker \
  --hold-side left \
  --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx
```

Commands entered or used during the diagnosis:

```bash
ros2 node list | grep -E 'live_catch|ball_regression|ball_tracking'
ros2 topic info /ball_state_raw --verbose
ros2 topic info /ball_state --verbose
ros2 topic info /catch_telemetry --verbose
ros2 topic echo /catch_telemetry --once
ros2 control list_controllers
ros2 param dump /ball_tracking_cpp
ros2 param dump /ball_regression_node
```

The graph was correct, but `/ball_state` was only the regression node's 60 Hz
`valid=false` heartbeat. Before another physical throw, use the Web UI
**Stop / back to safe** control and verify the command heartbeat, then inspect
the raw and fitted boundaries separately:

```bash
ros2 topic echo /catch_telemetry --once | grep command_enabled
# Expected: command_enabled: false

ros2 topic echo /ball_state_raw
ros2 topic echo /ball_state
```

The 2026-07-16 evidence showed no first valid raw Trace sample; changing
ball-regression gates cannot repair that upstream failure. See
[Real Perception Trace Test Runbook](../perception/real-perception-trace-test.md)
for the full diagnosis and perception-GUI checks.

Root cause fixed the same day: the tracker used to start in File/reader mode
(GUI default) and processed no camera events. New `ball_tracking_cpp`
parameters (`live_catch.yaml` defaults): `use_reader:=false` (live camera at
startup), `trace_polarity_mode:=all`, manual recording via the GUI REC toggle
(`record:=false` default, `record_file:=realtest.h5` target under
`recordings/`, existing non-empty targets archived with a timestamp suffix)
and `reader_file` for scripted replay. The tracker
now logs a 2 s `trace status` heartbeat with per-stage peaks. Offline replay
diagnostic (robot disarmed, no driver):

```bash
# Replay a recorded session through the tracker alone:
ros2 run ball_tracking_cpp talker --ros-args \
  --params-file src/ur3e_live_catch/config/live_catch.yaml \
  -p ball_state_topic:=ball_state_raw \
  -p use_reader:=true -p reader_file:=realtest_2026-07-09_backup.h5 -p record:=false

# Or through the launch (tracker + regression + live node dry-run):
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_tracker:=true use_ball_regression:=true enable_command:=false \
  use_reader:=true reader_file:=realtest_2026-07-09_backup.h5

# ROS-level capture during live sessions, next to the default H5 recording:
ros2 bag record -o rosbags/real_$(date +%Y%m%d_%H%M%S) \
  /ball_state_raw /ball_state /catch_telemetry /joint_states /tf /tf_static
```

`ur3e_catch_stack` is the one-command live-catch inference bring-up. It starts
the UR driver, MoveIt, `live_catch_node`, `test_ball_node` in trigger mode, the
Isaac-matched hoop TF and the Web UI. It defaults to dry-run
`enable_command=false`; command mode is enabled later from the Web UI Test tab
or with an explicit launch option.

Status note, 2026-07-02: user hardware validation confirmed that the real UR3e
can follow the virtual-ball policy stream and hold after the virtual ball
grounds. This is not the final real-ball deployment: the current response is
slow under bring-up limits (`v_safe_scale=0.5`), and watchdog, tuning, real
perception latency and camera/hoop TF validation remain open.

```bash
# Fake hardware + virtual ball + inference + UI.
ur3e_catch_stack --fake

# Real UR3e + virtual ball + inference + UI.
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 ur3e_catch_stack --real

# Explicit policy export.
ur3e_catch_stack --fake --model-path data/models/latest/policy_deterministic.onnx

# Real DVXplorer Trace perception INSIDE the single stack (since 2026-07-09):
# swaps test_ball_node for ball_tracking_cpp + ballistic regression. Never
# start a second live_catch.launch.py next to the stack instead — that
# duplicates live_catch_node and the ball_state producer
# (see wiki/live-catch/single-producer-contract.md).
ur3e_catch_stack --real --tracker --hold-side left --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx
# --hold-side (2026-07-09) drives the hoop TF side; the script previously
# hardcoded the right-side hoop_xyz, silently overriding hold_side:=left.
# Full ordered operator checklist:
# docs/Robot_Control/procedure_lancement_reel_trace_commande.md

# Stop the combined stack (also kills stray trackers / regression nodes /
# manual live_catch launches since 2026-07-09).
ur3e_catch_stop
```

Direct ROS equivalents:

```bash
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py use_fake_hardware:=true
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py \
  robot_ip:=192.168.0.5 reverse_ip:=192.168.0.3 use_fake_hardware:=false

# Racket mounted to the left (hoop TF at +0.5 m on wrist_3 X); must match the
# physical mount and the model's hold_side metadata.
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py \
  use_fake_hardware:=true hold_side:=left
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_test_ball:=true trigger_mode:=true publish_frame:=base_link enable_command:=false
ros2 service call /test_ball_node/throw std_srvs/srv/Trigger {}

# Ballistic-regression ball publisher (Isaac pop parity): raw sources move to
# ball_state_raw, the fitted BallState (position + velocity) lands on ball_state.
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_test_ball:=true trigger_mode:=true use_ball_regression:=true
```

Note: `test_ball_node` reads `noise_std`/`dropout_prob` once at startup, so a
noisy regression stress-test must set them at launch (or run the node with
`-p noise_std:=0.02 -p dropout_prob:=0.2`), not via `ros2 param set`.

Offline regression tuning on real captures (record during throws, then replay
with parameter overrides — no robot session needed):

```bash
BAG_DIRECTORY="recordings/ball_regression_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$BAG_DIRECTORY" /ball_state_raw /tf_static
# Stop recording with Ctrl+C, then:
python3 scripts/replay_ball_regression.py "$BAG_DIRECTORY" \
  --set depth_sigma_scale=8.0 --set max_rms_m=0.05
```

## Tests

```bash
(cd src/ur3e_live_catch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
(cd src/ur3e_rollout_replay && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
(cd src/ur3e_web_ui && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
(cd src/ur3e_sysid && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
```

## Isaac Sim2real Checks

On this PC the Isaac training repo lives at
`~/Documents/IsaacTrain/Cartpole/Cartpole/FirstTraining` (checked 2026-07-06;
the `~/Documents/6-Dof-Ur3e-Catch-a-ball` checkout named by older notes is
absent here). Train/play/evaluate/export details are compiled in
[Isaac Training Environment](../sim-to-real/isaac-training-environment.md).

```bash
cd ~/Documents/IsaacTrain/Cartpole/Cartpole/FirstTraining
source env.zsh   # replaces the former script.zsh
sim2real_export
sim2real_validate

# Left-hand (racket held left) variant: same commands, mirrored task.
train-left            # = FT_TASK=Template-Firsttraining-Direct-Left-v0 train
play-left best        # play pinned to the left task/checkpoint root
train-right           # explicit right-hand pin; plain train also defaults to right
FT_TASK=Template-Firsttraining-Direct-Left-v0 sim2real_export
```

The 2026-06-30 Stage model transfer used explicit exports for both selected
checkpoints (paths below are the historical locations used that day). `main`
keeps the model artifacts and metadata, but not the large
`rollouts_10_episodes.json` validation files:

```bash
play latest --headless --livestream 0 --rendering_mode performance \
  --export_policy --export_onnx \
  --export_dir=/home/rigon/Documents/Stage/Stage/data/models/latest

play best --headless --livestream 0 --rendering_mode performance \
  --export_policy --export_onnx \
  --export_dir=/home/rigon/Documents/Stage/Stage/data/models/best
```

Add `--record_actions --record_episodes=10` only when regenerating local rollout
validation files for replay or audit.

Deployment model directories such as `data/models/latest` and `data/models/best`
do not carry rollout JSON files. Validate their model files and metadata with:

```bash
python3 /home/rigon/Documents/6-Dof-Ur3e-Catch-a-ball/scripts/sim2real_validate_export.py \
  --exports data/models/latest --metadata-only
python3 /home/rigon/Documents/6-Dof-Ur3e-Catch-a-ball/scripts/sim2real_validate_export.py \
  --exports data/models/best --metadata-only
```

Run the same script without `--metadata-only` only on full export directories
that include `rollouts_*_episodes.json`.

## Wiki Maintenance

```bash
python3 scripts/lint_llm_wiki.py
python3 scripts/update_agent_wiki.py
```

## See Also

- [Real Robot Bring-Up Runbook](real-robot-bringup-runbook.md)
- [Real Perception Trace Test Runbook](../perception/real-perception-trace-test.md)
- [Intrinsic Calibration Runbook](../calibration/intrinsic-calibration-runbook.md)
- [Extrinsic Calibration Runbook](../calibration/extrinsic-calibration-runbook.md)
- [Rollout Replay And Driver Setup](../replay/rollout-replay-and-driver-setup.md)
- [UR3e Actuator Identification](../system-id/ur3e-actuator-identification.md)
- [Isaac Training Environment](../sim-to-real/isaac-training-environment.md)
- [Wiki Maintenance](wiki-maintenance.md)
- [Source Document Map](source-document-map.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
