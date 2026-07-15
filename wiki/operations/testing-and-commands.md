# Testing And Commands

> Sources: repository README, 2026-07-15; LaTeX compilation guide, 2026-07-15; live-catch README, 2026-06-29; implementation status, 2026-06-30; web UI docs, 2026-06-30; user hardware report, 2026-07-02; hold-side variant and Isaac repo path check, 2026-07-06; stack --tracker option, 2026-07-09
> Raw: [README](../../README.md); [LaTeX compilation guide](../../docs/latex_compilation.md); [Live-catch README](../../src/ur3e_live_catch/README.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Web UI docs](../../docs/Robot_Control/ur3e_web_ui.md); [Stack script](../../scripts/launch_ur3e_virtual_ball_stack.sh)

## Environment

```bash
source env.sh
```

## Internship Report

The report is compiled from the repository root with `latexmk`. A user-local
TeX Live installation works without `sudo`; add its binaries before sourcing
`env.sh` so that the `compile-report` alias can find `latexmk`:

```bash
export PATH="$HOME/.local/texlive/2026/bin/x86_64-linux:$PATH"
source env.sh
compile-report
```

This generates `Stage_summary.pdf` and removes the temporary LaTeX files. See
the [LaTeX compilation guide](../../docs/latex_compilation.md) for installation
alternatives and the direct script command.

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
ros2 bag record /ball_state_raw /tf_static
python3 scripts/replay_ball_regression.py <bag_dir> \
  --set depth_sigma_scale=8.0 --set max_rms_m=0.05
```

```bash
```

## Tests

```bash
cd src/ur3e_live_catch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_rollout_replay && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_web_ui && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_sysid && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
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
- [Isaac Training Environment](../sim-to-real/isaac-training-environment.md)
- [Wiki Maintenance](wiki-maintenance.md)
- [Source Document Map](source-document-map.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
