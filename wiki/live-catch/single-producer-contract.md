# Single Producer Contract

> Sources: 2026-07-09 real Trace command test analysis; live_catch diagnostics implementation, 2026-07-09
> Raw: [Analyse pipeline commande](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md); [Procédure session réelle commandée](../../docs/Robot_Control/procedure_lancement_reel_trace_commande.md); [Diagnostics module](../../src/ur3e_live_catch/ur3e_live_catch/diagnostics.py); [Live catch node](../../src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py); [Stack script](../../scripts/launch_ur3e_virtual_ball_stack.sh)

## Overview

The live loop assumes exactly ONE producer per contract topic: one ball source
on `ball_state` and one `live_catch_node` on `catch_telemetry`. The 2026-07-09
first real Trace command test violated both at once and produced the two
classic symptoms; this page records the failure signature, the enforcement
added that day, and the rule for operators.

## The 2026-07-09 Incident

The operator started `live_catch.launch.py use_tracker:=true
use_ball_regression:=true enable_command:=true` while the virtual-ball stack
(driver + web UI + its own live_catch and test_ball) was still running:

- **Two `live_catch_node`** published `/catch_telemetry` at 60 Hz with opposite
  `command_enabled` — the Web UI command state flickered ON/OFF every frame.
  With a single node that boolean cannot oscillate: flicker = duplicate node.
- **Two `ball_state` producers**: the idle trigger-mode `test_ball_node` kept
  publishing `valid=false` heartbeats at 30 Hz between the regression node's
  60 Hz fits. Every interleaved invalid message triggered a controlled stop
  plus a full policy-state reset (`_controlled_stop` + `_reset_sim`), so the
  robot only ever twitched a few millimetres per throw.
- Duplicate `~/enable_command` and parameter services meant UI calls could
  land on either node.

## Enforcement (since 2026-07-09)

- `live_catch_node` runs an exclusive-producer watchdog every 2 s
  (`diagnostics.producer_conflict_warnings`): it logs `PRODUCER CONFLICT`
  errors on multiple ball-topic publishers, multiple telemetry publishers, or
  duplicate `live_catch_node` graph names.
- A **ball-topic conflict fails command emission closed**
  (`_commanding_allowed` returns false) until the extra producer stops.
  Telemetry/name conflicts warn without blocking, because the *other* node may
  be the commanding one.
- The Web UI detects `command_enabled` flapping (≥3 transitions in 2 s,
  `ur3e_web_ui/flapping.FlapDetector`) and shows `command: CONFLICT` plus a red
  `catch: CONFLICT` badge instead of a flickering state.
- `launch_ur3e_virtual_ball_stack.sh --stop` (and the pre-launch cleanup) now
  also kills stray `ball_tracking_cpp` talkers, `ball_regression_node`s and
  manual `live_catch.launch.py` sessions.

## Operator Rule

Never start a second `live_catch.launch.py` next to the stack. To use real
perception, swap the ball source **inside** the single stack:

```bash
ur3e_catch_stop
ur3e_catch_stack --real --tracker --hold-side left --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx
```

`--tracker` sets `use_test_ball:=false use_tracker:=true` and enables the
ballistic regression by default (`--no-regression` for raw-feed debug).
`--hold-side` (2026-07-09) drives the hoop TF side; before this the script
hardcoded the right-side `hoop_xyz`, silently overriding `hold_side:=left`.
The full ordered operator procedure is
[procedure_lancement_reel_trace_commande.md](../../docs/Robot_Control/procedure_lancement_reel_trace_commande.md).
Pre-command checks: `ros2 topic info /ball_state --verbose` and
`/catch_telemetry --verbose` must each show exactly one publisher, and the
live node log must be free of `PRODUCER CONFLICT`.

## See Also

- [Message Contracts And Topics](message-contracts-and-topics.md)
- [Safety And Commanding](safety-and-commanding.md)
- [Real Perception Trace Test Runbook](../perception/real-perception-trace-test.md)
- [Current Status And Blockers](current-status-and-blockers.md)
- [UR3e Web UI](../web-ui/ur3e-web-ui.md)
