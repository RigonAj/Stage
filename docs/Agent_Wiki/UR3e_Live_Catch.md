# UR3e Live Catch

## Purpose

Run the low-latency loop that consumes ball position and robot joint state,
builds the policy observation, runs the exported policy and maps the action to a
safe UR3e command target.

## Read First

- [[Current_Status]]
- `src/ur3e_live_catch/README.md`
- `docs/Robot_Control/ur3e_live_catch_architecture.md`
- `docs/Robot_Control/ur3e_live_catch_implementation_status.md`
- `docs/Robot_Control/ur3e_ball_catch_sim_to_real.md`
- `docs/Robot_Control/ur3e_robot_control_architecture.md`

## Main Code

- `src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py`: 60 Hz node.
- `src/ur3e_live_catch/ur3e_live_catch/ball_frame.py`: `BallState` frame
  handling and velocity estimation.
- `src/ur3e_live_catch/ur3e_live_catch/observation.py`: 33-D observation.
- `src/ur3e_live_catch/ur3e_live_catch/policy_runtime.py`: TorchScript/ONNX
  policy runtime.
- `src/ur3e_live_catch/ur3e_live_catch/action.py`: action to joint target.
- `src/ur3e_live_catch/ur3e_live_catch/safety.py`: limiters and watchdog.
- `src/ur3e_live_catch/ur3e_live_catch/streaming.py`: controller command path.
- `src/ur3e_live_catch/config/live_catch.yaml`: runtime parameters.

## Safety Contracts

- `enable_command=false` is the safe default.
- The node must refuse command mode when no policy is loaded.
- The pipeline must reject empty or unknown `BallState.header.frame_id`.
- Position, velocity and acceleration limits remain independent from the policy.
- Real robot operation requires E-stop readiness and verified workspace bounds.

## Tests

```bash
cd src/ur3e_live_catch
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

## Related Notes

- [[Current_Status]]
- [[Robot_Control]]
- [[Sim_to_Real]]
- [[Web_UI]]
