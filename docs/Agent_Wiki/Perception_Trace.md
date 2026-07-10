# Perception Trace

## Purpose

Estimate the 3D position of a fast ball from DVXplorer event-camera data.

The current primary method is Trace: it measures the width of the event trail
left by the moving ball and converts that apparent diameter into depth.

## Read First

- `docs/Robot_Control/revue_perception_robuste_controle_fluide_2026-07-10.md`
- `wiki/perception/perception-robustness-flight-lifecycle.md`
- `README.md`
- `docs/trace_algorithm_explanation.html`
- `docs/Context/algo_trace_graph.html`
- `docs/Context/synthese_projet.md`
- `docs/Context/AGENT.md`

## Main Code

- `src/Ball_Tracking_Cpp/src/TraceAnalysis.cpp`: Trace ribbon fitting,
  supported edges, local width measurement, 3D conversion and outlier filters.
- `src/Ball_Tracking_Cpp/src/Gui.cpp`: event accumulation, runtime controls and
  visual diagnostics.
- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp`: ROS node loop,
  calibration selection and `BallState` publication.
- `src/Ball_Tracking_Cpp/src/Camera.cpp`: acquisition, filtering, undistortion,
  sampling and clustering.
- `src/Ball_Tracking_Cpp/src/BallTracker.cpp`: legacy circle fitting.
- `src/Ball_Tracking_Cpp/include/Ball_Tracking_Cpp/RegressionAccumulator.hpp`:
  trajectory regression helpers.

## Contracts

- Native output topic: `ball_state`.
- Message: `ur3e_catch_msgs/BallState`.
- `header.frame_id` must identify the camera frame, usually `camera_optical`.
- Position published to `BallState` is in meters.
- With ballistic regression enabled, raw Trace measurements use
  `ball_state_raw` and the 60 Hz fitted state uses `ball_state`.
- The legacy `ball_position_3d_mm` topic may exist for compatibility.

## Common Risks

- Radius/width pixel error creates large depth error.
- Current tracker/regression timestamps do not yet expose true source latency;
  `lead_time_s=0` is now the bring-up default from the 2026-07-10 review.
- Full-frame/manual ROI, fixed polarity and GUI-capped processing are current
  robustness/reproducibility gaps.
- Trace frame axes and camera frame axes must not be mixed with display axes.
- Calibration files are source data; do not rewrite them casually.
- Avoid touching raygui internals unless the bug is in GUI binding behavior.
