# Trace Ball Tracking

> Sources: Repository README, 2026-06-29; project synthesis, 2026-06-29; trace pose publication, 2026-07-03
> Raw: [README](../../README.md); [Synthese projet](../../docs/Context/synthese_projet.md); [Publisher node](../../src/Ball_Tracking_Cpp/src/publisher_member_function.cpp)

## Overview

`Ball_Tracking_Cpp` estimates the 3D position of a fast ball from DVXplorer
events. The primary algorithm is Trace: instead of fitting a circle to a
motion-blurred event trail, it measures the trail width perpendicular to motion
and converts that apparent diameter into depth.

## Pipeline

1. Acquire events from the DVXplorer or recorded/simulated files.
2. Filter background activity and undistort event coordinates.
3. Accumulate recent events around the moving ball.
4. Estimate trail direction with global and local PCA.
5. Project events into local `s/h` trace coordinates.
6. Detect supported upper and lower trace edges.
7. Fit ribbon curves and measure local trail width.
8. Convert width plus center point to 3D position.
9. Filter 3D outliers, fit trajectory and publish ROS output.

## Main Code

- `src/Ball_Tracking_Cpp/src/Gui.cpp`: Trace fitting and visual diagnostics.
- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp`: ROS node loop and
  `BallState` publication.
- `src/Ball_Tracking_Cpp/src/Camera.cpp`: acquisition, filtering, undistortion
  and clustering.
- `src/Ball_Tracking_Cpp/src/BallTracker.cpp`: legacy circle fitting.
- `src/Ball_Tracking_Cpp/include/Ball_Tracking_Cpp/RegressionAccumulator.hpp`:
  trajectory regression helpers.

## Output Contract

- Native topic: `ball_state` (parameter `ball_state_topic`; the launch re-points
  it to `ball_state_raw` when the regression publisher is enabled).
- Message: `ur3e_catch_msgs/BallState`.
- Position is in meters, `header.stamp` is event time.
- `header.frame_id` must be nonempty and match the camera TF frame.
- `pose_source` parameter selects the published estimate:
  - `"trace"` (bring-up config): the outlier-filtered mid-window pose from the
    Trace ribbon pipeline (`Trace3DAnalysis.currentWorld`), stamped at that
    sample's own event time and deduplicated by requiring the stamp to advance.
    The internal remapped world frame is converted back to `camera_optical`
    before publishing. This makes the primary algorithm feed ROS and removes
    the dependency on the circle-fitting GUI toggle.
  - `"circle"` (code default): legacy per-detection circle-fit pose, previous
    behavior.
- Legacy `ball_position_3d_mm` may still exist, but it is a fallback path.
- The per-axis temporal regressions inside the tracker (`Update3DTrack`,
  `Gui::Draw3DScene` trace curve, `StabilizeTraceCurve`) remain GUI diagnostics
  only; trajectory fitting for deployment lives in the Stage
  `ball_regression_node` (see
  [Live Catch Loop](../live-catch/live-catch-loop.md)).

## Risks

Depth is highly sensitive to pixel-width error. Frame conventions also matter:
camera coordinates, display coordinates and robot base coordinates must not be
mixed silently.

## See Also

- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
