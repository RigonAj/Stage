#!/usr/bin/env bash
# Hand-eye capture session (plan step 4 of ur3e_camera_base_calibration.md,
# 6-Dof-Ur3e-Catch-a-ball repo): phone mire server + event-camera collector.
#
# Prerequisites, in this order:
#   1. Robot stack running (ur3e_stack from the 6-Dof repo) so TF base->tool0
#      is published; same ROS_DOMAIN_ID as this terminal.
#   2. Intrinsics validated (Test calib / Test carre: distance and spacing
#      errors < 1 %).
#   3. Phone mounted on tool0, brightness 100 % / DC dimming, fixed refresh
#      rate, screen timeout never; open the URL printed below and press
#      "Demarrer"; caliper-check the spacing in "Mode mesure" the first time.
#
# Per pose (15-20 poses, tilt +/-25-40 deg varying all three axes):
#   UI Calibration tab "Go to next pose" -> wait until the robot is fully
#   stopped -> "Capture hand-eye" in the collector window. Samples are
#   auto-rejected if the robot moved, dots are missing, or the IPPE
#   ambiguity is too high at low tilt.
#
# Then solve offline:
#   python3 scripts/solve_handeye.py recordings/mire_calibration/handeye/handeye_samples_*.json \
#       --output-yaml recordings/mire_calibration/handeye/handeye_result.yaml

set -euo pipefail
cd "$(dirname "$0")"

MIRE_PORT="${MIRE_PORT:-8081}"

if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    else
        echo "WARNING: ROS 2 not sourced; TF lookups will fail." >&2
    fi
fi

python3 serve_phone_mire.py --host 0.0.0.0 --port "$MIRE_PORT" &
MIRE_PID=$!
trap 'kill "$MIRE_PID" 2>/dev/null || true' EXIT

sleep 1
echo
echo "==============================================================="
for ip in $(hostname -I); do
    echo "  Phone mire page:  http://${ip}:${MIRE_PORT}/"
done
echo "==============================================================="
echo

python3 event_mire_calibration.py --external-mire "http://127.0.0.1:${MIRE_PORT}" "$@"
