#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/rigon/Dv-Rosws/Dv-Rosws"

cd "$WORKSPACE"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
fi

if command -v ros2 >/dev/null 2>&1; then
  exec ros2 run ball_tracking_cpp talker
fi

exec "$WORKSPACE/install/ball_tracking_cpp/lib/ball_tracking_cpp/talker"
