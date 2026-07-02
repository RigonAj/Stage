#!/usr/bin/env bash
# Launch the UR3e driver, live-catch virtual-ball chain, and web UI together.
#
# Default mode targets the real robot documented in docs/Robot_Control:
#   robot: 192.168.0.5, ROS PC/reverse IP: 192.168.0.3, UI: 127.0.0.1:8080
#
# The live-catch node starts in dry-run by default (enable_command=false). Use
# the web UI Test tab to throw the virtual ball and explicitly arm robot command.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DV_ROSWS_ROOT="${DV_ROSWS_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

ROBOT_IP="${UR3E_ROBOT_IP:-192.168.0.5}"
REVERSE_IP="${UR3E_REVERSE_IP:-192.168.0.3}"
CALIB_FILE="${UR3E_CALIBRATION_FILE:-$HOME/ur3e_calibration.yaml}"
USE_FAKE_HARDWARE="${UR3E_USE_FAKE_HARDWARE:-false}"
HEADLESS_MODE="${UR3E_HEADLESS_MODE:-false}"
CLEAN_EXISTING="${UR3E_CLEAN_EXISTING:-true}"
LAUNCH_MOVEIT="${UR3E_LAUNCH_MOVEIT:-true}"
LAUNCH_UI="${UR3E_LAUNCH_UI:-true}"
UI_HOST="${UR3E_UI_HOST:-127.0.0.1}"
UI_PORT="${UR3E_UI_PORT:-8080}"
MODEL_PATH="${UR3E_LIVE_MODEL_PATH:-}"
ACTION_MODE="${UR3E_ACTION_MODE:-faithful}"
ENABLE_COMMAND="${UR3E_ENABLE_COMMAND:-false}"
PUBLISH_FRAME="${UR3E_BALL_PUBLISH_FRAME:-base_link}"
TRIGGER_MODE="${UR3E_BALL_TRIGGER_MODE:-true}"
LAUNCH_LATENCY_REPORT="${UR3E_LAUNCH_LATENCY_REPORT:-false}"
PUBLISH_HOOP_TF="${UR3E_PUBLISH_HOOP_TF:-true}"
HOOP_XYZ="${UR3E_HOOP_XYZ:--0.5 0.0 0.0}"
HOOP_QUAT="${UR3E_HOOP_QUAT:-1.0 0.0 0.0 0.0}"

EXTRA_LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [options] [extra ros2 launch args...]

Starts, in one command:
  - UR3e driver / robot control
  - live-catch with test_ball_node in trigger mode
  - ur3e_web_ui for the Test tab

Options:
  --stop                 Stop existing UR3e driver/UI/live-catch processes and exit.
  --no-clean             Do not stop existing UR3e processes before launch.
  --fake                 Use fake hardware instead of the real robot.
  --real                 Use the real robot (default).
  --headless             Launch UR driver headless_mode:=true.
  --no-moveit           Do not launch MoveIt.
  --no-ui               Do not launch the web UI.
  --host HOST            UI host, default: $UI_HOST
  --port PORT            UI port, default: $UI_PORT
  --model-path PATH      Policy model path passed to live_catch_node.
  --enable-command       Start live_catch with enable_command:=true (robot may move).
  --latency              Start latency_report.
  --publish-hoop-tf      Publish wrist_3_link -> hoop_center static TF.
  --hoop-xyz "X Y Z"     Hoop translation in wrist_3_link, metres.
  --hoop-quat "X Y Z W"  Hoop quaternion in wrist_3_link.
  -h, --help             Show this help.

Environment:
  UR3E_ROBOT_IP, UR3E_REVERSE_IP, UR3E_CALIBRATION_FILE
  UR3E_USE_FAKE_HARDWARE, UR3E_HEADLESS_MODE, UR3E_CLEAN_EXISTING
  UR3E_LAUNCH_MOVEIT, UR3E_LAUNCH_UI, UR3E_UI_HOST, UR3E_UI_PORT
  UR3E_LIVE_MODEL_PATH, UR3E_ACTION_MODE, UR3E_ENABLE_COMMAND
  UR3E_BALL_PUBLISH_FRAME, UR3E_BALL_TRIGGER_MODE
  UR3E_LAUNCH_LATENCY_REPORT, UR3E_PUBLISH_HOOP_TF, UR3E_HOOP_XYZ, UR3E_HOOP_QUAT
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "$value" || "$value" == --* ]]; then
    echo "ERROR: $option expects a value." >&2
    usage >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stop)
      STOP_ONLY=true
      shift
      ;;
    --no-clean)
      CLEAN_EXISTING=false
      shift
      ;;
    --fake)
      USE_FAKE_HARDWARE=true
      shift
      ;;
    --real)
      USE_FAKE_HARDWARE=false
      shift
      ;;
    --headless)
      HEADLESS_MODE=true
      shift
      ;;
    --no-moveit)
      LAUNCH_MOVEIT=false
      shift
      ;;
    --no-ui)
      LAUNCH_UI=false
      shift
      ;;
    --host)
      require_value "$1" "${2:-}"
      UI_HOST="$2"
      shift 2
      ;;
    --port)
      require_value "$1" "${2:-}"
      UI_PORT="$2"
      shift 2
      ;;
    --model-path)
      require_value "$1" "${2:-}"
      MODEL_PATH="$2"
      shift 2
      ;;
    --enable-command)
      ENABLE_COMMAND=true
      shift
      ;;
    --latency)
      LAUNCH_LATENCY_REPORT=true
      shift
      ;;
    --publish-hoop-tf)
      PUBLISH_HOOP_TF=true
      shift
      ;;
    --hoop-xyz)
      require_value "$1" "${2:-}"
      HOOP_XYZ="$2"
      shift 2
      ;;
    --hoop-quat)
      require_value "$1" "${2:-}"
      HOOP_QUAT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      EXTRA_LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

source /opt/ros/humble/setup.bash
if [[ -f "$DV_ROSWS_ROOT/install/setup.bash" ]]; then
  source "$DV_ROSWS_ROOT/install/setup.bash"
else
  echo "ERROR: missing $DV_ROSWS_ROOT/install/setup.bash" >&2
  echo "Build first: cd $DV_ROSWS_ROOT && colcon build --symlink-install --packages-select ur3e_catch_msgs ur3e_live_catch ur3e_web_ui" >&2
  exit 1
fi

matching_pids() {
  ps -eo pid=,cmd= |
    awk '
      /ros2 launch ur3e_live_catch virtual_ball_robot.launch.py/ ||
      /ros2 launch ur_robot_driver ur_control.launch.py/ ||
      /\/opt\/ros\/humble\/lib\/ur_robot_driver\/ur_ros2_control_node/ ||
      /\/opt\/ros\/humble\/lib\/controller_manager\/ros2_control_node/ ||
      /\/opt\/ros\/humble\/lib\/ur_robot_driver\/dashboard_client/ ||
      /\/opt\/ros\/humble\/lib\/ur_robot_driver\/robot_state_helper/ ||
      /\/opt\/ros\/humble\/lib\/ur_robot_driver\/controller_stopper_node/ ||
      /\/opt\/ros\/humble\/lib\/ur_robot_driver\/urscript_interface/ ||
      /\/opt\/ros\/humble\/lib\/robot_state_publisher\/robot_state_publisher/ ||
      /\/opt\/ros\/humble\/lib\/ur_robot_driver\/trajectory_until_node/ ||
      /ros2 launch ur_moveit_config ur_moveit.launch.py/ ||
      /\/opt\/ros\/humble\/lib\/moveit_ros_move_group\/move_group/ ||
      /ros2 run ur3e_web_ui ur3e_web_ui/ ||
      /\/ur3e_web_ui\/lib\/ur3e_web_ui\/ur3e_web_ui/ ||
      /\/ur3e_live_catch\/lib\/ur3e_live_catch\/live_catch_node/ ||
      /\/ur3e_live_catch\/lib\/ur3e_live_catch\/test_ball_node/ ||
      /\/ur3e_live_catch\/lib\/ur3e_live_catch\/latency_report/ {
        if ($0 !~ /awk / && $0 !~ /launch_ur3e_virtual_ball_stack/) print $1
      }'
}

stop_existing_stack() {
  local pids
  pids="$(matching_pids || true)"
  if [[ -z "$pids" ]]; then
    return 0
  fi

  echo "Stopping existing UR3e driver/UI/live-catch processes..."
  kill -TERM $pids 2>/dev/null || true
  sleep 2

  pids="$(matching_pids || true)"
  if [[ -n "$pids" ]]; then
    kill -KILL $pids 2>/dev/null || true
  fi

  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
}

if [[ "${STOP_ONLY:-false}" == "true" ]]; then
  stop_existing_stack
  echo "UR3e driver/UI/live-catch stopped."
  exit 0
fi

if [[ "$CLEAN_EXISTING" == "true" ]]; then
  stop_existing_stack
fi

if [[ "$USE_FAKE_HARDWARE" != "true" ]]; then
  if ! ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1; then
    echo "ERROR: robot $ROBOT_IP is not reachable. Check Ethernet or use --fake." >&2
    exit 1
  fi
fi

if [[ "$LAUNCH_UI" == "true" ]] && ss -ltn "sport = :$UI_PORT" | grep -q ":$UI_PORT"; then
  echo "ERROR: port $UI_PORT is already in use. Stop the old UI or pass --port." >&2
  exit 1
fi

launch_args=(
  "robot_ip:=$ROBOT_IP"
  "reverse_ip:=$REVERSE_IP"
  "use_fake_hardware:=$USE_FAKE_HARDWARE"
  "headless_mode:=$HEADLESS_MODE"
  "launch_moveit:=$LAUNCH_MOVEIT"
  "launch_ui:=$LAUNCH_UI"
  "ui_host:=$UI_HOST"
  "ui_port:=$UI_PORT"
  "action_mode:=$ACTION_MODE"
  "enable_command:=$ENABLE_COMMAND"
  "publish_frame:=$PUBLISH_FRAME"
  "trigger_mode:=$TRIGGER_MODE"
  "launch_latency_report:=$LAUNCH_LATENCY_REPORT"
  "publish_hoop_tf:=$PUBLISH_HOOP_TF"
  "hoop_xyz:=$HOOP_XYZ"
  "hoop_quat:=$HOOP_QUAT"
)

if [[ -n "$MODEL_PATH" ]]; then
  launch_args+=("model_path:=$MODEL_PATH")
fi
if [[ -f "$CALIB_FILE" ]]; then
  launch_args+=("kinematics_params_file:=$CALIB_FILE")
else
  echo "WARNING: no calibration file at $CALIB_FILE; using default ur_description kinematics."
fi
launch_args+=("${EXTRA_LAUNCH_ARGS[@]}")

cat <<EOF
Launching UR3e virtual-ball test stack:
  robot_ip=$ROBOT_IP reverse_ip=$REVERSE_IP fake_hardware=$USE_FAKE_HARDWARE
  ui=http://$UI_HOST:$UI_PORT launch_ui=$LAUNCH_UI launch_moveit=$LAUNCH_MOVEIT
  virtual_ball trigger_mode=$TRIGGER_MODE publish_frame=$PUBLISH_FRAME
  enable_command=$ENABLE_COMMAND

Stop with Ctrl+C in this terminal, or run:
  $(basename "$0") --stop
EOF

if [[ "$ENABLE_COMMAND" == "true" ]]; then
  echo
  echo "WARNING: enable_command=true; the robot may move when live-catch commands are valid." >&2
fi

exec ros2 launch ur3e_live_catch virtual_ball_robot.launch.py "${launch_args[@]}"
