#!/bin/bash

if [[ -n "${BASH_VERSION:-}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elif [[ -n "${ZSH_VERSION:-}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${(%):-%x}")" && pwd)"
else
  SCRIPT_DIR="$(pwd)"
fi
LOCAL_DEPS_PREFIX="${SCRIPT_DIR}/.deps/prefix"
DEPENDENCY_INSTALLER="${SCRIPT_DIR}/scripts/install_dependencies_ubuntu24.sh"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f "${ROS_SETUP}" ]; then
  source "${ROS_SETUP}"
fi

if [ -d "${LOCAL_DEPS_PREFIX}" ]; then
  export CMAKE_PREFIX_PATH="${LOCAL_DEPS_PREFIX}:${CMAKE_PREFIX_PATH}"
  export PKG_CONFIG_PATH="${LOCAL_DEPS_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH}"
fi

build() {
  colcon build \
    --symlink-install \
    --packages-select ur3e_catch_msgs && \
  colcon build \
    --symlink-install \
    --packages-select ball_tracking_cpp \
    --cmake-args \
    -DCMAKE_C_COMPILER=gcc-13 \
    -DCMAKE_CXX_COMPILER=g++-13 \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

run() {
  source install/setup.bash && ros2 run ball_tracking_cpp talker
}

calib() {
  python3 "${SCRIPT_DIR}/scripts/calibrate_intrinsics_from_mire.py" \
    --robust \
    --use-intrinsic-guess \
    --zero-tangent-dist \
    --fix-k3 \
    --output-xml "${SCRIPT_DIR}/recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml" \
    --output-json "${SCRIPT_DIR}/recordings/mire_calibration/intrinsics_from_mire_robust_constrained_report.json" \
    --camera-name DVXplorer_mire_robust \
    "$@"
}

calib_intrinsics() {
  calib "$@"
}

deps() {
  local action="${1:-check}"

  if [ "$#" -gt 0 ]; then
    shift
  fi

  case "${action}" in
    check|--check)
      "${DEPENDENCY_INSTALLER}" --check "$@"
      ;;
    install|--install)
      "${DEPENDENCY_INSTALLER}" --install "$@"
      ;;
    *)
      echo "Usage: deps [check|install] [installer options]"
      return 2
      ;;
  esac
}

deps_check() {
  deps check "$@"
}

deps_install() {
  deps install "$@"
}

compile_report() {
  "${SCRIPT_DIR}/scripts/compile_stage_summary.sh" "$@"
}

alias deps-check='deps_check'
alias deps-install='deps_install'
alias check-deps='deps_check'
alias install-deps='deps_install'
alias compile-report='compile_report'

export DV_ROSWS_ROOT="${DV_ROSWS_ROOT:-${SCRIPT_DIR}}"
export UR3E_STACK_SCRIPT="${UR3E_STACK_SCRIPT:-${DV_ROSWS_ROOT}/scripts/launch_ur3e_stack.sh}"
export UR3E_CATCH_STACK_SCRIPT="${UR3E_CATCH_STACK_SCRIPT:-${DV_ROSWS_ROOT}/scripts/launch_ur3e_virtual_ball_stack.sh}"

dvros_source() {
  local ros_setup="/opt/ros/humble/setup.bash"
  local dv_setup="${DV_ROSWS_ROOT}/install/setup.bash"

  if [[ -n "${ZSH_VERSION:-}" ]]; then
    ros_setup="/opt/ros/humble/setup.zsh"
    dv_setup="${DV_ROSWS_ROOT}/install/setup.zsh"
  fi

  if [[ ! -f "${ros_setup}" ]]; then
    echo "Missing ROS setup: ${ros_setup}" >&2
    return 1
  fi
  if [[ ! -f "${dv_setup}" ]]; then
    echo "Missing Dv-Rosws setup: ${dv_setup}" >&2
    echo "Build first: cd ${DV_ROSWS_ROOT} && colcon build --symlink-install --packages-select ur3e_rollout_replay ur3e_web_ui" >&2
    return 1
  fi

  source "${ros_setup}"
  source "${dv_setup}"
}

ur3e_ros() {
  dvros_source
}

ur3e_stack() {
  DV_ROSWS_ROOT="${DV_ROSWS_ROOT}" "${UR3E_STACK_SCRIPT}" "$@"
}

ur3e_stack_lan() {
  UR3E_UI_HOST="${UR3E_UI_HOST:-0.0.0.0}" \
    DV_ROSWS_ROOT="${DV_ROSWS_ROOT}" \
    "${UR3E_STACK_SCRIPT}" "$@"
}

ur3e_stop() {
  DV_ROSWS_ROOT="${DV_ROSWS_ROOT}" "${UR3E_STACK_SCRIPT}" --stop "$@"
}

ur3e_catch_stack() {
  DV_ROSWS_ROOT="${DV_ROSWS_ROOT}" "${UR3E_CATCH_STACK_SCRIPT}" "$@"
}

ur3e_catch_stop() {
  DV_ROSWS_ROOT="${DV_ROSWS_ROOT}" "${UR3E_CATCH_STACK_SCRIPT}" --stop "$@"
}

ur3e_ui() {
  dvros_source && ros2 run ur3e_web_ui ur3e_web_ui "$@"
}

ur3e_ui_lan() {
  local host="${UR3E_UI_HOST:-0.0.0.0}"
  local port="${UR3E_UI_PORT:-8080}"
  dvros_source && ros2 run ur3e_web_ui ur3e_web_ui --host "${host}" --port "${port}" "$@"
}

ur3e_install_web_app() {
  "${DV_ROSWS_ROOT}/scripts/install_ur3e_web_app.sh" "$@"
}

ur3e_ui_help() {
  ur3e_ui --help
}

ur3e_validate() {
  if [[ $# -eq 0 ]]; then
    set -- --episode 0
  fi
  dvros_source && ros2 run ur3e_rollout_replay ur3e_replay_validate "$@"
}

ur3e_replay_dry() {
  dvros_source && ros2 run ur3e_rollout_replay ur3e_replay_send \
    --episode 0 \
    --current-joints 0,0,0,0,0,0 \
    "$@"
}

ur3e_pkgs() {
  dvros_source && ros2 pkg list | grep -E '^(ur3e_web_ui|ur3e_rollout_replay)$'
}

ur3e_build() {
  (
    cd "${DV_ROSWS_ROOT}" &&
    dvros_source &&
    colcon build --symlink-install --packages-select ur3e_rollout_replay ur3e_web_ui "$@"
  )
}

ur3e_test() {
  (
    cd "${DV_ROSWS_ROOT}" &&
    dvros_source &&
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select ur3e_rollout_replay ur3e_web_ui "$@" &&
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test-result --verbose
  )
}

ur3e_fake_driver() {
  dvros_source && ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.0.5 \
    use_fake_hardware:=true \
    launch_rviz:=false \
    "$@"
}

ur3e_controllers() {
  dvros_source && ros2 control list_controllers
}

ur3e_joints_once() {
  dvros_source && ros2 topic echo /joint_states --once
}

clear() {
  rm -rf build install log && ccache -c
}
