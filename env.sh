#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_DEPS_PREFIX="${SCRIPT_DIR}/.deps/prefix"
DEPENDENCY_INSTALLER="${SCRIPT_DIR}/scripts/install_dependencies_ubuntu24.sh"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

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

alias deps-check='deps_check'
alias deps-install='deps_install'
alias check-deps='deps_check'
alias install-deps='deps_install'

clear() {
  rm -rf build install log && ccache -c
}
