#!/usr/bin/env bash
set -Eeuo pipefail

# Dependency installer for this ROS 2 Jazzy project on Ubuntu 24.04.
# References:
# - ROS 2 Jazzy deb install: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# - dv-processing apt install: https://dv-processing.inivation.com/2_0_0/installation.html

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LOCAL_PREFIX="${REPO_ROOT}/.deps/prefix"
LOCAL_SRC="${REPO_ROOT}/.deps/src"

MODE="check"
ASSUME_YES=0
FORCE_OS=0
SKIP_ROS_REPO=0
SKIP_DV_REPO=0
SKIP_RAYLIB_BUILD=0

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REQUIRED_UBUNTU_VERSION="24.04"
RAYLIB_VERSION="${RAYLIB_VERSION:-5.5}"

BASE_APT_PACKAGES=(
  build-essential
  ca-certificates
  cmake
  curl
  g++-13
  gcc-13
  git
  gnupg
  libasound2-dev
  libeigen3-dev
  libfmt-dev
  libgl1-mesa-dev
  libglu1-mesa-dev
  libhdf5-dev
  libopencv-dev
  libtbb-dev
  libusb-1.0-0-dev
  libx11-dev
  libxcursor-dev
  libxext-dev
  libxfixes-dev
  libxi-dev
  libxinerama-dev
  libxrandr-dev
  libxrender-dev
  lsb-release
  pkg-config
  python3-colcon-common-extensions
  python3-numpy
  python3-opencv
  python3-pyqt5
  software-properties-common
)

RAYLIB_BUILD_APT_PACKAGES=(
  mesa-common-dev
)

ROS_APT_PACKAGES=(
  "ros-${ROS_DISTRO}-ament-cmake"
  "ros-${ROS_DISTRO}-rclcpp"
  "ros-${ROS_DISTRO}-ros-base"
  "ros-${ROS_DISTRO}-std-msgs"
  ros-dev-tools
)

RAYLIB_APT_CANDIDATES=(
  libraylib-dev
  raylib-dev
)

missing_apt=()
missing_local=()
missing_manual=()
ok_items=()
need_ros_repo=0
need_dv_repo=0
need_raylib_build=0
os_ok=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--check|--install] [options]

Checks and installs dependencies for this project on Ubuntu 24.04.

Modes:
  --check              Only report missing dependencies. This is the default.
  --install           Run the check first, then install what is missing.

Options:
  -y, --yes           Do not prompt during apt installs.
  --force             Continue even if the OS is not Ubuntu 24.04.
  --no-ros-repo       Do not add the official ROS 2 apt source.
  --no-dv-repo        Do not add the iniVation dv-processing PPA.
  --no-raylib-build   Do not build raylib locally if no apt package exists.
  -h, --help          Show this help.

Environment:
  ROS_DISTRO          ROS distribution to check/install. Default: jazzy.
  RAYLIB_VERSION      Raylib tag to build locally. Default: 5.5.
EOF
}

log() {
  printf '[info] %s\n' "$*"
}

warn() {
  printf '[warn] %s\n' "$*" >&2
}

die() {
  printf '[error] %s\n' "$*" >&2
  exit 1
}

sudo_cmd() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

append_unique() {
  local item="$1"
  local -n array_ref="$2"
  local existing

  for existing in "${array_ref[@]}"; do
    [[ "${existing}" == "${item}" ]] && return 0
  done

  array_ref+=("${item}")
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

python_module_exists() {
  python3 - "$1" <<'PY' >/dev/null 2>&1
import importlib.util
import sys

sys.exit(0 if importlib.util.find_spec(sys.argv[1]) is not None else 1)
PY
}

apt_installed() {
  local status
  status="$(dpkg-query -W -f='${db:Status-Abbrev}' "$1" 2>/dev/null || true)"
  [[ "${status}" == ii* ]]
}

apt_candidate_exists() {
  local candidate
  candidate="$(apt-cache policy "$1" 2>/dev/null | awk '/Candidate:/ {print $2; exit}' || true)"
  [[ -n "${candidate}" && "${candidate}" != "(none)" ]]
}

pkg_config_exists() {
  PKG_CONFIG_PATH="${LOCAL_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}" \
    pkg-config --exists "$1" >/dev/null 2>&1
}

cmake_package_exists() {
  CMAKE_PREFIX_PATH="${LOCAL_PREFIX}:${CMAKE_PREFIX_PATH:-}" \
    cmake --find-package \
      -DNAME="$1" \
      -DCOMPILER_ID=GNU \
      -DLANGUAGE=CXX \
      -DMODE=EXIST >/dev/null 2>&1
}

local_file_exists() {
  [[ -e "${LOCAL_PREFIX}/$1" ]]
}

first_available_apt_candidate() {
  local pkg
  for pkg in "$@"; do
    if apt_installed "${pkg}" || apt_candidate_exists "${pkg}"; then
      printf '%s\n' "${pkg}"
      return 0
    fi
  done
  return 1
}

require_apt_package() {
  local pkg="$1"
  local label="${2:-$1}"

  if apt_installed "${pkg}"; then
    ok_items+=("${label}")
  else
    append_unique "${pkg}" missing_apt
  fi
}

check_os() {
  local os_id=""
  local os_version=""

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    os_id="${ID:-}"
    os_version="${VERSION_ID:-}"
  fi

  if [[ "${os_id}" == "ubuntu" && "${os_version}" == "${REQUIRED_UBUNTU_VERSION}" ]]; then
    os_ok=1
    ok_items+=("Ubuntu ${REQUIRED_UBUNTU_VERSION}")
    return 0
  fi

  os_ok=0
  missing_manual+=("Expected Ubuntu ${REQUIRED_UBUNTU_VERSION}, found ${os_id:-unknown} ${os_version:-unknown}. Use --force to continue anyway.")
}

check_base_packages() {
  local pkg

  for pkg in "${BASE_APT_PACKAGES[@]}"; do
    require_apt_package "${pkg}"
  done
}

check_ros() {
  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ok_items+=("ROS 2 ${ROS_DISTRO}")
  else
    missing_manual+=("ROS 2 ${ROS_DISTRO} setup file is missing at /opt/ros/${ROS_DISTRO}/setup.bash.")
    need_ros_repo=1
  fi

  local pkg
  for pkg in "${ROS_APT_PACKAGES[@]}"; do
    require_apt_package "${pkg}"
  done

  if command_exists colcon; then
    ok_items+=("colcon")
  else
    append_unique "python3-colcon-common-extensions" missing_apt
  fi
}

check_cpp_libraries() {
  if pkg_config_exists libusb-1.0; then
    ok_items+=("libusb pkg-config")
  else
    append_unique "libusb-1.0-0-dev" missing_apt
  fi

  if cmake_package_exists OpenCV; then
    ok_items+=("OpenCV CMake package")
  elif apt_installed libopencv-dev; then
    ok_items+=("libopencv-dev")
  else
    append_unique "libopencv-dev" missing_apt
  fi

  if cmake_package_exists Eigen3; then
    ok_items+=("Eigen3 CMake package")
  elif apt_installed libeigen3-dev; then
    ok_items+=("libeigen3-dev")
  else
    append_unique "libeigen3-dev" missing_apt
  fi

  if cmake_package_exists fmt; then
    ok_items+=("fmt CMake package")
  elif apt_installed libfmt-dev; then
    ok_items+=("libfmt-dev")
  else
    append_unique "libfmt-dev" missing_apt
  fi

  if cmake_package_exists TBB; then
    ok_items+=("TBB CMake package")
  elif apt_installed libtbb-dev; then
    ok_items+=("libtbb-dev")
  else
    append_unique "libtbb-dev" missing_apt
  fi

  if cmake_package_exists HDF5; then
    ok_items+=("HDF5 CMake package")
  elif apt_installed libhdf5-dev; then
    ok_items+=("libhdf5-dev")
  else
    append_unique "libhdf5-dev" missing_apt
  fi
}

check_dv_processing() {
  if cmake_package_exists dv-processing || pkg_config_exists dv-processing; then
    ok_items+=("dv-processing")
  else
    need_dv_repo=1
    append_unique "dv-processing" missing_apt
    missing_manual+=("dv-processing was not found by CMake/pkg-config.")
  fi

  if python_module_exists dv_processing; then
    ok_items+=("dv_processing Python module")
  else
    need_dv_repo=1
    append_unique "dv-processing-python" missing_apt
    missing_manual+=("Python module dv_processing is required by scripts/event_mire_calibration.py.")
  fi
}

check_raylib() {
  local raylib_pkg=""

  if pkg_config_exists raylib || cmake_package_exists raylib || local_file_exists "lib/pkgconfig/raylib.pc"; then
    ok_items+=("raylib")
    return 0
  fi

  if raylib_pkg="$(first_available_apt_candidate "${RAYLIB_APT_CANDIDATES[@]}")"; then
    append_unique "${raylib_pkg}" missing_apt
    return 0
  fi

  need_raylib_build=1
  missing_local+=("raylib ${RAYLIB_VERSION} will be built into ${LOCAL_PREFIX}.")

  local pkg
  for pkg in "${RAYLIB_BUILD_APT_PACKAGES[@]}"; do
    require_apt_package "${pkg}"
  done
}

run_checks() {
  missing_apt=()
  missing_local=()
  missing_manual=()
  ok_items=()
  need_ros_repo=0
  need_dv_repo=0
  need_raylib_build=0

  check_os

  command_exists apt-get || die "apt-get is required."
  command_exists dpkg-query || die "dpkg-query is required."

  check_base_packages
  check_ros
  check_cpp_libraries
  check_dv_processing
  check_raylib
}

print_report() {
  printf '\nDependency check for %s\n' "${REPO_ROOT}"
  printf 'Target: Ubuntu %s, ROS_DISTRO=%s\n\n' "${REQUIRED_UBUNTU_VERSION}" "${ROS_DISTRO}"

  if ((${#ok_items[@]} > 0)); then
    printf 'Found:\n'
    printf '  - %s\n' "${ok_items[@]}"
    printf '\n'
  fi

  if ((${#missing_apt[@]} > 0)); then
    printf 'Missing apt packages:\n'
    printf '  - %s\n' "${missing_apt[@]}"
    printf '\n'
  fi

  if ((${#missing_local[@]} > 0)); then
    printf 'Missing local builds:\n'
    printf '  - %s\n' "${missing_local[@]}"
    printf '\n'
  fi

  if ((${#missing_manual[@]} > 0)); then
    printf 'Notes:\n'
    printf '  - %s\n' "${missing_manual[@]}"
    printf '\n'
  fi

  if ((${#missing_apt[@]} == 0 && ${#missing_local[@]} == 0 && ${#missing_manual[@]} == 0)); then
    printf 'All checked dependencies are present.\n\n'
  fi
}

apt_install() {
  local packages=("$@")
  ((${#packages[@]} > 0)) || return 0

  if ((ASSUME_YES)); then
    sudo_cmd apt-get install -y "${packages[@]}"
  else
    sudo_cmd apt-get install "${packages[@]}"
  fi
}

ensure_ros_repo() {
  ((need_ros_repo)) || return 0
  ((SKIP_ROS_REPO)) && {
    warn "ROS repository setup skipped by --no-ros-repo."
    return 0
  }

  if apt_candidate_exists "ros-${ROS_DISTRO}-ros-base"; then
    return 0
  fi

  log "Adding official ROS 2 apt source."
  apt_install curl software-properties-common
  sudo_cmd add-apt-repository -y universe

  local codename
  local version
  codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME:-noble}}")"
  version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  [[ -n "${version}" ]] || die "Could not resolve latest ros-apt-source release."

  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${codename}_all.deb"
  sudo_cmd dpkg -i /tmp/ros2-apt-source.deb
}

ensure_dv_repo() {
  ((need_dv_repo)) || return 0
  ((SKIP_DV_REPO)) && {
    warn "dv-processing repository setup skipped by --no-dv-repo."
    return 0
  }

  if apt_candidate_exists dv-processing; then
    return 0
  fi

  log "Adding iniVation dv-processing apt repositories."
  apt_install software-properties-common
  sudo_cmd add-apt-repository -y ppa:ubuntu-toolchain-r/test
  sudo_cmd add-apt-repository -y ppa:inivation-ppa/inivation
}

install_apt_dependencies() {
  local apt_args=("${missing_apt[@]}")

  ((${#apt_args[@]} > 0)) || return 0
  log "Updating apt package lists."
  sudo_cmd apt-get update

  log "Installing missing apt packages."
  apt_install "${apt_args[@]}"
}

build_raylib_local() {
  ((need_raylib_build)) || return 0
  ((SKIP_RAYLIB_BUILD)) && {
    warn "raylib local build skipped by --no-raylib-build."
    return 0
  }

  log "Building raylib ${RAYLIB_VERSION} into ${LOCAL_PREFIX}."
  mkdir -p "${LOCAL_SRC}" "${LOCAL_PREFIX}"

  if [[ ! -d "${LOCAL_SRC}/raylib/.git" ]]; then
    git clone --depth 1 --branch "${RAYLIB_VERSION}" \
      https://github.com/raysan5/raylib.git "${LOCAL_SRC}/raylib"
  else
    git -C "${LOCAL_SRC}/raylib" fetch --tags --depth 1 origin "${RAYLIB_VERSION}"
    git -C "${LOCAL_SRC}/raylib" checkout -q "${RAYLIB_VERSION}"
  fi

  cmake -S "${LOCAL_SRC}/raylib" -B "${LOCAL_SRC}/raylib/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${LOCAL_PREFIX}" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GAMES=OFF
  cmake --build "${LOCAL_SRC}/raylib/build" --parallel
  cmake --install "${LOCAL_SRC}/raylib/build"
}

install_missing() {
  if ((os_ok == 0 && FORCE_OS == 0)); then
    die "OS check failed. Re-run with --force only if you know this system is compatible."
  fi

  if [[ "${EUID}" -ne 0 ]] && ! command_exists sudo; then
    die "sudo is required for system package installation."
  fi

  sudo_cmd true

  log "Updating apt package lists."
  sudo_cmd apt-get update

  ensure_ros_repo
  ensure_dv_repo

  # Recompute after repository setup, because ROS/dv/raylib candidates can appear.
  run_checks
  install_apt_dependencies
  build_raylib_local

  run_checks
  print_report

  if ((${#missing_apt[@]} > 0 || ${#missing_local[@]} > 0)); then
    die "Some dependencies are still missing. See the report above."
  fi
}

while (($# > 0)); do
  case "$1" in
    --check)
      MODE="check"
      ;;
    --install)
      MODE="install"
      ;;
    -y|--yes)
      ASSUME_YES=1
      ;;
    --force)
      FORCE_OS=1
      ;;
    --no-ros-repo)
      SKIP_ROS_REPO=1
      ;;
    --no-dv-repo)
      SKIP_DV_REPO=1
      ;;
    --no-raylib-build)
      SKIP_RAYLIB_BUILD=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "Unknown option: $1"
      ;;
  esac
  shift
done

run_checks
print_report

case "${MODE}" in
  check)
    if ((${#missing_apt[@]} > 0 || ${#missing_local[@]} > 0 || (${#missing_manual[@]} > 0 && FORCE_OS == 0))); then
      exit 1
    fi
    ;;
  install)
    install_missing
    ;;
esac
