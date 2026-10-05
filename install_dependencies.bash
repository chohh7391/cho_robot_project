#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="$(cd "${REPO_ROOT}/.." && pwd)"
# A non-option first argument selects the workspace source directory.
if [[ $# -gt 0 && "${1}" != -* ]]; then
  WORKSPACE_SRC="${1}"
  shift
fi
ROSDEP_ARGS=("$@")
SIMULATE_APT=0
for rosdep_arg in "${ROSDEP_ARGS[@]}"; do
  if [[ "${rosdep_arg}" == "--simulate" || "${rosdep_arg}" == "-s" ]]; then
    SIMULATE_APT=1
    break
  fi
done
SKIP_KEYS=(
  # Install this ABI-compatible pair explicitly below rather than through rosdep.
  libfranka
  pinocchio
  # The external OpenArm packages declare this absent source dependency.
  openarm_description
)
# Installed by apt directly rather than resolved by rosdep, for two unrelated
# reasons:
#
#  - libfranka and Pinocchio are an ABI-compatible pair. They are pinned to the
#    ROS distribution's build of each and their rosdep keys are skipped above,
#    so the selection stays in one place.
#
#  - CLI11 is an upstream manifest omission, not a pin. extern/openarm_can 1.3.4
#    builds its openarm-can-cli tool unconditionally and so does
#    find_package(CLI11 REQUIRED) (CMakeLists.txt:129), but its package.xml
#    declares no dependencies at all beyond ament_cmake. rosdep therefore has
#    nothing to resolve, and a real OpenArm MIT build fails at CMake configure
#    time without it. Unlike the pair above this is a plain Ubuntu package, not a
#    ros-<distro> one: libcli11-dev, in jammy/universe.
APT_PACKAGES=(
  "ros-${ROS_DISTRO_NAME}-libfranka"
  "ros-${ROS_DISTRO_NAME}-pinocchio"
  libcli11-dev
)
EXCLUDED_SOURCE_PACKAGES=(
  "${REPO_ROOT}/extern/mujoco_vendor"
)
# The packages each extern/ submodule provides from source. `git pull` does not
# check out a submodule added since the clone (extern/bota_driver_ros2_example
# was), and a package that is not checked out is an unknown name to
# `rosdep --ignore-src`: bota_driver, bota_driver_example, openarm_can and the
# openarm_ros2 packages have no rosdep rule at all, and some franka_ros2 ones
# resolve to debs that do not exist. Missing submodules are therefore checked
# out below; one left out on purpose (SKIP_SUBMODULE_UPDATE=1) has these keys
# skipped, and the packages that need them will not build.
declare -A SUBMODULE_PACKAGES=(
  [extern/franka_ros2]="franka_bringup franka_example_controllers franka_fr3_moveit_config
    franka_gazebo_bringup franka_gripper franka_hardware franka_ign_ros2_control
    franka_mobile_example_controllers franka_mobile_sensors franka_msgs
    franka_robot_state_broadcaster franka_ros2 franka_semantic_components
    integration_launch_testing"
  [extern/mujoco_ros2_control]="mujoco_ros2_control mujoco_ros2_control_demos
    mujoco_ros2_control_msgs mujoco_ros2_control_plugins mujoco_ros2_control_tests"
  [extern/Universal_Robots_ROS2_Driver]="ur ur_bringup ur_calibration ur_controllers
    ur_dashboard_msgs ur_moveit_config ur_robot_driver"
  [extern/bota_driver_ros2]="bota_driver"
  [extern/bota_driver_ros2_example]="bota_driver_example"
  [extern/openarm_ros2]="openarm openarm_bimanual_moveit_config openarm_bringup openarm_hardware"
  [extern/openarm_can]="openarm_can"
)

# A submodule is checked out when its directory holds the .git file a checkout
# writes; an uninitialized one is an empty directory.
missing_submodules() {
  local path
  while read -r _ path; do
    if [[ ! -e "${REPO_ROOT}/${path}/.git" ]]; then
      echo "${path}"
    fi
  done < <(git config -f "${REPO_ROOT}/.gitmodules" --get-regexp '^submodule\..*\.path$')
}

MISSING_SUBMODULES=()
if [[ -f "${REPO_ROOT}/.gitmodules" ]]; then
  mapfile -t MISSING_SUBMODULES < <(missing_submodules)
fi
if (( ${#MISSING_SUBMODULES[@]} > 0 )); then
  if [[ "${SIMULATE_APT}" == "1" || "${SKIP_SUBMODULE_UPDATE:-0}" == "1" ]]; then
    echo "Submodules not checked out: ${MISSING_SUBMODULES[*]}" >&2
    echo "  To check them out: git -C ${REPO_ROOT} submodule update --init --recursive -- ${MISSING_SUBMODULES[*]}" >&2
  elif ! git -C "${REPO_ROOT}" rev-parse --git-dir > /dev/null 2>&1; then
    echo "Submodules not checked out and ${REPO_ROOT} is not a git checkout: ${MISSING_SUBMODULES[*]}" >&2
    echo "  Clone with --recursive, or set SKIP_SUBMODULE_UPDATE=1 to go on without them." >&2
    exit 1
  else
    # Only the missing ones: an initialized submodule is left at whatever
    # commit it is on, never moved back to the recorded one.
    echo "Checking out submodules: ${MISSING_SUBMODULES[*]}"
    if ! git -C "${REPO_ROOT}" submodule update --init --recursive -- "${MISSING_SUBMODULES[@]}"; then
      echo "Could not check out ${MISSING_SUBMODULES[*]} (network or access to its remote?)." >&2
      echo "  Retry, or set SKIP_SUBMODULE_UPDATE=1 to go on without them." >&2
      exit 1
    fi
    mapfile -t MISSING_SUBMODULES < <(missing_submodules)
  fi
fi
for submodule in "${MISSING_SUBMODULES[@]}"; do
  if [[ -z "${SUBMODULE_PACKAGES[${submodule}]+set}" ]]; then
    echo "Submodule ${submodule} is missing and install_dependencies.bash does not know its packages;" \
      "add it to SUBMODULE_PACKAGES." >&2
    exit 1
  fi
  read -r -d '' -a submodule_keys <<< "${SUBMODULE_PACKAGES[${submodule}]}" || true
  echo "Without ${submodule}: skipping the rosdep keys ${submodule_keys[*]}" >&2
  SKIP_KEYS+=("${submodule_keys[@]}")
done

is_excluded_package_dir() {
  local package_dir
  package_dir="$(realpath -m "$1")"

  local excluded_dir
  for excluded_dir in "${EXCLUDED_SOURCE_PACKAGES[@]}"; do
    excluded_dir="$(realpath -m "${excluded_dir}")"
    if [[ "${package_dir}" == "${excluded_dir}" ]]; then
      return 0
    fi
  done

  return 1
}

ROSDEP_PATHS=()
while IFS= read -r package_xml; do
  package_dir="$(dirname "${package_xml}")"
  if is_excluded_package_dir "${package_dir}"; then
    echo "Skipping source package for rosdep: ${package_dir}"
    continue
  fi
  ROSDEP_PATHS+=("${package_dir}")
done < <(
  find "${WORKSPACE_SRC}" \
    -name package.xml \
    -not -path '*/build/*' \
    -not -path '*/install/*' \
    -not -path '*/log/*' \
    -print
)

if [[ "${SKIP_ROSDEP_UPDATE:-0}" != "1" ]]; then
  rosdep update
fi

if [[ "${SIMULATE_APT}" == "1" ]]; then
  apt-get --simulate install -y "${APT_PACKAGES[@]}"
else
  sudo apt update
  sudo apt install -y "${APT_PACKAGES[@]}"
fi

rosdep install \
  "${ROSDEP_ARGS[@]}" \
  --from-paths "${ROSDEP_PATHS[@]}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO_NAME}" \
  --skip-keys "${SKIP_KEYS[*]}" \
  -y
