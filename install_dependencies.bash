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
EXCLUDED_SOURCE_PACKAGES=(
  "${REPO_ROOT}/extern/mujoco_vendor"
)

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
  apt-get --simulate install -y \
    "ros-${ROS_DISTRO_NAME}-libfranka" \
    "ros-${ROS_DISTRO_NAME}-pinocchio"
else
  sudo apt update
  sudo apt install -y \
    "ros-${ROS_DISTRO_NAME}-libfranka" \
    "ros-${ROS_DISTRO_NAME}-pinocchio"
fi

rosdep install \
  "${ROSDEP_ARGS[@]}" \
  --from-paths "${ROSDEP_PATHS[@]}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO_NAME}" \
  --skip-keys "${SKIP_KEYS[*]}" \
  -y
