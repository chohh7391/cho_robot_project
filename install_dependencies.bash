#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="${1:-$(cd "${REPO_ROOT}/.." && pwd)}"
if [[ $# -gt 0 ]]; then
  shift
fi
ROSDEP_ARGS=("$@")
SKIP_KEYS=(
  libfranka
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

rosdep install \
  --from-paths "${ROSDEP_PATHS[@]}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO_NAME}" \
  --skip-keys "${SKIP_KEYS[@]}" \
  -y \
  "${ROSDEP_ARGS[@]}"
