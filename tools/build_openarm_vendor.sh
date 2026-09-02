#!/usr/bin/env bash
set -euo pipefail

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace_dir=$(cd "${project_dir}/../.." && pwd)

expected_ros2_sha=4e837e1d0dae692ff67b560b69d8d281d7a8d4ed
expected_can_sha=a30364622ca939b8c8a741167c317b3e95e38a49

check_only=false
if (( $# > 0 )); then
  if (( $# == 1 )) && [[ $1 == --check-only ]]; then
    check_only=true
  else
    echo "This helper accepts no colcon arguments; its package allowlist is not overridable." >&2
    exit 2
  fi
fi

check_revision() {
  local path=$1
  local expected=$2
  local actual
  local indexed
  local untracked

  if [[ ! -e "${project_dir}/${path}/.git" ]]; then
    echo "Missing ${path}; run git submodule update --init --recursive." >&2
    exit 1
  fi
  actual=$(git -C "${project_dir}/${path}" rev-parse HEAD)
  if [[ "${actual}" != "${expected}" ]]; then
    echo "Refusing unreviewed ${path} revision ${actual}; expected ${expected}." >&2
    exit 1
  fi
  indexed=$(git -C "${project_dir}" ls-files -s -- "${path}" | awk '$1 == "160000" {print $2}')
  if [[ "${indexed}" != "${expected}" ]]; then
    echo "Refusing ${path}: superproject gitlink ${indexed:-missing} does not match ${expected}." >&2
    exit 1
  fi
  if ! git -C "${project_dir}/${path}" diff --quiet --ignore-submodules=none -- ||
      ! git -C "${project_dir}/${path}" diff --cached --quiet --ignore-submodules=none --; then
    echo "Refusing dirty tracked files in ${path}; vendor checkouts must be pristine." >&2
    exit 1
  fi
  untracked=$(git -C "${project_dir}/${path}" ls-files --others --exclude-standard)
  if [[ -n "${untracked}" ]]; then
    echo "Refusing non-ignored untracked files in ${path}; vendor source checkouts must be clean." >&2
    exit 1
  fi
}

check_revision extern/openarm_ros2 "${expected_ros2_sha}"
check_revision extern/openarm_can "${expected_can_sha}"

if [[ "${check_only}" == true ]]; then
  echo "OpenArm vendor revisions, gitlinks, tracked/staged files, and non-ignored untracked files are valid."
  exit 0
fi

if ! cmake --find-package -DNAME=CLI11 -DCOMPILER_ID=GNU \
    -DLANGUAGE=CXX -DMODE=EXIST >/dev/null 2>&1; then
  echo "CLI11 is required. On Ubuntu 22.04: sudo apt-get install libcli11-dev" >&2
  exit 1
fi

cd "${workspace_dir}"
# ROS 2 setup files reference optional variables and are not nounset-clean.
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
set -u

# This allowlist is the build boundary: do not silently build the upstream
# bringup, MoveIt config, or metapackage.
exec colcon build --symlink-install --packages-select \
  openarm_can openarm_hardware
