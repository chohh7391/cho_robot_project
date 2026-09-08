#!/usr/bin/env bash
# Enforce the colcon boundary around the cuRobo / cuMotion vendor submodules.
#
# WHY THIS EXISTS
# ---------------
# extern/isaac_ros_cumotion carries eleven packages; this project builds five.
# One of the six it does not build, curobo_core, *fails* a plain workspace build:
# it wants torch in the system interpreter and its own curobo/ submodule is
# empty. extern/curobo is worse -- it has no package.xml at all, so colcon
# identifies it by setup.py and tries to build the pip source tree.
#
# colcon's only discovery-level opt-out is a COLCON_IGNORE file inside the
# package directory, and those directories belong to submodules: a submodule
# records an upstream commit SHA, so a marker we create inside one is not ours
# to commit. Putting COLCON_IGNORE at the submodule root would hide the five
# packages we do want.
#
# So the markers cannot be tracked, but their creation can. Run this once after
# checking the submodules out. It is idempotent, and it verifies the result
# rather than trusting it.
#
#   tools/setup_curobo_vendor.sh              # create the markers and verify
#   tools/setup_curobo_vendor.sh --check-only # verify only, change nothing
#
# See docs/installation.md for the surrounding setup, and
# extern/VENDORED_CUROBO.md for the pinned revisions and the open item this
# works around.
set -euo pipefail

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# The five packages this project builds out of extern/isaac_ros_cumotion.
# Everything else in that submodule is excluded. Keep this list equal to the
# allowlist in docs/installation.md and todo/CUROBO_MOVEIT_TODO.md D4.
keep=(
  isaac_ros_cumotion
  isaac_ros_cumotion_interfaces
  isaac_ros_cumotion_python_utils
  isaac_ros_cumotion_robot_description
  isaac_ros_cumotion_moveit
)

# What colcon must see afterwards -- no more, no less. nvblox_msgs comes from
# the sparse checkout described in docs/installation.md, not from a submodule.
expected_packages=(
  isaac_ros_cumotion
  isaac_ros_cumotion_interfaces
  isaac_ros_cumotion_moveit
  isaac_ros_cumotion_python_utils
  isaac_ros_cumotion_robot_description
  nvblox_msgs
)

marker_note="\
Excluded from this workspace by tools/setup_curobo_vendor.sh: not used by
cho_robot_project, and curobo_core additionally fails to build here. This file
is not tracked - the enclosing directory is a vendor submodule. Re-run the
script after re-checking out the submodule.
"

check_only=false
if (( $# > 0 )); then
  if (( $# == 1 )) && [[ $1 == --check-only ]]; then
    check_only=true
  else
    echo "Usage: ${BASH_SOURCE[0]} [--check-only]" >&2
    exit 2
  fi
fi

require_checkout() {
  local path=$1
  local hint=$2
  if [[ ! -e "${project_dir}/${path}" ]]; then
    echo "Missing ${path}. ${hint}" >&2
    exit 1
  fi
}

require_checkout extern/curobo \
  "Run: git submodule update --init extern/curobo"
require_checkout extern/isaac_ros_cumotion \
  "Run: git submodule update --init extern/isaac_ros_cumotion"
require_checkout extern/nvblox_msgs_src/nvblox_msgs \
  "Take the sparse checkout described in docs/installation.md."

write_marker() {
  local dir=$1
  if [[ -f "${dir}/COLCON_IGNORE" ]]; then
    return 0
  fi
  if [[ "${check_only}" == true ]]; then
    echo "Missing marker: ${dir#"${project_dir}/"}/COLCON_IGNORE" >&2
    return 1
  fi
  printf '%s' "${marker_note}" > "${dir}/COLCON_IGNORE"
  echo "  created ${dir#"${project_dir}/"}/COLCON_IGNORE"
}

missing=0

# extern/curobo is a pip source tree, not a colcon package.
write_marker "${project_dir}/extern/curobo" || missing=1

# extern/isaac_ros_cumotion: exclude everything outside the allowlist.
for dir in "${project_dir}"/extern/isaac_ros_cumotion/*/; do
  [[ -f "${dir}/package.xml" ]] || continue
  name=$(basename "${dir}")
  skip=false
  for kept in "${keep[@]}"; do
    if [[ "${name}" == "${kept}" ]]; then
      skip=true
      break
    fi
  done
  [[ "${skip}" == true ]] && continue
  write_marker "${dir%/}" || missing=1
done

if (( missing )); then
  echo "Boundary is incomplete. Re-run without --check-only to create the markers." >&2
  exit 1
fi

# Verify against colcon itself rather than trusting the loop above.
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not on PATH; skipping verification. Source /opt/ros/humble/setup.bash first." >&2
  exit 0
fi

mapfile -t seen < <(
  colcon list --base-paths "${project_dir}/extern" 2>/dev/null \
    | awk '{print $1}' \
    | grep -E 'isaac|curobo|nvblox' \
    | sort
)
mapfile -t want < <(printf '%s\n' "${expected_packages[@]}" | sort)

if [[ "${seen[*]}" != "${want[*]}" ]]; then
  echo "Unexpected package set under extern/:" >&2
  printf '  want: %s\n' "${want[*]}" >&2
  printf '  seen: %s\n' "${seen[*]-<none>}" >&2
  exit 1
fi

echo "colcon boundary OK: ${#seen[@]} packages (${seen[*]})"
