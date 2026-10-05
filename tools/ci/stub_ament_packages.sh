#!/usr/bin/env bash
# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Register empty stand-ins for packages CI cannot install, one prefix each.
#
#   tools/ci/stub_ament_packages.sh <root> <package>...
#   export AMENT_PREFIX_PATH="$(tools/ci/stub_ament_packages.sh <root> pkg...):${AMENT_PREFIX_PATH}"
#
# Each <package> gets its own prefix, <root>/<package>, holding nothing but the
# ament index marker and empty share/<package> and lib/ directories, so
# get_package_share_directory() and get_package_prefix() resolve it. Prints the
# prefixes, colon-separated.
#
# Only for evaluating launch files without starting anything (the launch golden
# test): a launch that merely looks a package up - Franka's gz bringup reads
# franka_ign_ros2_control's lib/ for the Gazebo plugin path, its real bringup
# includes franka_gripper's gripper.launch.py by path - evaluates exactly as it
# would against the real package. Those packages come from extern/franka_ros2,
# which has no Humble debs and needs libfranka built from source. One prefix per
# package keeps the paths the golden output prints the same as in a workspace
# with an isolated install (<prefix:pkg>, <share:pkg>).
set -euo pipefail

if (( $# < 2 )); then
  echo "usage: $0 <root> <package>..." >&2
  exit 2
fi

root=$1
shift
mkdir -p "${root}"
root=$(cd "${root}" && pwd)

prefixes=()
for package in "$@"; do
  if [[ ! "${package}" =~ ^[a-z][a-z0-9_]*$ ]]; then
    echo "not a ROS package name: '${package}'" >&2
    exit 2
  fi
  prefix="${root}/${package}"
  mkdir -p "${prefix}/share/ament_index/resource_index/packages" "${prefix}/share/${package}" "${prefix}/lib"
  : > "${prefix}/share/ament_index/resource_index/packages/${package}"
  printf 'stand-in registered by tools/ci/stub_ament_packages.sh; holds nothing\n' \
    > "${prefix}/share/${package}/STUB"
  prefixes+=("${prefix}")
done

(IFS=:; echo "${prefixes[*]}")
