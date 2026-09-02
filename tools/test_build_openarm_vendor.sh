#!/usr/bin/env bash
set -euo pipefail

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
helper="${project_dir}/tools/build_openarm_vendor.sh"
sentinel=""
fixture_root=""

cleanup() {
  if [[ -n "${sentinel}" ]]; then
    rm -f -- "${sentinel}"
  fi
  if [[ -n "${fixture_root}" ]]; then
    rm -rf -- "${fixture_root}"
  fi
}
trap cleanup EXIT

"${helper}" --check-only >/dev/null

for forbidden in \
    "--packages-select openarm" \
    "--packages-up-to openarm" \
    "--packages-skip openarm_hardware" \
    "--base-paths /tmp" \
    "--paths /tmp" \
    "--event-handlers console_direct+"; do
  # Intentional word splitting presents the same argv a caller would use.
  # shellcheck disable=SC2086
  if "${helper}" ${forbidden} >/dev/null 2>&1; then
    echo "Helper accepted forbidden arguments: ${forbidden}" >&2
    exit 1
  fi
done

sentinel=$(mktemp "${project_dir}/extern/openarm_can/.cho-boundary-test-untracked.XXXXXX")
if "${helper}" --check-only >/dev/null 2>&1; then
  echo "Helper accepted an untracked vendor file." >&2
  exit 1
fi
rm -f -- "${sentinel}"
sentinel=""

"${helper}" --check-only >/dev/null

# Exercise revision/gitlink/dirty failures in disposable repositories. Never
# mutate the real shared submodules: developers may be using those worktrees.
fixture_root=$(mktemp -d)
fixture_project="${fixture_root}/project"
mkdir -p "${fixture_project}/tools" "${fixture_project}/extern"
cp -- "${helper}" "${fixture_project}/tools/build_openarm_vendor.sh"

git init -q "${fixture_project}"
for vendor in openarm_ros2 openarm_can; do
  vendor_dir="${fixture_project}/extern/${vendor}"
  git init -q "${vendor_dir}"
  printf 'fixture\n' >"${vendor_dir}/tracked.txt"
  printf 'ignored-*\n' >"${vendor_dir}/.gitignore"
  git -C "${vendor_dir}" add tracked.txt .gitignore
  git -C "${vendor_dir}" -c user.name=fixture -c user.email=fixture@example.invalid \
    commit -qm initial
done

ros2_sha=$(git -C "${fixture_project}/extern/openarm_ros2" rev-parse HEAD)
can_sha=$(git -C "${fixture_project}/extern/openarm_can" rev-parse HEAD)
sed -i \
  -e "s/^expected_ros2_sha=.*/expected_ros2_sha=${ros2_sha}/" \
  -e "s/^expected_can_sha=.*/expected_can_sha=${can_sha}/" \
  "${fixture_project}/tools/build_openarm_vendor.sh"
git -C "${fixture_project}" add tools/build_openarm_vendor.sh
git -C "${fixture_project}" update-index --add \
  --cacheinfo "160000,${ros2_sha},extern/openarm_ros2"
git -C "${fixture_project}" update-index --add \
  --cacheinfo "160000,${can_sha},extern/openarm_can"
git -C "${fixture_project}" -c user.name=fixture -c user.email=fixture@example.invalid \
  commit -qm fixture
fixture_helper="${fixture_project}/tools/build_openarm_vendor.sh"
"${fixture_helper}" --check-only >/dev/null

expect_fixture_rejection() {
  local case_name=$1
  if "${fixture_helper}" --check-only >/dev/null 2>&1; then
    echo "Fixture unexpectedly accepted ${case_name}." >&2
    exit 1
  fi
}

# Wrong checked-out HEAD, while the superproject gitlink remains pinned.
printf 'second\n' >>"${fixture_project}/extern/openarm_ros2/tracked.txt"
git -C "${fixture_project}/extern/openarm_ros2" add tracked.txt
git -C "${fixture_project}/extern/openarm_ros2" \
  -c user.name=fixture -c user.email=fixture@example.invalid commit -qm second
wrong_ros2_sha=$(git -C "${fixture_project}/extern/openarm_ros2" rev-parse HEAD)
expect_fixture_rejection "wrong vendor HEAD"
git -C "${fixture_project}/extern/openarm_ros2" checkout -q --detach "${ros2_sha}"

# Wrong superproject gitlink, while the checked-out HEAD remains reviewed.
git -C "${fixture_project}" update-index \
  --cacheinfo "160000,${wrong_ros2_sha},extern/openarm_ros2"
expect_fixture_rejection "wrong superproject gitlink"
git -C "${fixture_project}" update-index \
  --cacheinfo "160000,${ros2_sha},extern/openarm_ros2"

printf 'dirty\n' >>"${fixture_project}/extern/openarm_can/tracked.txt"
expect_fixture_rejection "unstaged tracked change"
git -C "${fixture_project}/extern/openarm_can" checkout -q -- tracked.txt

printf 'staged\n' >>"${fixture_project}/extern/openarm_can/tracked.txt"
git -C "${fixture_project}/extern/openarm_can" add tracked.txt
expect_fixture_rejection "staged tracked change"
git -C "${fixture_project}/extern/openarm_can" reset -q -- tracked.txt
git -C "${fixture_project}/extern/openarm_can" checkout -q -- tracked.txt

printf 'untracked\n' >"${fixture_project}/extern/openarm_can/source.tmp"
expect_fixture_rejection "non-ignored untracked file"
rm -f -- "${fixture_project}/extern/openarm_can/source.tmp"

# Ignored build artifacts are intentionally outside the source-clean policy.
printf 'artifact\n' >"${fixture_project}/extern/openarm_can/ignored-build"
"${fixture_helper}" --check-only >/dev/null

echo "OpenArm vendor helper negative tests passed."
