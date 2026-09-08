# No-op stand-in for isaac_ros_common's generate_version_info().
#
# Upstream embeds a git-derived version string into the built package. Nothing in
# this project reads it, and reproducing it would mean vendoring upstream code
# into project space. See ../CMakeLists.txt for why this file exists at all.
#
# Signature must match upstream: generate_version_info(<project_name>).
function(generate_version_info PROJECT_NAME)
  message(STATUS
    "cho_moveit_curobo_deps: skipping isaac_ros_common version stamping for "
    "${PROJECT_NAME}")
endfunction()
