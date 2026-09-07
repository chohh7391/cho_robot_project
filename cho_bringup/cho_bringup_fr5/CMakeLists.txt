cmake_minimum_required(VERSION 3.5)
project(cho_bringup_fr5)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# launch_utils is shared by the real bringups. It goes to lib/ (not share/) and
# is loaded by absolute path, matching cho_bringup_franka and _openarm.
install(
  DIRECTORY utils/
  DESTINATION lib/${PROJECT_NAME}/utils
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  # No ament_lint_auto here, matching cho_bringup_openarm. It applies ROS's
  # default style - 99 columns and pydocstyle's D213 - while CLAUDE.md declares
  # this project's standard as 120 columns and relaxed pep257, so it was failing
  # 13 times on code that is correct by the project's own rules. package.xml is
  # still linted repo-wide by the ament_xmllint step in .github/workflows/lint.yml.
  find_package(ament_cmake_pytest REQUIRED)
  # A user-local anyio plugin targets a newer pytest than ROS 2 Humble ships.
  # Keep package tests deterministic and limited to explicitly requested tests.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(test_gripper_selection
    test/test_gripper_selection.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
