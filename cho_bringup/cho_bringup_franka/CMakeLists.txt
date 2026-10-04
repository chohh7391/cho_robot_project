cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(cho_bringup_franka)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# The Franka-specific launch helpers, loaded by the launch files with
# cho_bringup_common.load_package_utils('cho_bringup_franka').
# Excludes rather than FILES_MATCHING: ament's --symlink-install does not
# support FILES_MATCHING and silently falls back to copying, and a copy goes
# stale the moment the source changes.
install(DIRECTORY utils/
  DESTINATION lib/${PROJECT_NAME}/utils
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/mock_franka_gripper.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
