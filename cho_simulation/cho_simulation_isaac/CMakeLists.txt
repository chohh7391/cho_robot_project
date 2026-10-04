cmake_minimum_required(VERSION 3.5)
project(cho_simulation_isaac)

find_package(ament_cmake REQUIRED)

# isaac/ holds scripts that run under Isaac Sim's own interpreter (python.sh),
# not under ROS, so they are installed as data rather than as executables.
install(
  DIRECTORY isaac
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/isaac_command_gate.py
  scripts/isaac_ft_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  # D212 (summary on the first line) and D213 (on the second) are mutually
  # exclusive; ament ignores D212, this repo writes D212 (as cho_oak does).
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()
  ament_pep257(--add-ignore D213)
endif()

ament_package()
