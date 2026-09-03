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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
