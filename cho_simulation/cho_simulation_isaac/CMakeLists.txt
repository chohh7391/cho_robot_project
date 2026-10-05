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
  # flake8 with its plugins' rules pinned, so a machine with the ROS flake8
  # plugins installed and one without agree; see .flake8.
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")
  ament_lint_auto_find_test_dependencies()
  ament_pep257(--add-ignore D213)
endif()

ament_package()
