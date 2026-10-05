# Copyright (c) 2023 Franka Robotics GmbH
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
#
# Derived from franka_bringup (https://github.com/frankaemika/franka_ros2); see NOTICE.

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
  # flake8 with its plugins' rules pinned, so a machine with the ROS flake8
  # plugins installed and one without agree; see .flake8.
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
