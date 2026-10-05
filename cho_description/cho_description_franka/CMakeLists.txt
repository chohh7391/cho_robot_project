#  Copyright (c) 2023 Franka Robotics GmbH
#  Copyright 2026 Hyunho Cho
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
#  Derived from franka_description (https://github.com/frankaemika/franka_description);
#  see NOTICE.

cmake_minimum_required(VERSION 3.5)
project(cho_description_franka)
find_package(ament_cmake REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # flake8 with its plugins' rules pinned, so a machine with the ROS flake8
  # plugins installed and one without agree; see .flake8.
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_tests test/urdf_tests.py)

endif()

# Install launch files.
install(DIRECTORY
  launch robots end_effectors meshes rviz urdf xml
  DESTINATION share/${PROJECT_NAME}/
)

# USD assets for Isaac Sim. Generated (and large), so they are not tracked in
# git -- see usd/README.md for the one-shot conversion command. Guarded so a
# fresh clone that has not run the converter yet still builds.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/usd")
  install(DIRECTORY
    usd
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()