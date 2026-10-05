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

cmake_minimum_required(VERSION 3.8)
project(cho_camera_calibration)

find_package(ament_cmake REQUIRED)

# config/ is where a finished calibration lands, so that it can be handed to a
# driver as package://cho_camera_calibration/config/<file>.yaml rather than a path
# under someone's home directory. It starts out holding nothing but .gitkeep.
install(DIRECTORY targets config
  DESTINATION share/${PROJECT_NAME}
)
# Under lib/, so the README's `ros2 run cho_camera_calibration <script>` finds them.
install(PROGRAMS
  scripts/make_checkerboard.py
  scripts/record_board_views.py
  scripts/solve_hand_eye.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")
  ament_lint_cmake()
  # D212 (summary on the first line) and D213 (on the second) are mutually
  # exclusive; ament ignores D212, this repo writes D212. Same choice as
  # cho_object_pose/test/test_pep257.py.
  ament_pep257(--add-ignore D213)
  ament_xmllint()
endif()

ament_package()
