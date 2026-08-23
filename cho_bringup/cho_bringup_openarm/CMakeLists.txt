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
project(cho_bringup_openarm)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# launch_utils is shared by every launch file. It goes to lib/ (not share/) and
# is loaded by absolute path, matching cho_bringup_franka.
install(
  DIRECTORY utils/
  DESTINATION lib/${PROJECT_NAME}/utils
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
