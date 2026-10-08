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
project(cho_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(MSG_FILES
  "msg/ActionChunk.msg"
  "msg/CameraVisibility.msg"
  "msg/GripperState.msg"
  "msg/ObjectVisibility.msg"
  "msg/ObjectVisibilityArray.msg"
  "msg/PoseLog.msg"
  "msg/VlaTelemetry.msg"
)

# Action
set(ACTION_FILES
  action/JointSpace.action
  action/TaskSpace.action
  action/Gripper.action
  action/VisionLanguageAction.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${ACTION_FILES}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces sensor_msgs)

install(
  DIRECTORY msg action
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
