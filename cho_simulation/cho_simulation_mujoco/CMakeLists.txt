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

cmake_minimum_required(VERSION 3.16)
project(cho_simulation_mujoco)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
# mujoco_ros2_control_plugins exports MuJoCo (mujoco_vendor), rclcpp,
# realtime_tools and pluginlib with its target.
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/body_pose_publisher.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins
  ${geometry_msgs_TARGETS}
  ${tf2_msgs_TARGETS})

pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_clang_format(src include test)

  # Which bodies, their poses and the publishing schedule, on a model built in
  # the test: no simulation, no ROS.
  ament_add_gtest(test_scene_bodies test/test_scene_bodies.cpp)
  target_link_libraries(test_scene_bodies ${PROJECT_NAME})
endif()

ament_package()
